"""Real-time visualizer: audio -> perception -> command -> drone, all on screen.

Left: a 2D drone driven by the exact channel state `FakeArduino` receives.
Right: perception telemetry.  Bottom: a scrolling timeline lining up beats,
predicted beats, and when commands actually fired.

    python -m host.sim.visualizer --source synth
    python -m host.sim.visualizer --source clip.wav                  # plays clip.wav over speakers too
    python -m host.sim.visualizer --source clip.wav --no-play        # silent
    python -m host.sim.visualizer --source mic --port /dev/ttyUSB0   # drive a real drone too

The audio pipeline + controller run on a background thread; the pygame loop
renders at 60 fps off the latest state. wav sources also play out loud via
sounddevice, started at the same moment frame pacing begins so it stays in
sync with the visuals (mic sources never do, to avoid feedback).
"""

from __future__ import annotations

import argparse
import os
import threading
import time
from collections import deque

from host.audio.capture import DEFAULT_BLOCK_SIZE, DEFAULT_SAMPLE_RATE
from host.audio.features import FeatureExtractor
from host.audio.sources import open_source, wav_signal
from host.control.link import SerialLink
from host.control.motion_policy import MotionPolicy, _family
from host.control.motion_primitives import ACT_NAMES, expand
from host.control.music_state import MusicStateEstimator
from host.control.scheduled_controller import ScheduledController
from host.control.scheduler import DroneScheduler
from host.sim import panels
from host.sim.drone_2d import Drone2D
from host.sim.fake_arduino import FakeArduino

WIN_W, WIN_H = 980, 660

# scheduler drop reasons worth a timeline mark (min_interval is routine, skipped)
_MARK_DROPS = {"cooldown", "conflict", "repeat", "too_long"}


class Visualizer:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.frames, self.sr, self.src_desc = open_source(
            args.source, rate=args.rate, block=args.block, backend=args.backend,
            device=args.device, bpm=args.bpm, seconds=args.seconds,
            realtime=True,
        )
        self.play_sig = None
        self.play_sr = None
        if not args.no_play and not args.screenshot and args.source not in ("mic", "synth"):
            # wav sources: reload for speaker playback (mic would feed back;
            # synth is a click track, not worth playing). open_source() used
            # the same wav_signal() at the same sample rate, so timing lines up.
            self.play_sig, self.play_sr = wav_signal(args.source)

        self.fx = FeatureExtractor(self.sr, args.block)
        self.est = MusicStateEstimator()
        self.policy = MotionPolicy()
        self.model = None
        self.model_threaded = False
        if args.model:
            enc = None
            if args.encoder == "clap":
                from host.audio.encoders import ClapEncoder
                enc = ClapEncoder(device=args.device_model)
            if args.model_sync:
                from host.audio.audio_model import AudioModel
                self.model = AudioModel(sample_rate=self.sr, device=args.device_model, encoder=enc)
            else:
                from host.audio.audio_model import ThreadedAudioModel
                self.model = ThreadedAudioModel(
                    sample_rate=self.sr, device=args.device_model, encoder=enc,
                    inference_delay_s=args.inject_delay_ms / 1000.0,
                )
                self.model_threaded = True

        self.own_sim = args.port is None
        self.fa = FakeArduino() if self.own_sim else None
        port = self.fa.start() if self.own_sim else args.port
        # short resync interval doubles as a link keepalive (watchdog is 500 ms)
        self.ctl = ScheduledController(DroneScheduler(SerialLink(port), resync_interval=0.35))
        self.drone = Drone2D()
        self.timeline = panels.Timeline(window_s=args.window)

        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._t_start = time.monotonic()
        self.latest_f = None
        self.latest_state = None
        self.latest_musical = None
        self.log: deque[tuple[str, tuple]] = deque(maxlen=80)
        # latched decision state, so the ~2 Hz readout doesn't flicker at 60 fps
        self.hud_fam = "-"
        self.hud_prim = "-"
        self.hud_int: float | None = None
        self.hud_dec: tuple | None = None      # ("emit", ops, env) | ("reject", reason)
        self.hud_dec_t = 0.0
        self._plans: list = []
        self._marked: set[int] = set()
        self._last_predict = 0.0
        self._last_section_seq = -1
        self._t0: float | None = None      # first frame's audio clock, for the TIME readout

    # -- audio / control thread ------------------------------------
    def _audio_loop(self) -> None:
        ctl = self.ctl
        if self.play_sig is not None:
            import sounddevice as sd
            sd.play(self.play_sig, self.play_sr)
        for frame in self.frames:
            if self._stop.is_set():
                break
            f = self.fx.push(frame)
            if self._t0 is None:
                self._t0 = f.t
            st = self.est.update(f)
            if self.model_threaded:
                self.model.push(frame)                     # non-blocking
                musical = self.model.latest()
            else:
                musical = self.model.push(frame) if self.model else None
            if musical is not None:
                self.latest_musical = musical
                if musical.section_change and musical.seq != self._last_section_seq:
                    self._last_section_seq = musical.seq
                    self.timeline.push_mark(musical.t, "section", musical.texture[:4])
            now = time.monotonic()
            nb = self.fx.detector.predict_next_beat(now, min_lead=0.08)
            at = nb if nb else now + self.args.lookahead

            # the render thread also touches the scheduler (cooldowns / running
            # set), so run select() under the same lock.
            with self.lock:
                cmd = self.policy.select(f, st, self.latest_musical)
                self.latest_f, self.latest_state = f, st
                self._record_decision(f)
                self.timeline.push_sample(f.t, f.energy)
                if f.beat:
                    self.timeline.push_mark(f.t, "beat")
                if nb and abs(nb - self._last_predict) > 0.05:
                    self.timeline.push_mark(nb, "predict")
                    self._last_predict = nb

            plan = None
            if cmd and ctl.clock is not None and not ctl.safe_hold:
                plan = ctl.perform(cmd, at=at)
            ctl.service()

            with self.lock:
                if plan:
                    self._plans.append(plan)
                self._collect_fire_marks()

    def _record_decision(self, f) -> None:
        """Latch + log the candidate the policy just produced and the scheduler's ruling."""
        cand = self.policy.last_candidate
        dec = self.policy.last_decision
        if cand is None:
            return
        m = self.latest_musical
        self.hud_fam = _family(m.texture, m.energy) if m else "-"
        self.hud_prim = cand.primitive
        self.hud_int = cand.intensity
        self.hud_dec_t = f.t
        ts = f"{f.t - (self._t0 or f.t):6.1f}"
        if dec is not None and dec.emit:
            ops, env = expand(cand)
            self.hud_dec = ("emit", ops, env)
            chan = " ".join(f"{ACT_NAMES.get(o.act, o.act)} {o.dur_ms}" for o in ops)
            self.log.append((f"{ts}  {self.hud_fam}/{cand.primitive}  ->  {chan}", panels.INK))
        else:
            reason = dec.reason if dec else "?"
            self.hud_dec = ("reject", reason)
            self.log.append((f"{ts}  {cand.primitive}  REJECTED  ({reason})", panels.REJECT))
            if reason in _MARK_DROPS:
                self.timeline.push_mark(f.t, "rej", reason[:4])

    def _collect_fire_marks(self) -> None:
        for plan in list(self._plans):
            if plan.grp in self._marked:
                if plan.status in ("released", "rejected"):
                    self._plans.remove(plan)
                continue
            ev0 = plan.events[0] if plan.events else None
            if ev0 and ev0.fired_ms is not None and self.ctl.clock is not None:
                self.timeline.push_mark(self.ctl.clock.to_host(ev0.fired_ms), "fire", plan.primitive[:4])
                self._marked.add(plan.grp)
            elif plan.status == "rejected":
                reason = plan.events[0].reason if plan.events else "?"
                self.timeline.push_mark(plan.at or time.monotonic(), "rej", reason[:4])
                self._marked.add(plan.grp)

    # -- setup / teardown ----------------------------------------
    def _connect(self) -> None:
        self.ctl.open()
        self.ctl.scheduler.link.drain()
        self.ctl.handshake(sync_probes=8)
        self.ctl.arm()
        if self.model_threaded:
            self.model.start()

    def close(self) -> None:
        self._stop.set()
        try:
            if self.model_threaded:
                self.model.stop()
            self.ctl.close()
        finally:
            if self.play_sig is not None:
                import sounddevice as sd
                sd.stop()
            if self.fa:
                self.fa.stop()

    # -- render loop -------------------------------------------
    def run(self) -> None:
        import pygame

        pygame.init()
        pygame.display.set_caption("acoustic drone - perception / motion")
        screen = pygame.display.set_mode((WIN_W, WIN_H))
        clock = pygame.time.Clock()

        drone_rect = pygame.Rect(0, 0, 440, 430)
        tele_rect = pygame.Rect(440, 0, WIN_W - 440, 430)
        time_rect = pygame.Rect(0, 430, WIN_W, 140)
        log_rect = pygame.Rect(0, 570, WIN_W, WIN_H - 570)

        self._connect()
        audio = threading.Thread(target=self._audio_loop, daemon=True)
        audio.start()

        title = panels._font(13)
        self._t_start = last = time.monotonic()
        running = True
        while running and audio.is_alive():
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                    running = False
                elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_SPACE:
                    self.ctl.stop()

            now = time.monotonic()
            dt, last = now - last, now
            ch = dict(self.fa.pins) if self.fa else {}
            self.drone.step(dt, ch)

            screen.fill(panels.BG)
            pygame.draw.rect(screen, panels.PANEL, drone_rect)
            self.drone.draw(screen, drone_rect, ch)
            with self.lock:
                f, st = self.latest_f, self.latest_state
                t_audio = f.t if f else 0.0
                panels.draw_telemetry(
                    screen, tele_rect,
                    t=t_audio - (self._t0 or t_audio), bpm=(f.bpm if f else None),
                    beat=(f.beat if f else False), musical=self.latest_musical,
                    family=self.hud_fam, primitive=self.hud_prim, intensity=self.hud_int,
                    decision=self.hud_dec, dec_age=(t_audio - self.hud_dec_t),
                    sched=self.policy.scheduler, now=t_audio,
                    model_ms=(self.model.infer_ms if self.model_threaded else None),
                    model_stale_ms=((t_audio - self.latest_musical.t) * 1000
                                    if self.latest_musical else None),
                )
                panels.draw_command_log(screen, log_rect, self.log)
                self.timeline.draw(screen, time_rect, now)
            screen.blit(title.render(self.src_desc, True, panels.DIM), (12, 8))
            if self.ctl.safe_hold:
                screen.blit(panels._font(16, True).render("SAFE HOLD", True, panels.REJECT), (12, 26))

            pygame.display.flip()
            if self.args.screenshot and now - self._t_start > self.args.screenshot_after:
                pygame.image.save(screen, self.args.screenshot)
                running = False
            clock.tick(60)

        self.close()
        pygame.quit()


def _parse(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", default="synth", help="'synth', 'mic', or a .wav path")
    p.add_argument("--backend", default="auto", choices=["auto", "sounddevice", "arecord"])
    p.add_argument("--device", default=None)
    p.add_argument("--rate", type=int, default=DEFAULT_SAMPLE_RATE)
    p.add_argument("--block", type=int, default=DEFAULT_BLOCK_SIZE)
    p.add_argument("--bpm", type=float, default=120.0)
    p.add_argument("--seconds", type=float, default=45.0, help="synth length / screenshot delay")
    p.add_argument("--window", type=float, default=8.0, help="timeline width in seconds")
    p.add_argument("--lookahead", type=float, default=0.12, help="fallback aim when tempo isn't locked")
    p.add_argument("--model", action="store_true", help="run the GPU/DALI audio model branch")
    p.add_argument("--model-sync", action="store_true", help="run the model inline instead of on its own thread")
    p.add_argument("--encoder", default="randproj", choices=["randproj", "clap"],
                   help="audio encoder (clap downloads ~2 GB on first run)")
    p.add_argument("--device-model", default="auto", help="'auto' | 'cuda' | 'cpu' for the model")
    p.add_argument("--inject-delay-ms", type=float, default=0.0,
                   help="fake inference latency in the model worker (decoupling demo)")
    p.add_argument("--port", default=None, help="drive a real Arduino instead of the sim")
    p.add_argument("--no-play", action="store_true",
                   help="don't play the source over speakers (wav sources play by default)")
    p.add_argument("--screenshot", default=None, help="headless: save a PNG then exit")
    p.add_argument("--screenshot-after", type=float, default=6.0, help="seconds to run before the PNG")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = _parse(argv)
    if args.screenshot:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    viz = Visualizer(args)
    try:
        viz.run()
    except KeyboardInterrupt:
        viz.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
