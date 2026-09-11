"""Frozen-policy behavioural snapshot — the guard before crossing into hardware.

Opt-in (slow): needs the three local reference tracks (`media/` is gitignored)
and loads CLAP.

    pytest -m slow tests/test_policy_snapshot.py

`test_translation.py` and `test_physical_scheduler.py` pin the *rules* and the
*gate semantics*. This pins the *emergent behaviour*: for each reference track,
the dominant motion family, the command rate, and the mean intensity must stay
in a band. If one drifts out, the policy changed in a way the unit tests don't
see — re-run `python -m analysis.choreo_compare media/track_{A,B,C}.wav`, decide
whether it was intended, and only then re-baseline the bands here.

Bands measured 2026-08-31 on tag `policy-v1` (min_interval 400 / slam 650,
energy-capped DROP), generous ± around the observed run.
"""

from collections import Counter
from pathlib import Path

import pytest

pytest.importorskip("torch")

pytestmark = pytest.mark.slow

_MEDIA = Path(__file__).resolve().parent.parent / "media"

# family: (dominant family, minimum window share)
# rate:   (cmd/min low, high)
# inten:  (mean intensity low, high)
_SNAPSHOT = {
    "track_A.wav": {"family": ("SLAM", 0.80), "rate": (78, 105), "inten": (0.78, 0.95)},
    "track_B.wav": {"family": ("SWAY", 0.80), "rate": (58, 90),  "inten": (0.38, 0.58)},
    "track_C.wav": {"family": ("SWAY", 0.85), "rate": (75, 112), "inten": (0.44, 0.66)},
}


@pytest.mark.parametrize("name, want", _SNAPSHOT.items())
def test_reference_track_behaviour_within_band(name, want):
    path = _MEDIA / name
    if not path.exists():
        pytest.skip(f"{name} not present (media/ is gitignored)")

    from analysis.choreo_compare import run_track

    r = run_track(str(path), encoder="clap", seconds=120)
    windows = Counter(r["fam_windows"])
    total = sum(windows.values()) or 1
    dom, dom_n = windows.most_common(1)[0]
    cmds = r["sem_cmds"]
    assert cmds, f"{name}: policy produced no commands"
    rate = len(cmds) / (r["span_s"] / 60.0)
    inten = sum(i for *_, i in cmds) / len(cmds)

    want_fam, want_share = want["family"]
    assert dom == want_fam, f"{name}: dominant family {dom!r}, expected {want_fam!r}"
    assert dom_n / total >= want_share, (
        f"{name}: {want_fam} share {dom_n / total:.2f} < {want_share}")
    lo, hi = want["rate"]
    assert lo <= rate <= hi, f"{name}: {rate:.1f} cmd/min outside [{lo}, {hi}]"
    lo, hi = want["inten"]
    assert lo <= inten <= hi, f"{name}: mean intensity {inten:.2f} outside [{lo}, {hi}]"
