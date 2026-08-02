"""Identity gate: torso signatures, enrollment storage, and pursuer selection."""

import numpy as np
import pytest

from app import config
from app.perception import identity
from tests.conftest import BLUE, RED, GREEN, ORANGE


@pytest.fixture
def enrolled(scene, tmp_path):
    """A database with 'charlie' (blue shirt) and 'jaafar' (red shirt)."""
    db = identity.IdentityDB(str(tmp_path))
    for name, shirt in (("charlie", BLUE), ("jaafar", RED)):
        frame, dets = scene([(shirt, 320, 160)])
        sig = identity.torso_signature(frame, dets[0].bbox)
        db.save(name, np.stack([sig] * 3))
    return identity.IdentityDB(str(tmp_path))


# ---- signatures ----

def test_same_shirt_scores_high_different_shirt_low(scene):
    frame, dets = scene([(BLUE, 320, 160)])
    base = identity.torso_signature(frame, dets[0].bbox)

    # Same shirt, moved and under a slight lighting shift.
    frame2, dets2 = scene([((190, 70, 40), 260, 150)])
    same = identity.signature_score(identity.torso_signature(frame2, dets2[0].bbox),
                                    np.stack([base]))
    frame3, dets3 = scene([(RED, 320, 160)])
    diff = identity.signature_score(identity.torso_signature(frame3, dets3[0].bbox),
                                    np.stack([base]))
    assert same > config.ID_MATCH_THRESHOLD
    assert diff < config.ID_MATCH_THRESHOLD
    assert same > diff


def test_tiny_box_yields_no_signature(scene):
    frame, _ = scene([(BLUE, 320, 160)])
    assert identity.torso_signature(frame, (0, 0, 15, 15)) is None


def test_signature_is_normalized(scene):
    frame, dets = scene([(BLUE, 320, 160)])
    sig = identity.torso_signature(frame, dets[0].bbox)
    assert sig.sum() == pytest.approx(1.0, abs=1e-4)


# ---- storage ----

def test_database_roundtrip(enrolled, tmp_path):
    assert sorted(enrolled.names()) == ["charlie", "jaafar"]
    assert len(identity.IdentityDB(str(tmp_path))) == 2


def test_gate_is_off_when_nobody_enrolled(tmp_path):
    assert len(identity.IdentityDB(str(tmp_path))) == 0


# ---- IoU helper ----

@pytest.mark.parametrize("a,b,expected", [
    ((0, 0, 10, 10), (0, 0, 10, 10), 1.0),      # identical
    ((0, 0, 10, 10), (20, 20, 30, 30), 0.0),    # disjoint
    ((0, 0, 10, 10), (5, 0, 15, 10), 1 / 3),    # half overlap
])
def test_iou(a, b, expected):
    assert identity._iou(a, b) == pytest.approx(expected)


# ---- pursuer selection (the multi-person behavior) ----

def _run(selector, scene, people, frames):
    """Feed the same synthetic scene for N frames; return the last decision."""
    person = name = None
    for _ in range(frames):
        frame, dets = scene(people)
        person, name = selector.select(frame, dets)
    return person, name


def test_stranger_alone_is_ignored(enrolled, scene):
    sel = identity.PursuerSelector(enrolled)
    person, name = _run(sel, scene, [(GREEN, 320, 260)], 8)
    assert person is None and name is None


def test_enrolled_person_is_found_despite_a_closer_stranger(enrolled, scene):
    """The regression this class of bug produced: a stranger who is nearer, or
    simply arrived first, must not hold the tracker."""
    sel = identity.PursuerSelector(enrolled)
    _run(sel, scene, [(GREEN, 320, 260)], 8)            # stranger arrives first
    person, name = _run(sel, scene,
                        [(GREEN, 300, 260), (BLUE, 560, 110)], 6)
    assert name == "charlie"
    assert person.bbox[0] > 400                          # the right-hand box


def test_picks_the_enrolled_one_among_several_strangers(enrolled, scene):
    sel = identity.PursuerSelector(enrolled)
    person, name = _run(sel, scene,
                        [(GREEN, 120, 200), (ORANGE, 330, 220), (RED, 540, 120)], 6)
    assert name == "jaafar"


def test_never_selects_a_stranger_when_enrolled_are_present(enrolled, scene):
    sel = identity.PursuerSelector(enrolled)
    _, name = _run(sel, scene,
                   [(BLUE, 160, 150), (GREEN, 330, 240), (RED, 520, 150)], 6)
    assert name in ("charlie", "jaafar")


def test_strangers_do_not_dilute_votes(enrolled, scene):
    """Per-candidate voting: recognition takes ID_MIN_VOTES frames whether or
    not strangers share the frame."""
    sel = identity.PursuerSelector(enrolled)
    _, name = _run(sel, scene,
                   [(GREEN, 200, 240), (BLUE, 520, 140)], config.ID_MIN_VOTES)
    assert name == "charlie"


def test_single_frame_cannot_decide(enrolled, scene):
    sel = identity.PursuerSelector(enrolled)
    _, name = _run(sel, scene, [(BLUE, 320, 160)], 1)
    assert name is None


def test_selector_releases_a_pursuer_who_leaves(enrolled, scene):
    sel = identity.PursuerSelector(enrolled)
    _run(sel, scene, [(BLUE, 320, 160)], 5)
    person, _ = _run(sel, scene, [(GREEN, 320, 240)],
                     config.ID_CANDIDATE_MAX_MISSED + 3)
    assert person is None


def test_walking_person_stays_one_candidate(enrolled, scene):
    """IoU association: a person crossing the frame must not spawn a new
    candidate (and a fresh vote window) every frame."""
    sel = identity.PursuerSelector(enrolled)
    name = None
    for x in range(150, 500, 40):
        frame, dets = scene([(BLUE, x, 150)])
        _, name = sel.select(frame, dets)
    assert name == "charlie"
    assert len(sel.candidates) == 1


def test_empty_frame_is_safe(enrolled, scene):
    sel = identity.PursuerSelector(enrolled)
    frame, _ = scene([])
    assert sel.select(frame, []) == (None, None)


# ---- distance + ambiguity rejection (fixes for far-away confusion) ----

def _person(scene_fn, shirt, cx=320, box_h=360):
    """One person whose apparent size is set by box_h (smaller = farther)."""
    # conftest scene() places people at fixed y=60..420 (h=360); to simulate
    # distance we shrink the detection box the signature is cropped from.
    import numpy as np
    frame = np.full((480, 640, 3), 200, np.uint8)
    x1, x2 = cx - 80, cx + 80
    y1, y2 = 60, 60 + box_h
    rng = np.random.default_rng(1)
    patch = np.array(shirt, np.float32) + rng.normal(0, 15, (y2 - y1, x2 - x1, 3))
    frame[y1:y2, x1:x2] = np.clip(patch, 0, 255).astype(np.uint8)
    return frame, (x1, y1, x2, y2)


def test_distant_person_yields_no_signature():
    """A far-away (small) person must not produce a signature at all -- this is
    the root cause of distant people matching everyone."""
    far_frame, far_bbox = _person(None, BLUE, box_h=110)     # ~23% of frame
    close_frame, close_bbox = _person(None, BLUE, box_h=360)
    assert identity.torso_signature(far_frame, far_bbox) is None
    assert identity.torso_signature(close_frame, close_bbox) is not None


def test_distant_stranger_is_not_identified(enrolled):
    sel = identity.PursuerSelector(enrolled)
    name = None
    for _ in range(6):
        frame, bbox = _person(None, GREEN, box_h=130)
        det = type("D", (), {"bbox": bbox})()
        _, name = sel.select(frame, [det])
    assert name is None


def test_margin_blocks_ambiguous_match(enrolled):
    """When two enrolled people score within the margin of each other, refuse
    to name either -- prevents flip-flopping between similar signatures."""
    cand = identity._Candidate((0, 0, 10, 10), enrolled.names(), window=8)
    # Force two near-equal average scores above threshold.
    for _ in range(4):
        for n in cand.scores:
            cand.scores[n].append(0.70)
    name, _ = cand.decide(threshold=0.62, min_votes=3, margin=0.08)
    assert name is None            # 0.70 vs 0.70 -> too close to call

    # Clear winner passes.
    cand2 = identity._Candidate((0, 0, 10, 10), enrolled.names(), window=8)
    names = enrolled.names()
    for _ in range(4):
        cand2.scores[names[0]].append(0.85)
        cand2.scores[names[1]].append(0.55)
    name, _ = cand2.decide(threshold=0.62, min_votes=3, margin=0.08)
    assert name == names[0]


# ---- track locking (commit to an identity once highly confident) ----

class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def tick(self, dt=0.1):
        self.t += dt


def _blue_det(scene, cx=320):
    frame, dets = scene([(BLUE, cx, 160)])
    return frame, dets[0]


def test_lock_engages_after_sustained_high_confidence(enrolled, scene):
    clk = _Clock()
    sel = identity.PursuerSelector(enrolled, lock_score=0.90, lock_seconds=2.0,
                                   time_fn=clk)
    for _ in range(40):                       # 4 s at 10 Hz
        frame, det = _blue_det(scene)
        sel.select(frame, [det])
        clk.tick()
    assert sel.locked_identity() == "charlie"


def test_brief_high_confidence_does_not_lock(enrolled, scene):
    clk = _Clock()
    sel = identity.PursuerSelector(enrolled, lock_score=0.90, lock_seconds=2.0,
                                   time_fn=clk)
    for _ in range(5):                        # only 0.5 s
        frame, det = _blue_det(scene)
        sel.select(frame, [det])
        clk.tick()
    assert sel.locked_identity() is None


def test_locked_identity_survives_ambiguous_frames(enrolled, scene):
    clk = _Clock()
    sel = identity.PursuerSelector(enrolled, lock_score=0.90, lock_seconds=2.0,
                                   time_fn=clk)
    for _ in range(40):
        frame, det = _blue_det(scene)
        sel.select(frame, [det])
        clk.tick()
    # Grey shirt = low, ambiguous score that pre-lock would return None.
    name = None
    for _ in range(8):
        frame, dets = scene([((120, 120, 120), 320, 160)])
        _, name = sel.select(frame, [dets[0]])
        clk.tick()
    assert name == "charlie"


def test_lock_never_commits_to_a_stranger(enrolled, scene):
    clk = _Clock()
    sel = identity.PursuerSelector(enrolled, lock_score=0.90, lock_seconds=2.0,
                                   time_fn=clk)
    name = None
    for _ in range(40):
        frame, dets = scene([((120, 120, 120), 320, 160)])   # grey stranger
        _, name = sel.select(frame, [dets[0]])
        clk.tick()
    assert sel.locked_identity() is None
    assert name is None


def test_lock_releases_when_track_is_lost(enrolled, scene):
    import numpy as np
    clk = _Clock()
    sel = identity.PursuerSelector(enrolled, lock_score=0.90, lock_seconds=2.0,
                                   time_fn=clk, max_missed=15)
    for _ in range(40):
        frame, det = _blue_det(scene)
        sel.select(frame, [det])
        clk.tick()
    assert sel.locked_identity() == "charlie"
    blank = np.full((480, 640, 3), 200, np.uint8)
    for _ in range(20):
        sel.select(blank, [])
        clk.tick()
    assert sel.locked_identity() is None
