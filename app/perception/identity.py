"""Target re-identification: only flee ENROLLED pursuers (the team), ignore others.

Implements the proposal's "target re-identification" nice-to-have with zero extra
models: the torso region of the detected person's bounding box (their shirt) is
summarized as a smoothed Hue-Saturation histogram and matched against enrolled
samples, with a short temporal voting window so one noisy frame cannot flip the
identity. Costs well under a millisecond per frame -- it will not touch the
10 FPS week-1 gate.

Multi-candidate selection: every person the detector finds is scored each
frame, so an enrolled pursuer is picked up even when a stranger is closer,
larger, or arrived first. Votes accumulate PER CANDIDATE (tracks are
associated frame-to-frame by bbox overlap), so a stranger in frame can never
dilute an enrolled person's score.

Enrollment: python -m scripts.enroll --name charlie   (see scripts/enroll.py)
Signatures are stored one .npz per person under IDENTITY_DIR (config.py).
Re-enroll whenever you change clothes; enroll in the demo room, demo clothes.

Known limitations (state in the report): this identifies CLOTHING, not faces --
two pursuers in near-identical shirts are indistinguishable, and extreme
lighting changes lower scores. Both are acceptable for a chase robot: faces are
unusable at chase distance / from behind, which is exactly what this camera sees.
"""

import os
import time
from collections import deque

import cv2
import numpy as np

from .. import config


def torso_signature(frame, bbox):
    """Appearance signature of the person's torso, or None if the crop is unusable.

    The torso is taken as a fixed sub-rectangle of the person bbox (fractions in
    config.py): vertically the shoulders-to-hips band, horizontally the central
    strip, so we sample shirt rather than arms or background.

    Returns a flattened, L1-normalized, smoothed HS histogram (float32).
    """
    x_min, y_min, x_max, y_max = bbox
    w, h = x_max - x_min, y_max - y_min
    x1 = int(x_min + config.ID_TORSO_X_FRAC * w)
    x2 = int(x_max - config.ID_TORSO_X_FRAC * w)
    y1 = int(y_min + config.ID_TORSO_Y_TOP_FRAC * h)
    y2 = int(y_min + config.ID_TORSO_Y_BOT_FRAC * h)

    fh, fw = frame.shape[:2]
    x1, x2 = max(0, x1), min(fw, x2)
    y1, y2 = max(0, y1), min(fh, y2)
    if (x2 - x1) < config.ID_MIN_PATCH_PX or (y2 - y1) < config.ID_MIN_PATCH_PX:
        return None
    # Distance guard: the WHOLE person must be a reasonable fraction of frame
    # height. A tiny far-away person yields a few-pixel torso whose histogram
    # is mostly noise and matches everyone -- refuse to sign it at all.
    if (y_max - y_min) < config.ID_MIN_PERSON_H_FRAC * fh:
        return None

    hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
    # Very dark pixels carry no reliable hue; mask them out.
    mask = cv2.inRange(hsv, (0, 0, 40), (180, 255, 255))
    # Need enough coloured shirt pixels for the histogram to mean anything.
    if int(cv2.countNonZero(mask)) < config.ID_MIN_SHIRT_PX:
        return None
    hist = cv2.calcHist([hsv], [0, 1], mask,
                        [config.ID_H_BINS, config.ID_S_BINS],
                        [0, 180, 0, 256])
    # Smooth before normalizing: a solid-color shirt is otherwise a single-bin
    # spike, and the SAME shirt under slightly different light lands one bin
    # over and correlates near zero. Blurring makes close shades correlate
    # highly while distant colors still do not.
    hist = cv2.GaussianBlur(hist, (7, 7), 1.5)
    total = hist.sum()
    if total < 1e-6:
        return None
    return (hist / total).astype(np.float32).ravel()


def signature_score(sig, enrolled):
    """Best histogram correlation between sig and any enrolled sample.

    Correlation is in [-1, 1]; ~1 = same shirt, ~0 = unrelated color.
    """
    best = -1.0
    a = sig.reshape(config.ID_H_BINS, config.ID_S_BINS)
    for row in enrolled:
        b = row.reshape(config.ID_H_BINS, config.ID_S_BINS)
        best = max(best, float(cv2.compareHist(a, b, cv2.HISTCMP_CORREL)))
    return best


class IdentityDB:
    """Enrolled signatures, one compressed .npz per person."""

    def __init__(self, dir_path=None):
        self.dir = dir_path or config.IDENTITY_DIR
        os.makedirs(self.dir, exist_ok=True)
        self.people = {}
        self.load()

    def load(self):
        self.people.clear()
        for fn in sorted(os.listdir(self.dir)):
            if fn.endswith(".npz"):
                data = np.load(os.path.join(self.dir, fn))
                self.people[fn[:-4]] = data["signatures"]
        return self

    def save(self, name, signatures):
        path = os.path.join(self.dir, f"{name}.npz")
        arr = np.asarray(signatures, np.float32)
        np.savez_compressed(path, signatures=arr)
        self.people[name] = arr
        return path

    def names(self):
        return list(self.people.keys())

    def __len__(self):
        return len(self.people)


class IdentityMatcher:
    """Decides WHO the tracked person is, voting over a sliding window.

    update() once per frame with that frame's signature; decide() returns
    (name, mean_score) once enough votes agree above the threshold, else
    (None, best_score). A single frame can never flip the decision.
    """

    def __init__(self, db, threshold=None, window=None, min_votes=None):
        self.db = db
        self.threshold = threshold or config.ID_MATCH_THRESHOLD
        window = window or config.ID_VOTE_WINDOW
        self.min_votes = min_votes or config.ID_MIN_VOTES
        self.scores = {n: deque(maxlen=window) for n in db.names()}

    def reset(self):
        for d in self.scores.values():
            d.clear()

    def update(self, sig):
        if sig is None:
            return
        for name, enrolled in self.db.people.items():
            self.scores[name].append(signature_score(sig, enrolled))

    def decide(self):
        best_name, best_mean = None, -1.0
        for name, d in self.scores.items():
            if len(d) >= self.min_votes:
                m = float(np.mean(d))
                if m > best_mean:
                    best_name, best_mean = name, m
        if best_name is not None and best_mean >= self.threshold:
            return best_name, best_mean
        return None, best_mean


def _iou(a, b):
    """Intersection-over-union of two (x_min, y_min, x_max, y_max) boxes."""
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0


class _Candidate:
    """One person being scored across frames, with their own vote history.

    A candidate can become LOCKED: once its best identity has stayed above a
    high confidence bar continuously for a set time, we commit to that identity
    and stop re-questioning it. From then on the candidate always reports that
    name (as long as the track survives, associated frame-to-frame by IoU) --
    which gives the servo camera a stable target instead of a label that
    flickers when the shirt momentarily looks ambiguous.
    """

    __slots__ = ("bbox", "scores", "missed", "locked_name", "_streak_name",
                 "_streak_since")

    def __init__(self, bbox, names, window):
        self.bbox = bbox
        self.scores = {n: deque(maxlen=window) for n in names}
        self.missed = 0
        self.locked_name = None      # committed identity, or None
        self._streak_name = None     # who is currently on a high-conf streak
        self._streak_since = None    # time.time() the streak began

    def vote(self, sig, people):
        if sig is None:
            return
        for name, enrolled in people.items():
            self.scores[name].append(signature_score(sig, enrolled))

    def _ranked(self, min_votes):
        ranked = []
        for name, d in self.scores.items():
            if len(d) >= min_votes:
                ranked.append((float(np.mean(d)), name))
        ranked.sort(reverse=True)
        return ranked

    def update_lock(self, min_votes, lock_score, lock_seconds, now):
        """Promote to a permanent lock once an identity holds a high score long
        enough. No-op if already locked."""
        if self.locked_name is not None:
            return
        ranked = self._ranked(min_votes)
        if ranked and ranked[0][0] >= lock_score:
            name = ranked[0][1]
            if name != self._streak_name:
                self._streak_name, self._streak_since = name, now
            elif now - self._streak_since >= lock_seconds:
                self.locked_name = name          # commit permanently
        else:
            self._streak_name = self._streak_since = None

    def decide(self, threshold, min_votes, margin=0.0):
        """Return (name, score). A locked candidate always returns its committed
        identity. Otherwise the winner must clear `threshold` AND beat the
        second-best enrolled person by at least `margin`; else the frame is
        ambiguous (distant / similar shirts) and we name no one."""
        if self.locked_name is not None:
            d = self.scores.get(self.locked_name)
            score = float(np.mean(d)) if d and len(d) else 1.0
            return self.locked_name, score
        ranked = self._ranked(min_votes)
        if not ranked:
            return None, -1.0
        best_mean, best_name = ranked[0]
        if best_mean < threshold:
            return None, best_mean
        if len(ranked) > 1 and (best_mean - ranked[1][0]) < margin:
            return None, best_mean   # too close to call
        return best_name, best_mean


class PursuerSelector:
    """Picks which detected person to flee: the best-matching ENROLLED one.

    Scores EVERY detection each frame rather than only the most prominent, so
    an enrolled pursuer is found even if a stranger is nearer the camera or
    appeared first. Each candidate keeps its own voting window, associated
    across frames by bbox IoU, so a stranger's low scores never contaminate an
    enrolled person's average.

    select(frame, detections) -> (Detection, name) or (None, None).
    """

    def __init__(self, db, threshold=None, window=None, min_votes=None,
                 iou_match=None, max_missed=None, margin=None,
                 lock_score=None, lock_seconds=None, time_fn=None):
        self.db = db
        self.threshold = threshold if threshold is not None else config.ID_MATCH_THRESHOLD
        self.window = window if window is not None else config.ID_VOTE_WINDOW
        self.min_votes = min_votes if min_votes is not None else config.ID_MIN_VOTES
        self.iou_match = iou_match if iou_match is not None else config.ID_IOU_MATCH
        self.max_missed = max_missed if max_missed is not None else config.ID_CANDIDATE_MAX_MISSED
        self.margin = margin if margin is not None else config.ID_MATCH_MARGIN
        self.lock_score = lock_score if lock_score is not None else config.ID_LOCK_SCORE
        self.lock_seconds = lock_seconds if lock_seconds is not None else config.ID_LOCK_SECONDS
        self._now = time_fn or time.monotonic     # injectable for tests
        self.candidates = []

    def locked_identity(self):
        """The committed identity of the currently tracked pursuer, or None."""
        for cand in self.candidates:
            if cand.locked_name is not None:
                return cand.locked_name
        return None

    def reset(self):
        self.candidates = []

    def _associate(self, bbox):
        """Find the candidate whose last bbox best overlaps this detection."""
        best, best_iou = None, self.iou_match
        for cand in self.candidates:
            score = _iou(cand.bbox, bbox)
            if score >= best_iou:
                best, best_iou = cand, score
        return best

    def select(self, frame, detections):
        """Vote on every detection, then return the best enrolled pursuer."""
        names = self.db.names()
        seen = []

        for det in detections:
            cand = self._associate(det.bbox)
            if cand is None:
                cand = _Candidate(det.bbox, names, self.window)
                self.candidates.append(cand)
            cand.bbox = det.bbox
            cand.missed = 0
            cand.vote(torso_signature(frame, det.bbox), self.db.people)
            seen.append((cand, det))

        # Age out candidates that were not matched this frame.
        for cand in self.candidates:
            if not any(cand is c for c, _ in seen):
                cand.missed += 1
        self.candidates = [c for c in self.candidates
                           if c.missed <= self.max_missed]

        # Promote sustained high-confidence candidates to a permanent lock.
        now = self._now()
        for cand, _ in seen:
            cand.update_lock(self.min_votes, self.lock_score,
                             self.lock_seconds, now)

        # Choose the enrolled candidate with the strongest agreement. A locked
        # candidate reports its committed identity and always wins ties, so the
        # servo keeps following the same person once we are sure.
        best_det, best_name, best_score = None, None, -1.0
        for cand, det in seen:
            name, score = cand.decide(self.threshold, self.min_votes,
                                      self.margin)
            if name is None:
                continue
            priority = score + (1.0 if cand.locked_name is not None else 0.0)
            if priority > best_score:
                best_det, best_name, best_score = det, name, priority
        return best_det, best_name
