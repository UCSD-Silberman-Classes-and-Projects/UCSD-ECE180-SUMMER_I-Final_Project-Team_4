# Tuning the identity gate (confusing other people with you)

Symptom: strangers get identified as you, *especially far away*. Root cause: a
distant person is only a few torso pixels, so their colour histogram is noisy
and matches loosely -- and the old logic accepted any match over a single
threshold, even a weak one with nothing to compare against.

Four defences now exist, all in `app/config.py`. Tune in this order, one at a
time, watching `python -m scripts.vision_preview` (rejected people are now
labelled *"too far (23%)"* or *"not you? (0.58)"* so you can see which guard
should fire).

1. **`ID_MIN_PERSON_H_FRAC` (0.35)** -- the big one for distance. A person must
   fill at least this fraction of frame height to be identified at all; smaller
   people are ignored, not guessed. If distant strangers still match, RAISE
   toward 0.45 (identify only closer people). If you need longer range, lower
   toward 0.25 and lean on the guards below.

2. **`ID_MATCH_MARGIN` (0.08)** -- the winner must beat the runner-up enrolled
   person by this much, else "too close to call". Raise toward 0.15 if you and
   your partner get swapped; this is what stops flip-flopping between similar
   shirts.

3. **`ID_MATCH_THRESHOLD` (0.62)** -- absolute minimum correlation. Raise toward
   0.72 to reject weak matches generally (stricter, but you must be well-lit and
   facing similar to enrollment). Lower if YOU stop being recognised.

4. **`ID_MIN_SHIRT_PX` (400)** / **`ID_MIN_VOTES` (3)** / **`ID_VOTE_WINDOW`
   (8)** -- secondary. More shirt pixels and more required votes = steadier but
   slower to lock on. Raise `ID_MIN_VOTES` to 5 if single bad frames cause blips.

## Beyond tuning

The gate matches CLOTHING colour. Two things make it dramatically more reliable
than any parameter change:

* **Wear a distinctive, saturated shirt** for the demo -- bright orange/green,
  not grey/black/navy (dark, low-saturation shirts all look alike to an HS
  histogram, which is why navy vs. blue is hard).
* **Enroll in the demo room, in demo clothes, at demo distances.** Walk the
  actual range you'll be chased over during enrollment so the samples cover it.

If clothing-colour genuinely is not enough for your scene, the next step up is a
face-recognition lock at close range (OpenCV YuNet+SFace) fused with this colour
gate -- more code and only useful when the pursuer is near and frontal, but
available if needed.

## Track locking (committing to a pursuer)

Once a candidate holds a very high score (`ID_LOCK_SCORE`, default 0.90) for a
sustained time (`ID_LOCK_SECONDS`, default 2.0 s), the gate LOCKS onto that
identity: it commits permanently for the life of that track and stops
re-questioning it, even if later frames look ambiguous. This gives the servo
camera a stable target -- once it is sure a person is jaafar, it keeps calling
them jaafar and keeps tracking them, instead of dropping the label whenever the
shirt momentarily looks uncertain (e.g. side-on, partial occlusion, a shadow).

The track is held together frame-to-frame by bbox IoU, so the lock follows the
person as they move. It releases only when the track is lost -- the person
leaves frame for more than `ID_CANDIDATE_MAX_MISSED` frames -- at which point
the gate is free to lock onto whoever it next becomes sure of.

Safety: locking requires clearing the HIGH `ID_LOCK_SCORE` bar continuously, so
a stranger (who never scores that high) can never be locked, and a brief lucky
frame is not enough. Tune:

* raise `ID_LOCK_SECONDS` (e.g. 3.0) or `ID_LOCK_SCORE` (e.g. 0.94) to be more
  cautious before committing -- fewer false locks, slower to commit;
* lower them to commit faster once you trust enrollment.

The main loop can read `selector.locked_identity()` to tell the servo which
target to keep centred. `scripts/vision_preview.py` shows `[LOCKED] name` on the
box once commitment happens, so you can watch it engage.

## Bigger enrollment set

`scripts/enroll.py` now captures 120 samples (was 40) and walks you through
four conditions during capture -- normal distance, side profiles, farther away,
and moving through different lighting. A larger, more VARIED set is the single
best way to raise day-to-day confidence, because the runtime score is the best
match against any enrolled sample: the more angles/distances/lighting you
enrolled, the more often a live frame has a close match. Re-enroll in demo
clothes, in the demo room.
