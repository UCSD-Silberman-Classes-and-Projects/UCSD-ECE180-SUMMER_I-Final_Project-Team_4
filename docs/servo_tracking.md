# Camera pan servo: search, track, and how bearing is computed

The Logitech C920 rides on an HS-225MG pan servo. The camera answers *which
direction* the pursuer is; the two LD19 LiDARs answer *how far*. The
ultrasonic sensor is no longer used.

## The angle chain

A bearing measured in the frame is relative to where the camera is *pointing*,
not to the chassis, so three conversions happen every iteration
(`app/perception/geometry.py`):

1. **pixels -> camera angle** — `bbox_to_camera_bearing_deg()` maps the box
   centre to degrees off the optical axis using the true pinhole (tangent)
   mapping and the C920's 70.42 deg HFOV. A linear mapping is 2-3 deg off near
   the frame edges, and that error would feed straight into servo aim.
2. **camera angle -> chassis angle** — `camera_to_robot_bearing()` adds where
   the servo is pointing: `SERVO_DIR * (servo_deg - SERVO_CENTER_DEG) +
   camera_bearing`. 0 deg is dead ahead, positive is the robot's right.
3. **chassis angle -> steering** — `robot_bearing_to_steer_signal()` produces
   the value `evasion.escape()` consumes.

Step 3 is not the obvious `-bearing/180`. The robot wants to point *directly
away* from the pursuer, so it steers on the error between where it faces and
where it wants to face:

| pursuer bearing | desired action | heading |
|---|---|---|
| 0 deg (dead ahead) | spin away | hard turn |
| +90 (right) | turn left | -1 |
| 180 (behind) | already fleeing, drive straight | ~0 |
| -90 (left) | turn right | +1 |

The naive formula gets the *behind* case badly wrong — it orders a hard turn
while the robot is already pointed correctly. All four cases are unit-tested
in `tests/test_servo_geometry.py`.

## Search and track

`app/control/scanner.py` is a two-state machine:

* **SEARCHING** — step the servo `SCAN_STEP_DEG` per iteration, ping-ponging
  between the limits, running detection and the identity gate every frame.
  Strangers never end the sweep; only an enrolled person does.
* **TRACKING** — proportionally steer the servo to null out the pursuer's
  off-centre error (`TRACK_GAIN`, with a `TRACK_DEADZONE_DEG` so it does not
  twitch). If they vanish, hold position for `TRACK_LOST_FRAMES` — brief
  dropouts are normal — then resume sweeping **from the current angle**, so
  re-acquisition starts near where they were last seen.

While searching, or during a dropout, the scanner reports bearing `None`, not
`0`. Reporting 0 would read as "pursuer dead ahead" and trigger a spin.

## Coverage, and the blind arc behind

    servo sweep      160 deg (10..170)
    camera HFOV      70 deg
    total coverage   230 deg  -> +/-115 deg from the nose
    BLIND            130 deg directly behind

**This matters for an evasion robot**: when fleeing works, the pursuer ends up
*behind* you, which is exactly the arc the camera cannot see. Three ways to
handle it, in increasing effort:

1. **Lean on the LiDAR** (current design). The LiDAR is 360 deg, so once the
   pursuer leaves the camera's arc the robot keeps fleeing on its last known
   bearing while `range_at_bearing()` still reports how close the nearest
   thing behind is. Simple, no hardware change.
2. **Mount the camera facing backward** (`SERVO_CENTER_DEG` pointing aft).
   Covers the chase geometry directly, at the cost of not seeing where you are
   driving — acceptable because obstacle avoidance is the LiDAR's job anyway.
3. **Continuous-rotation servo or a second camera.** Most work; only worth it
   if 1 and 2 both prove inadequate in testing.

Option 1 is what the code does today. If testing shows the robot losing the
pursuer the moment it successfully flees, try option 2 — it is a one-line
config change plus remounting the bracket.

## Firmware side

`sketch/src/servo_cam.{h,cpp}` owns the servo. The MPU only ever posts a
*target*; the firmware slews toward it at `SERVO_SLEW_DEG_PER_STEP` per
`loop()`, so pans stay smooth and a slow vision frame cannot produce a jerky
sweep that blurs the next frame.

The MPU does **not** read the angle back each iteration — that would be a
blocking Bridge round-trip inside a 15 FPS control loop. It tracks the
commanded angle itself, which means the reported bearing lags slightly during
fast pans. That is why `TRACK_GAIN` is below 1.0.

## Safety, now that the ultrasonic is gone

The old fast local collision reflex is replaced by:

* **speed easing** near obstacles, in the MPU evasion policy
  (`sectors["front"] < SAFETY_DISTANCE` scales speed down)
* **a command watchdog on the STM32** — if no `set_motion` arrives for 500 ms
  the motors halt. This is the one safety behaviour that does not depend on
  Linux being healthy, and it covers the crashed/wedged/disconnected-MPU case
  that matters most.

## Bring-up order

```bash
python3 -m scripts.servo_test --centre     # does it park straight ahead?
python3 -m scripts.servo_test --sweep      # smooth, no buzzing at the ends?
python3 -m scripts.servo_test --verify     # CHECK SERVO_DIR -- see below
python3 -m scripts.track_test --camera 0   # sweep + track, wheels disabled
python3 -m app.main --no-drive             # full loop, wheels still disabled
python3 -m app.main                        # live
```

**Do not skip `--verify`.** If `config.SERVO_DIR` is backwards every bearing
is mirrored and the robot will flee *toward* the pursuer. The script walks you
through a physical check and tells you which value to set.
