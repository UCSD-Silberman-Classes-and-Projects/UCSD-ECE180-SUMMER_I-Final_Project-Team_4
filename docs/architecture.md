# Architecture

## Processor split

| Concern | Processor | Notes |
|---|---|---|
| Camera capture | MPU (Debian) | USB webcam (V4L2) or ESP32-CAM MJPEG stream, see Camera section below |
| Person detection (TFLite) | MPU | `app/perception/detector.py` |
| Bearing / proximity | MPU | `app/perception/geometry.py` (pure) |
| LiDAR read / mask / merge / sectorize | MPU | both LD19s on MPU UART |
| Evasion policy | MPU | `app/control/evasion.py` (pure) |
| Motor PWM (TB6612) | STM32 (Zephyr) | executes commands from MPU |
| Ultrasonic safety reflex | STM32 | **local** hard-stop, no MPU round-trip |

The MPU decides; the STM32 acts and keeps one fast local reflex so a slow vision
frame can never cause a head-on collision.

## Camera

`app/perception/camera.py` opens `config.CAMERA_SOURCE` through
`cv2.VideoCapture`, which accepts either a V4L2 device index (int, USB
webcam) or a stream URL (str). `parse_source()` / `--camera` on the CLI
scripts pick between them without editing the config file.

The kit's stock ELEGOO ESP32-WROVER camera module is a stream-URL source: it
is its own microcontroller, not a USB device. It connects to the main shield
only through a 4-pin serial header (UART, for command relay), never for
video. ELEGOO's reference firmware boots it in `WIFI_AP` mode (default IP
`192.168.4.1`) and serves MJPEG over HTTP at `:81/stream` alongside a
separate TCP socket on port 100 for JSON motion commands over `Serial2` --
video and control are distinct channels, and video never touches the shield
or a USB line. The MPU must already be a WiFi client of that AP (or of
whatever network the ESP32 is reflashed to join in STA mode); `camera.py`
does not join the network for you. Not yet validated against the real ESP32
(see `DEVELOPMENT_LOG.md` Next Steps); `--camera` unset with no ESP32 present
falls back to `config.CAMERA_INDEX` (a local USB webcam).

## MPU -> STM32 messages (RPC over the Bridge)

MPU side (`app/control/bridge_client.py`) uses `arduino.app_utils.Bridge`;
STM32 side (`sketch/sketch.ino`) uses `Arduino_RouterBridge`'s `Bridge`
object. Both APIs were verified against their actual source before use.

- `set_motion(left_pwm, right_pwm)` — signed PWM (-255..255), sign = direction.
  Sent via `Bridge.notify` (fire-and-forget): it's sent every control loop
  iteration, so a dropped ack should never stall motion. Registered on the
  STM32 with `Bridge.provide_safe`, since the handler calls
  `digitalWrite`/`analogWrite`.
- `stop()` — halt both motors. Also `Bridge.notify` / `provide_safe`.
- `get_range()` — single atomic ultrasonic read (meters); the STM32 already
  enforces its own stop, this is just for telemetry/backup. Sent via
  `Bridge.call` since it needs a return value. Registered with
  `Bridge.provide_safe`, since the handler calls `pulseIn`.

## LiDAR geometry (fill in from measurement)

Two LD19s, front and rear, because the centrally-mounted UNO Q + wiring
obstructs any single unit's 360 sweep. Each unit's fixed phantom returns
(central clutter, own body, the other LiDAR) are masked per-unit before merge.

`app/sensing/ld19_driver.py` wraps `lds2d` (PyPI, Apache-2.0), a maintained
plain-pyserial parser that supports LD19 directly, rather than a hand-rolled
protocol decoder. Its LD19 support is ported from the kaiaai/LDS C++ library
and unit-tested against synthetic packets, but not yet hardware-confirmed by
lds2d's own maintainers -- validate its output with `scripts/lidar_viz.py`
before trusting it.

Measure and record:

| Unit | Offset (x, y, yaw) | Masked arcs (deg) |
|---|---|---|
| Front LD19 | (____, ____, ____) | (____, ____) |
| Rear LD19 | (____, ____, ____) | (____, ____) |

These live in `app/config.py` (`*_LIDAR_OFFSET`, `*_LIDAR_MASKS`). Verify with
`scripts/lidar_viz.py`: the masked arcs should show no phantom close returns, and
a known object (e.g. a box at a measured spot) should land correctly in the merged
robot frame.

## Sector reduction

Merged points reduce to six sector minimums (front, front_left, front_right,
left, right, rear). The evasion policy only ever sees these six numbers, never a
raw point cloud. Corner condition: `front_left` and `front_right` both closer
than `CORNER_DISTANCE`.

## Pin map (VERIFY chassis -> UNO Q GPIO)

The ELEGOO chassis wires TB6612 + ultrasonic to specific R3 pins. The UNO Q is
UNO-footprint compatible, but confirm each control pin lands on GPIO the STM32
actually drives before trusting `sketch/sketch.ino`.

This shield exposes only one direction pin per TB6612 channel to the MCU
(confirmed against ELEGOO's own `DeviceDriverSet_xxx0.h`/`.cpp` reference
code); the other direction input is hardwired on the shield PCB.

| Signal | Sketch pin | Verified? |
|---|---|---|
| PWMA / AIN1 (right motor) | 5 / 7 | [x] |
| PWMB / BIN1 (left motor) | 6 / 8 | [x] |
| STBY | 3 | [x] |
| Ultrasonic TRIG / ECHO | 12 / 13 | [ ] |

## Target re-identification (identity gate)

`app/perception/identity.py` implements the proposal's "target re-identification"
nice-to-have with no extra model: the person bbox's torso region (shirt) becomes
a smoothed HS histogram, matched against enrolled samples with temporal voting.
The gate turns on automatically once anyone is enrolled (`scripts/enroll.py`),
and the main loop treats an unrecognized person as "no pursuer" so the robot
ignores strangers.

**Multi-candidate selection.** With the gate on, the loop calls
`detector.detect()` (all people) rather than `detector.best()` (most prominent
one) and hands every detection to `PursuerSelector`, which returns the
best-matching enrolled person. This matters because the detector's notion of
"best" is score/prominence: a stranger standing closer, or simply present
first, would otherwise occupy the tracker and an enrolled pursuer entering the
frame would never be evaluated. Each candidate also keeps its OWN voting
window, associated frame-to-frame by bbox IoU (`ID_IOU_MATCH`), so a
stranger's low scores never dilute an enrolled person's average -- an enrolled
pursuer is recognised in exactly `ID_MIN_VOTES` frames regardless of how many
strangers share the frame. Candidates that go unmatched for
`ID_CANDIDATE_MAX_MISSED` frames are dropped, so the selector releases a
pursuer who leaves instead of clinging to a stale box. Cost is sub-millisecond per frame, so it does not threaten
the 10 FPS week-1 gate. Identity is written to the run log (`identity` column)
for the evaluation.

Enroll each pursuer in demo clothes, in the demo room:

    python -m scripts.enroll --name charlie
    python -m scripts.enroll --name jaafar

Tunables live in `app/config.py` under "Identity gate". Limitation: identifies
clothing, not faces -- two pursuers in near-identical shirts are
indistinguishable; extreme lighting changes lower scores. This is an acceptable
trade for a chase robot, since faces are unusable at chase distance / from
behind (exactly what the camera sees), and it keeps the whole pipeline at one
model to protect the FPS budget.


## Evaluation: time-to-capture

`RunLogger` writes one CSV row per loop iteration; `scripts/analyze_runs.py`
turns those into the project's headline metric:

    python -m scripts.analyze_runs data/runs/*.csv
    python -m scripts.analyze_runs data/runs/*.csv --plot survival.png

Survival time is the timestamp of the first `caught == 1` row. Runs where the
pursuer never crossed `CAUGHT_PROXIMITY` are reported as ESCAPED with survival
equal to the full run length, so they are not averaged in as instant captures.
The report also surfaces mean loop FPS (flagged when it falls under the 10 FPS
gate), how much of the run the pursuer was visible, and how often the robot was
boxed in -- the last two explain *why* a survival number came out the way it
did.

Compare configurations by mean survival across several runs, never a single
run: run-to-run spread is large (the synthetic sanity check showed a ~5 s
standard deviation across three runs), so one good run proves nothing.


## Vision bring-up

`scripts/vision_preview.py` is the visual counterpart to `benchmark_fps.py`:
where the benchmark answers "is it fast enough", the preview answers "is it
seeing the right thing, and are the numbers we hand the controller correct".
It renders detections, the identity gate's choice, and the live bearing /
proximity values, and it runs on saved frames as well as a live camera, so
detector regressions can be checked offline without the robot.

Two model-specific assumptions live in `detector.py` and are the usual cause of
a model that "loads but detects nothing": `_preprocess()` feeds a uint8
quantized input, and `_read_outputs()` reads output tensors in the order
`[boxes, classes, scores]`. Run `vision_preview.py --inspect` against a new
model file to see its actual input dtype and output layout before debugging
anything else.


## Camera pan servo and the removal of the ultrasonic sensor

The camera now rides on an HS-225MG pan servo, and the ultrasonic sensor has
been removed from the design. Consequences, in order of importance:

**Bearing is no longer frame-relative.** `geometry.camera_to_robot_bearing()`
composes the in-frame angle with the servo angle to produce a chassis-relative
bearing. Anything consuming bearing must use that, not `bbox_to_bearing()`.
`config.SERVO_DIR` absorbs a mirrored linkage; verify it with
`scripts/servo_test.py --verify` before driving.

**Range comes from the LiDAR, not box height.** `lidar.range_at_bearing()`
returns the nearest return within `BEARING_WINDOW_DEG` of the pursuer's
bearing, and `geometry.range_to_proximity()` maps that to [0, 1]. The old
`bbox_to_proximity()` remains for reference and is still unit-tested, but the
main loop no longer uses it: a real distance beats a height proxy, and the
proxy degrades badly once the camera can be pointed off-axis.

**The MCU gained the servo and lost the reflex.** `sketch/src/servo_cam.cpp`
slew-limits the pan so the MPU only posts targets. The ultrasonic hard stop is
replaced by a 500 ms command watchdog: no `set_motion`, motors halt. Obstacle
*speed easing* moved to the MPU evasion policy, which is acceptable because it
is a comfort behaviour; the watchdog covers the safety-critical case where the
MPU stops running entirely.

**RPC surface** (names must match `app/control/bridge_client.py`):

| Name | Args | Returns | Transport |
|---|---|---|---|
| `set_motion` | left_pwm, right_pwm | - | notify (fire-and-forget) |
| `stop` | - | - | notify |
| `set_servo` | angle_deg | - | notify |
| `get_servo` | - | int deg | call (bring-up scripts only) |

`get_servo` is deliberately unused by the control loop: a blocking round-trip
at 15 FPS is exactly the stall the split-processor design exists to avoid.
