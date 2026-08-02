<div align="center">
  
# Mousebot 
# Autonomous Person Tracking and Obstacle Avoidance Evasion Robot

<img width="1000" height="200" alt="image" src="https://github.com/user-attachments/assets/b066e3b0-ca6e-486a-8b3d-a0fa0c022a4e" />

### Team 4
### ECE 180 Final Project Summer Session 1 2026

</div>

---

<p>
 <img width="2048" height="1537" alt="finalmousebot" src="https://github.com/user-attachments/assets/dce4e1ac-c8eb-4efa-b6f7-85e6b185fb4d" /> 
</p>

---

## Table of Contents
1. [Abstract](#abstract)
2. [Team Members](#team-members)
3. [What We Promised](#what-we-promised)
4. [Accomplishments](#accomplishments)
5. [Project Demonstration](#project-demonstration)
6. [Challenges](#challenges)
7. [Future Development](#future-development)
8. [Project Hardware](#hardware-and-cad)
9. [Project Software](#computer-vision-and-software)
10. [Final project Presentation](final-project-presentation)
11. [References](#references)
12. [Acknowledgments](#acknowledgments)
13. [Contacts](#contacts)
---

## Abstract
An autonomous robot car that uses on-device vision to detect and track a pursuing person as it drives away to avoid being caught while attempting to navigate around obstacles in its path. Using dual 2D LiDAR in front and back for corner detection and obstacle avoidance with a rotating webcam for person tracking, the mousebot maneuvers to keep the person in the camera frame while threading between other people, chairs, and under tables. All perception and decision-making run on-device on an Arduino UNO Q.

---

## Team Members
- Ryan Chen - Electrical and Computer Engineering
- Charlie Kushelevsky - Mathmatics-Computer Science
- Jaafar Sameer - Electrical and Computer Engineering

---

## What We Promised
### Must Have
- Person recognition and tracking of specific people to escape from
- Estimation of proximity and bearing of person (direction of approach)
- Reactive escape steering away from person 
- Obstacle avoidance of objects within its path

### Nice to Have
- Continuous Camera tracking via servo
- Multiple pursuers/ target identification to escape from 
- Corner detection and evasion to avoid entrapment
- Training of evasion tactics (pursuer prediction, threading through spaces where pursuer cannot pass through)

---

## Accomplishments
- Successfully built base chassis with custom 3D-printed top plate to mount all sensors and equipment
- Smooth person tracking with camera-mounted servo
- Identification of specific people via personal profiles
- Object avoidance with lidar mapping
- Integration of camera with lidars to command chassis movement
- Approximation of person distance and position using lidar to assist camera in tracking pursuer

---

## Project Demonstration
The lidar was later found to be set to only 60 deg on each side. Setting it to 90 deg on each side will help eliminate blindspots while approaching obstacles in the future

https://github.com/user-attachments/assets/69da7641-b7ed-4182-bcb2-6be922de7190

https://github.com/user-attachments/assets/35d9001b-ba4c-442b-992d-67fd3eab1270

---

## Challenges
- Insufficient ports for camera and 2 lidars; had to add a USB hub
- Arduino Uno Q USB-C port only takes input power and data; could not output power through the board
- Connectivity issues through USB hub to Arduino Uno Q's C port, required "jumping" the system by connecting the hub's USB-C to a phone to draw power and data, then replugging into Arduino Uno Q
- Insufficient power for all sensors, required portable battery bank to power externals
- Camera only mounts via a single M6 screw; required multiple redesigns to mount onto servo
- Had to reverse direction of car so camera can track pursuer chasing behind 
- Space limitation of deck, overhang lidar over edge of plate, stacked battery box and Arduino vertically, raised servo mount for clearance of rotating camera
- Power difference between motors - had to adjust throttle settings for each side to synchronize power and movement
- Lidars required masking the rear 180 deg to avoid detecting wires and equipment on the top plate, and wires had to be routed carefully from the USB hub in front of the lidar
- Used Tailscale to facilitate a more reliable connection to Arduino Uno Q on campus

---

## Future Development
- Additional ESP32 camera/C920 webcam for 360 deg vision coverage
- Multi-person designation and evasion
- Reinforcement Learning model to experiment with evasion tactics
- SLAM and object recognition for more successful evasion
- Larger chassis deck, larger/more powerful battery, stronger motors with encoders
- More rigid mounting solution for camera on servo
- IMU feedback for movement

<img width="353" height="313" alt="Future" src="https://github.com/user-attachments/assets/b5826f9d-8019-4ed9-a230-a60a743e3cea" />

---

## Hardware and CAD
<img width="800" height="570" alt="mousebotsidelabel" src="https://github.com/user-attachments/assets/891a20cd-d9c0-4343-b313-e4330778546e" />

### CAD:

<img width="616" height="360" alt="CAD" src="https://github.com/user-attachments/assets/d63ca254-d545-4f17-8f04-c8d06e7e36b1" />

The top deck plate is a heavily modified version of the original provided by the kit. The servo mounting hole has been moved back as much as possible to allow for the Lidar, which still hangs over the edge. 3 pairs of holes are made for the now central battery box position under the raised plate for the Arduino Uno Q. 4 18 mm diameter holes were also created to route cables from the bottom plate to the top.


Plans have been made to use 3D printed female to female standoffs to raise the plate, using alternating sets of holes in the stackplate to infinitely stack upwards as needed. However, standard female to male M3 standoffs were found that could already be stacked without altering hole layouts. This allowed for the possibility of a second servo mounted or fixed camera on top of the Arduino plate. 


**In future developments with access to printers possessing >250mm of printing length or width, the entire deckplate can be extended ~30-40mm to give more room for the camera to turn and bring the USB hub between the Lidar and Arduino plate.** Servos can be installed at the deck level to reduce height profile, and a second camera at deck level could be possible with an additional ~50 mm. Replacing the standoffs between the top and bottom deck for taller ones can allow the top deck to overhang the wheels, giving more width to secure wiring or larger batteries. **Using an 1/18 or an 1/10 scale RC car similar to ECE148 could give the necessary space and power to realize these goals.**


### Hardware List
- **Arduino UNO Q** — dual processor:
  - **Qualcomm MPU (Debian Linux)** — camera capture, person detection, LiDAR processing, evasion policy
  - **STM32U585 (Zephyr RTOS)** — motor PWM, executes motion commands
- **ELEGOO Smart Robot Car V4.0** chassis (TB6612 motor driver, TT DC motors)
- **Camera** — either a USB webcam (a C920 Logitech webcam was used) plugged directly into the MPU (V4L2), or
  the kit's stock **ESP32-WROVER camera module**: its own microcontroller,
  hosting a WiFi AP and streaming MJPEG over HTTP from its own web server. It
  is *not* a USB device — it links to the main shield only via a 4-pin UART
  header for command relay, never for video (see `docs/architecture.md`).
  Selected via `config.CAMERA_SOURCE` / `--camera`.
- **2× LDRobot LD19** 2D LiDAR — one front, one rear (see LiDAR note below)


### Why two LiDARs

The UNO Q sits in the **center** of the top plate, surrounded by wiring and mounts. That central
clutter obstructs a single LiDAR's 360° sweep no matter where it's placed, and a riser can't clear
the wire height. So we mount **one LD19 at the front and one at the rear**: each covers the arc the
central obstruction blocks for the other, and the two scans are merged into one 360° picture.

Each unit's body, the central clutter, and the *other* LiDAR appear as fixed phantom returns and are
**masked out per-unit** before the scans are merged (see `app/sensing/lidar.py`).

---

## Computer Vision and Software

## Computer Vision Demo

https://github.com/user-attachments/assets/76992d57-054b-4c80-b433-088bdea6a52e

## Camera Feed and Lidar Map
The camera scans until it sees a person. If the person is recognized based on the onboard profile of the pursuer, it will start tracking it. If it is over 95% certain it is the pursuer, it will lock onto that person and ignore any other person for a set period of time 

<img width="639" height="479" alt="camdetection" src="https://github.com/user-attachments/assets/f6f76212-4c50-42d6-837b-07fd4e3da633" />

The Lidar map is divided into two hemispheres: the lidar in the back (next to the camera) plots detections in the lower hemisphere (bottom half) of the graph, while the front plots in the upper hemisphere. The red X is the estimated psoition of the pursuer.
<img width="674" height="756" alt="lidarscreen" src="https://github.com/user-attachments/assets/c82f1d95-0065-4e2a-bc2e-756165e31ab4" />


## Architecture

Two processors, two very different jobs:

```
                 ┌─────────────────────── UNO Q MPU (Debian, Python) ───────────────────────┐
   Camera     ──▶│ camera ─▶ detector ─▶ geometry (bearing, proximity) ─┐                    │
   LD19 front ──▶│ lidar (mask ─▶ merge ─▶ sectorize) ──────────────────┼─▶ evasion policy ─┼─┐
   LD19 rear  ──▶│                                                       ┘                    │ │
                 └────────────────────────────────────────────────────────────────────────┘ │
                                                                                              │ motion cmd (RPC)
                 ┌─────────────────────── STM32 (Zephyr) ──────────────────────────────────┐ │
                 │ set_motion(left_pwm, right_pwm) ◀───────────────────────────────────────┼─┘
                 │         │
                 └──────────────────────────────────────────────────────────────────────────┘
```

A ultrasonic sensor that came with the kit and was part of the ESP32 camera mount was considered for emergency stops when an object was detected too close, as it ran off of the STM32 and potentially could react faster. However, when we switched to using solely an C920 webcam with lidar, the ultrasonic sensor became an unneeded redundancy.

---

## The main loop (MPU)

```
capture frame
  ─▶ detect person            → bbox
  ─▶ geometry                 → bearing, proximity
read + merge + sectorize LiDAR → sector distances (front, FL, FR, left, right, rear)
  ─▶ evasion policy           → heading, speed
  ─▶ convert to L/R PWM
  ─▶ bridge.set_motion(...)   → STM32
  ─▶ log everything (for time-to-capture eval)
repeat
```

---

## Repo layout

```
mouse-bot/
├── README.md
├── pytest.ini                   # limits collection to tests/ (scripts/track_test.py would
│                                 #   otherwise match the *_test.py pattern)
├── requirements.txt             # numpy, opencv-python, pyserial, pytest
├── sketch.yaml                  # arduino-app-cli build manifest
│
├── sketch/                      # STM32 side (Zephyr)
│   ├── sketch.ino               # motors + pan servo, exposed to the MPU over Bridge
│   ├── motor_test/              # hardware bring-up sketch, not a pytest test
│   │   └── motor_test.ino       # standalone check of motor wiring/direction
│   ├── polarity_test/           # hardware bring-up sketch, not a pytest test
│   │   └── polarity_test.ino    # standalone check of motor polarity
│   └── src/                     # implementation used by sketch.ino
│       ├── motors.cpp/.h        # motor control functions, called from the MPU
│       └── servo_cam.cpp/.h     # pan servo (HS-225MG) control
│
├── app/                         # MPU (Python)
│   ├── __init__.py              # package marker (empty)
│   ├── main.py                  # top-level evasion loop (capture → detect → track → evade)
│   ├── config.py                # all tunables in one place
│   ├── simulate.py              # hardware-free 2D flee simulation (real pipeline, fake room)
│   ├── telemetry.py             # JSON snapshot of control-loop state for laptop-side viewers
│   ├── control/
│   │   ├── __init__.py          # package marker (empty)
│   │   ├── bridge_client.py     # RPC to STM32 (motion, servo)
│   │   ├── bt_console.py        # Bluetooth command console (RFCOMM)
│   │   ├── evasion.py           # escape policy (pure)
│   │   └── scanner.py           # camera pan: sweep to search, then track
│   ├── perception/
│   │   ├── __init__.py          # package marker (empty)
│   │   ├── camera.py            # USB webcam (V4L2) / ESP32-CAM MJPEG stream capture
│   │   ├── detector.py          # TFLite person detection
│   │   ├── geometry.py          # bbox → bearing + proximity (pure)
│   │   └── identity.py          # person re-ID: torso HSV signatures, enrollment DB, voting matcher
│   ├── sensing/
│   │   ├── __init__.py          # package marker (empty)
│   │   ├── ld19_driver.py       # self-contained LD19 serial protocol driver
│   │   ├── ld19_driver.py.bak   # old lds2d-wrapper driver, superseded by ld19_driver.py above
│   │   └── lidar.py             # dual LD19: mask → transform → merge → sectorize
│   └── utils/
│       ├── __init__.py          # package marker (empty)
│       └── logging.py           # structured per-frame CSV run logs
│
├── models/                      # TFLite model(s) live here (gitignored)
│   ├── README.md                # tensor layout, model notes
│   ├── coco_labels.txt          # COCO class names (debug only)
│   ├── person_detect.tflite     # default: SSD MobileNet V2, COCO, uint8, 300x300
│   └── person_detect_mobiledet.tflite  # alternative: SSDLite MobileDet, 320x320
│
├── scripts/                     # pytest doesn't collect these; run directly with python -m
│   ├── analyze_runs.py          # time-to-capture + run stats from logs
│   ├── benchmark_fps.py         # week-1 gate: detection FPS on the MPU
│   ├── bt_client.py             # Linux Bluetooth (RFCOMM) client for bt_console.py
│   ├── collect_frames.py        # save frames for debug/eval
│   ├── enroll.py                # enroll a pursuer's appearance signature
│   ├── fetch_models.sh          # download the TFLite detection model(s)
│   ├── flee_sim.py              # run + plot the flee simulation (class-demo visual)
│   ├── lidar_test.py            # LD19 bring-up: list, identify, watch, verify
│   ├── lidar_viz.py             # verify masks, merged scan, and sector reduction
│   ├── radar_view.py            # laptop-side LiDAR + pursuer + escape-path dashboard
│   ├── servo_test.py            # bring up + verify pan servo direction
│   ├── sign_check.py            # on-robot camera-lock/donut diagnostic
│   ├── stub_smoketest.py        # offline smoke test of the control path, no hardware
│   ├── test_camera.py           # verify a camera source (index or stream URL)
│   ├── track_test.py            # sweep + track rehearsal, wheels disabled
│   ├── vision_preview.py        # boxes, bearing, proximity, identity ([LOCKED] tag)
│   └── watch.py                 # live MJPEG stream to a browser over WiFi
│
├── tests/                       # pytest unit tests, pure logic only
│   ├── README.md                # what each test file covers
│   ├── conftest.py              # shared test fixtures
│   ├── test_evasion.py          # flee direction, speed scaling, corner override, PWM mapping
│   ├── test_flee_integration.py # closed-loop flee test in a simulated 6x6 m room
│   ├── test_geometry.py         # bbox → bearing/proximity
│   ├── test_identity.py         # torso signatures, enrollment storage, pursuer selection
│   ├── test_ld19.py             # LD19 protocol driver (synthetic bytes)
│   ├── test_lidar.py            # arc math, masking, frame transform, sectorizing, corners
│   ├── test_logging_and_analysis.py  # run logging + time-to-capture analysis
│   ├── test_scanner.py          # sweep, lock on, hold through dropouts
│   └── test_servo_geometry.py   # pixels → camera angle → chassis angle → steer
│
├── docs/
│   ├── architecture.md          # processor split, message schema, pin map, LiDAR offsets/masks
│   ├── identity_tuning.md       # tuning the identity gate (misidentifying strangers)
│   └── servo_tracking.md        # pan servo search/track and how bearing is computed
│
├── assets/                      # README images
│   ├── Mousebotfront.png
│   ├── V3.png
│   ├── camdetection.png
│   ├── finalmousebot.png
│   ├── fronton.png
│   ├── lidarscreen.png
│   ├── mousebotV2.png
│   └── protomousebot.png
│
└── proposal/                    # original project proposal
    ├── slides.pdf
    ├── slides.tex
    └── UCSDLogo_JSOE_BlueGold_0_0.png
```

---

## Build order (de-risks the unknowns first)

1. **Bridge with a stub** — run the MPU loop end-to-end printing fake motion commands, no hardware.
2. **Detector on recorded video** — benchmark FPS off-robot. **Week-1 gate: must clear ~10 FPS.**
   Re-run with the LiDAR driver(s) active, since both share the MPU.
3. **STM32 sketch alone** — motors + ultrasonic reflex, driven by manual RPC calls.
4. **LiDAR bring-up** — one LD19, then two: verify masks and scan merge with `lidar_viz.py`.
5. **Join everything** — real bridge, camera, motors, LiDAR.

---

## Setup notes

- Install OpenCV via **apt** (`sudo apt install python3-opencv`), *not* pip, on the board's ARM/Debian.
- `Arduino_RouterBridge` (STM32 side) and `arduino.app_utils.Bridge` (MPU side) are both used as
  documented in `docs/architecture.md`, verified against their actual source.
- LiDAR parsing uses `lds2d` (`pip install lds2d`) rather than a hand-rolled protocol decoder; see
  `app/sensing/ld19_driver.py` for the caveat on its LD19 support being unverified on real hardware
  by lds2d's own maintainers.
- Keep any STM32 sensor read a single atomic RPC — no multi-round-trip reads.
- Measure each LD19's mounting offset (x, y, yaw) from the robot center — the scan merge depends on it.

---

## Must haves vs nice to haves

**Must haves:** on-device person detection · bearing + proximity · reactive escape steering ·
LiDAR-based corner detection + avoidance · time-to-capture metric.

**Nice to haves:** learned evasion policy (RL in sim → transfer) · multi-pursuer / target
re-ID · live FPV / detection overlay stream.


## Getting the detection model

The model binaries are gitignored; fetch them once per machine:

```
bash scripts/fetch_models.sh
```

This pulls a uint8-quantized SSD MobileNet V2 (COCO, 300x300) to
`models/person_detect.tflite`. It has been verified against `detector.py`
unmodified: uint8 input, output order `[boxes, classes, scores]`, person =
class 0. See `models/README.md` for the full tensor layout and an alternative
model.

## Watching the camera live in a browser

No display needed on the board -- stream to any browser on the same network:

```
python3 -m scripts.watch --camera 0                 # raw feed
python3 -m scripts.watch --camera 0 --annotate       # boxes + identity gate live
```

Then open `http://<board-hostname>.local:8080/` from your laptop. Frame rate
and JPEG quality are capped (`--fps`, `--quality`) so the stream does not
compete with the detector for CPU/bandwidth. Ctrl-C on the board to stop.

## Camera pan servo (search + track)

The C920 is mounted on an HS-225MG pan servo: it sweeps until it finds an
enrolled pursuer, then tracks them while the chassis drives away. Bearing
comes from the camera, range from the LiDAR. Full explanation, including the
230 deg coverage limit and the blind arc behind, in `docs/servo_tracking.md`.

```
python3 -m scripts.servo_test --verify     # CHECK SERVO_DIR FIRST
python3 -m scripts.track_test --camera 0   # sweep + track, wheels disabled
python3 -m app.main --no-drive             # full loop, wheels disabled
```

## Testing the vision stack

```
python -m scripts.vision_preview --inspect              # check a new model file
python -m scripts.vision_preview                        # live camera
python -m scripts.vision_preview --source data/frames   # offline, saved frames
python -m scripts.vision_preview --source shot.jpg --save out.jpg --no-window
```

Draws every person detection, marks which one the identity gate selected
(green = selected, red = rejected), and overlays the exact bearing and
proximity the evasion policy will receive. Use it to sanity-check the bearing
sign and to tune `PROXIMITY_MIN/MAX_BOX_FRAC` -- stand at the distance that
should read "caught" and adjust until proximity crosses `CAUGHT_PROXIMITY`.

`--inspect` prints the model's input shape/dtype and output tensor order; if
detections look wrong, that is the first thing to check, since
`detector._read_outputs()` assumes the order `[boxes, classes, scores]` and it
varies between models.

## Evaluating a run

```
python -m scripts.analyze_runs data/runs/*.csv            # time-to-capture
python -m scripts.analyze_runs data/runs/*.csv --plot survival.png
```

Reports survival time per run (runs where the pursuer never got close are
marked ESCAPED and credited the full duration), plus mean loop FPS, how much of
the run the pursuer was visible, and how often the robot was cornered. Compare
configurations on **mean survival across several runs** -- single runs are noisy.

## Tests

```
pip install pytest
python -m pytest tests/ -q
```

54 unit tests over the pure logic (geometry, evasion policy, LiDAR sectorizing,
identity gate, run analysis). No hardware, camera, or model file required, so
they run on any laptop. See `tests/README.md`.

---

## [Final Project Presentation](https://docs.google.com/presentation/d/1U6K6tgcmcW5ZD5GqswljRYP57PqoK4Ar_VmlcLOLLok/edit?usp=sharing)

---

## References
- [Lidar LD19 datasheet](https://www.ldrobot.com/images/2023/05/23/LDROBOT_LD19_Datasheet_EN_v2.6_Q1JXIRVq.pdf)
- [Lidar CAD](https://grabcad.com/library/lidar-ld19-1)
---

## Acknowledgments  
Special thank you to Professor Silberman and TA Jose Castillo for advice, insights, and part sourcing!

---

## Contacts
- Ryan Chen - ryc004@ucsd.edu
- Charlie Kushelevsky - ckushelevsky@ucsd.edu
- Jaafar Sameer - jsameer@ucsd.edu


