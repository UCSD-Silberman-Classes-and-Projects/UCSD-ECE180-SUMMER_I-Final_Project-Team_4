<div align="center">
  
# Mousebot: Autonomous Person Tracking and Obstacle Avoidance Evasion Robot

<img width="800" height="200" alt="image" src="https://github.com/user-attachments/assets/b066e3b0-ca6e-486a-8b3d-a0fa0c022a4e" />

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
10. [References](#references)
11. [Acknowledgments](#acknowledgments)
12. [Contacts](#contacts)
---

## Abstract
An autonomous robot car that uses on-device vision to detect and track a pursuing person as it drives away to avoid being caught while attemping to navigate around obstacles in its path. Using dual 2D LiDAR in front and back for corner detection and obstacle avoidance with a rotating webcam for person tracking, the mousebot manuvers to keep the person in the camera frame while threading between other people, chairs, and under tables. All perception and decision-making run on-device on an Arduino UNO Q.

---

## Team Members
- Ryan Chen - ELectrical and Computer Engineering
- Charlie Kushelevsky - Mathmatics-Computer Science
- Jaafar Sameer - ELectrical and Computer Engineering

---

## What We Promised
### Must Have
- Person recognition and tracking of specific people to escape from
- Estimation of proximity and bearing of person (direction of approach)
- Reactive escape steering away from person 
- obstacle avoidance of objects within its path

### Nice to Have
- Continuous Camera tracking via servo
- Multiple pursuers/ target identification to escape from 
- Corner detection and evasion to avoid entrapment
- Training of evasion tactics (pursuer prediction, threading through spaces where pursuer cannot pass through)

---

## Accomplishments
- Sucessfully built base chassis with custom 3D printed top plate to mount all sensors and equipment
- Smooth person tracking with camera mounted servo
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
- Insufficient ports for camera and 2 lidars, had to add USB hub
- Arduino Uno Q USB C port only takes input power and data, could not output power through board
- Connectivity issues through USB hub to Arduino Uno Q's C port, required "jumping" the system by connecting the hub's USB C to a phone to draw power and data, then replugging into Arduino Uno Q
- Insufficient power for all sensors, required portable battery bank to power externals
- Camera only mounts via single M6 screw, require multiple redesigns to mount onto servo
- Had to reverse direction of car so camera can track pursuer chasing behind 
- Space limitation of deck, overhang lidar over edge of plate, stacked battery box and Arduino vertically, raised servo mount for clerance of rotating camera
- Power difference between motors - had to adjust throttle settigns for each side to synchronize power and movement
- Lidars required masking the rear 180 deg to avoid detecting wires and equipment on the top plate, and wires had to be routed carefully from the usb hub in front of the lidar
- Used Tailscale to facillitate more reliable connection to Arduino Uno Q on campus

---

## Future Development
- Additional ESP32 camera/C920 webcam for 360 deg vision coverage
- Multi person designation and evasion
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


### Hardware List
- **Arduino UNO Q** — dual processor:
  - **Qualcomm MPU (Debian Linux)** — camera capture, person detection, LiDAR processing, evasion policy
  - **STM32U585 (Zephyr RTOS)** — motor PWM, ultrasonic safety reflex, executes motion commands
- **ELEGOO Smart Robot Car V4.0** chassis (TB6612 motor driver, TT DC motors)
- **Camera** — either a USB webcam (a C920 Logitech webcam was used) plugged directly into the MPU (V4L2), or
  the kit's stock **ESP32-WROVER camera module**: its own microcontroller,
  hosting a WiFi AP and streaming MJPEG over HTTP from its own web server. It
  is *not* a USB device — it links to the main shield only via a 4-pin UART
  header for command relay, never for video (see `docs/architecture.md`).
  Selected via `config.CAMERA_SOURCE` / `--camera`.
- **2× LDRobot LD19** 2D LiDAR — one front, one rear (see LiDAR note below)
- **Ultrasonic sensor** (from kit) — retained purely as an STM32-side emergency-stop backstop

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
                 │ ultrasonic safety reflex: if range < STOP → halt motors locally         │
                 └──────────────────────────────────────────────────────────────────────────┘
```

Key principle: the STM32 holds a **local safety reflex** (halt if the ultrasonic reads too close)
that does not wait on the MPU. A slow vision frame can never cause a head-on collision.
The LiDAR does all the *smart* spatial reasoning on the MPU; the ultrasonic is a dumb, fast backstop.

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
├── sketch.yaml                 # arduino-app-cli build manifest
├── .gitignore
│
├── sketch/
│   └── sketch.ino              # STM32: motors, ultrasonic reflex, Bridge RPC
│
├── app/                        # MPU (Python)
│   ├── main.py                 # top-level loop
│   ├── config.py               # all tunables in one place
│   ├── perception/
│   │   ├── camera.py           # USB webcam / ESP32-CAM stream capture
│   │   ├── detector.py         # TFLite person detection
│   │   ├── geometry.py         # bbox → bearing + proximity (pure)
│   │   └── identity.py         # person re-ID: torso signatures, enrollment DB, voting matcher
│   ├── sensing/
│   │   ├── lidar.py            # LD19 read, mask, merge, sectorize
│   │   └── ld19_driver.py      # thin wrapper over lds2d's LD19 driver
│   ├── control/
│   │   ├── evasion.py          # escape policy (pure)
│   │   ├── bridge_client.py    # RPC to STM32 (motion, ultrasonic backstop)
│   │   └── bt_console.py       # Bluetooth remote-control server, see below
│   └── utils/
│       └── logging.py          # structured run logs
│
├── models/                     # TFLite model(s) live here (gitignored if large)
├── data/
│   └── identities/              # enrolled person re-ID signatures (one .npz per person)
├── scripts/
│   ├── benchmark_fps.py        # week-1 gate: detection FPS on the MPU
│   ├── lidar_viz.py            # visualize/verify merged scan + masks
│   ├── collect_frames.py       # save frames for debug/eval
│   ├── bt_client.py            # Linux Bluetooth client for bt_console.py
│   ├── test_camera.py          # verify a camera source (index or stream URL)
│   ├── vision_preview.py       # annotate frames/live: boxes, identity, [LOCKED]
│   ├── watch.py                # live MJPEG stream to a browser over WiFi
│   ├── enroll.py               # enroll a pursuer's appearance signature
│   ├── analyze_runs.py         # time-to-capture + run stats from logs
│   ├── stub_smoketest.py       # control path with no hardware
│   └── fetch_models.sh         # download the TFLite detection model
└── docs/
    └── architecture.md         # message schema, pin map, LiDAR offsets/masks
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

## Bluetooth command console

`app/control/bt_console.py` runs a small RFCOMM (Bluetooth serial) server on
the MPU as a wireless remote for the robot, separate from the `app.main`
evasion loop itself:

```
python -m app.control.bt_console --front-port /dev/ttyUSB0 --rear-port /dev/ttyUSB1
```

Pair the board once via `bluetoothctl` (`power on`, `discoverable on`,
`pairable on`, `agent on`, then pair from your phone/laptop). From a phone,
any Bluetooth serial-terminal app connecting on RFCOMM channel 1 works. From
a Linux PC, use `scripts/bt_client.py` rather than the `rfcomm` CLI tool —
`rfcomm`/`hcitool`/`sdptool` are legacy `bluez-utils` tools deprecated
upstream and missing on plenty of distros:

```
python -m scripts.bt_client 14:B5:CD:EA:BB:09
```

Either way, send one command per line:

- `start` — launch `app.main.run` in a background thread
- `stop` — signal it to stop and join (the loop's own `finally` block halts
  the motors via `bridge.stop()`)
- `usb` — `lsusb` output plus any `/dev/ttyUSB*`/`/dev/ttyACM*` ports found,
  handy for confirming which port is the front vs. rear LD19

It uses the standard library's `socket.AF_BLUETOOTH`/`BTPROTO_RFCOMM`
directly (Linux-only, needs BlueZ installed — `sudo apt install bluetooth
bluez`), so no extra Python package is required. `--stub` makes `start` run
the evasion loop hardware-free, same as `app.main --stub`.

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

## References
- [Lidar LD19 datasheet](https://www.ldrobot.com/images/2023/05/23/LDROBOT_LD19_Datasheet_EN_v2.6_Q1JXIRVq.pdf)
- [Lidar CAD](https://grabcad.com/library/lidar-ld19-1)
---

## Acknowledgments  
Special thank you to Professor Silberman and TA Jose Castillo for advice, insights, and part sourcing!

---

## Contacts
- Ryan Chen -
- Charlie Kushelevsky - 
- Jaafar Sameer - 


