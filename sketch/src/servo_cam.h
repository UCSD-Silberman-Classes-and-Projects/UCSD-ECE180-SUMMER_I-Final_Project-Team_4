/**
 * @file servo_cam.h
 * @brief Camera pan servo (HS-225MG) control on the STM32 side.
 * @date 2026-07-24
 *
 * The camera rides on a single pan servo so it can sweep for the pursuer and
 * then stay locked on them while the chassis drives away. Angle is commanded
 * from the MPU over the Bridge; the actual motion is slew-limited HERE rather
 * than on the Linux side, because a servo snapping to a new angle blurs the
 * very frame the MPU needs to see. Smoothing on the MCU also means a stalled
 * or slow vision loop cannot produce a jerky pan.
 *
 * Convention: degrees 0..180 mechanical, SERVO_CENTER_DEG looks straight
 * forward along the chassis. Larger angle turns the camera toward the robot's
 * LEFT by default; flip config.SERVO_DIR on the MPU side if your linkage is
 * mirrored (no firmware change needed).
 */

#ifndef SERVO_CAM_H
#define SERVO_CAM_H

/** @brief Attach the servo and move it to the centre position. */
void servo_begin();

/**
 * @brief Set the target pan angle. Motion is slew-limited toward this.
 * @param angle_deg Target angle in degrees, clamped to [SERVO_MIN_DEG,
 * SERVO_MAX_DEG].
 */
void servo_set_target(int angle_deg);

/**
 * @brief Step the servo toward its target. Call every loop() iteration.
 *
 * Moves at most SERVO_SLEW_DEG_PER_STEP per call, so the pan speed is set by
 * how often loop() runs. Cheap and non-blocking: no delay() inside.
 */
void servo_update();

/** @brief Current commanded angle in degrees (what the servo is being held at). */
int servo_current_deg();

#endif
