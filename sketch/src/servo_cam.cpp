/**
 * @file servo_cam.cpp
 * @brief Implements camera pan servo control (HS-225MG).
 * @date 2026-07-24
 */

#include <Arduino.h>
#include <Servo.h>

#include "servo_cam.h"

// HS-225MG is a standard analog servo: 50 Hz frame, ~600-2400 us pulse for
// the full mechanical range. If your horn does not reach the full sweep, or
// buzzes at an end stop, narrow these two numbers rather than the angle
// limits -- buzzing means the servo is being commanded past its hard stop.
static const int PULSE_MIN_US = 600;
static const int PULSE_MAX_US = 2400;

static const int SERVO_PIN = 10;      // PWM-capable pin, not shared with TB6612

static const int SERVO_MIN_DEG = 10;  // keep off the mechanical hard stops
static const int SERVO_MAX_DEG = 170;
static const int SERVO_CENTER_DEG = 90;

// Degrees per servo_update() call. loop() runs roughly every 20 ms, so 2 deg
// gives ~100 deg/s -- slow enough that frames stay sharp while sweeping.
static const int SERVO_SLEW_DEG_PER_STEP = 2;

static Servo pan;
static int target_deg = SERVO_CENTER_DEG;
static int current_deg = SERVO_CENTER_DEG;

void servo_begin()
{
    pan.attach(SERVO_PIN, PULSE_MIN_US, PULSE_MAX_US);
    target_deg = SERVO_CENTER_DEG;
    current_deg = SERVO_CENTER_DEG;
    pan.write(current_deg);
}

void servo_set_target(int angle_deg)
{
    if (angle_deg < SERVO_MIN_DEG)
    {
        angle_deg = SERVO_MIN_DEG;
    }
    if (angle_deg > SERVO_MAX_DEG)
    {
        angle_deg = SERVO_MAX_DEG;
    }
    target_deg = angle_deg;
}

void servo_update()
{
    if (current_deg == target_deg)
    {
        return;
    }
    int delta = target_deg - current_deg;
    if (delta > SERVO_SLEW_DEG_PER_STEP)
    {
        delta = SERVO_SLEW_DEG_PER_STEP;
    }
    if (delta < -SERVO_SLEW_DEG_PER_STEP)
    {
        delta = -SERVO_SLEW_DEG_PER_STEP;
    }
    current_deg += delta;
    pan.write(current_deg);
}

int servo_current_deg()
{
    return current_deg;
}
