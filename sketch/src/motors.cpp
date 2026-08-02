/**
 * @file motors.cpp
 * @author Charlie Kushelevsky (ckushelevsky@ucsd.edu)
 * @brief implements MCU functions for motor control from MPU
 * @version 0.1
 * @date 2026-07-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <Arduino.h>
#include "motors.h"

// Pin map confirmed by polarity_test.ino against ELEGOO's reference code:
// only one direction pin per channel is exposed; channel A is physically
// RIGHT, channel B is physically LEFT. Both positive-is-forward.
static const int PWMA = 5, AIN1 = 7;   // right motor
static const int PWMB = 6, BIN1 = 8;   // left motor
static const int STBY = 3;

static const float RIGHT_MOTOR_TRIM = 1.0f;   // tune: >1 boosts weak right side


/**
 * @brief Drives one TB6612 channel at a signed PWM value.
 *
 * @param pwm_pin PWM pin for this channel.
 * @param dir_pin Direction pin for this channel.
 * @param value Signed PWM magnitude in [-255, 255]. Positive is forward and
 * negative is reverse, confirmed for both channels by polarity_test.ino.
 */
static void set_side(int pwm_pin, int dir_pin, int value) {
    bool forward = value >= 0;
    digitalWrite(dir_pin, forward ? HIGH : LOW);
    int mag = abs(value);
    if (mag > 255) mag = 255;
    analogWrite(pwm_pin, mag);
}

void motors_begin()
{
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(STBY, OUTPUT);
    // TB6612 is disabled while STBY is low -- without this the driver stays
    // in standby and the wheels never turn, whatever PWM we write.
    digitalWrite(STBY, HIGH);
}

/**
 * @brief Drives both wheels at independent signed PWM values.
 *
 * left_pwm and right_pwm are physical sides, not TB6612 channel letters:
 * left_pwm is routed to channel B and right_pwm to channel A, per the
 * mapping confirmed in polarity_test.ino.
 *
 * @param left_pwm Signed PWM for the physically left wheel pair, in
 * [-255, 255]. Routed to channel B.
 * @param right_pwm Signed PWM for the physically right wheel pair, in
 * [-255, 255]. Routed to channel A.
 */
void motors_drive(int left_pwm, int right_pwm)
{
    set_side(PWMA,AIN1, right_pwm);
    set_side(PWMB,BIN1, left_pwm);
}

void motors_stop()
{
    motors_drive(0,0);
}
