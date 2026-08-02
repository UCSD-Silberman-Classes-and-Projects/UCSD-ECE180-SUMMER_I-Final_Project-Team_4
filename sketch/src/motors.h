/**
 * @file motors.h
 * @author Charlie Kushelevsky (ckushelevsky@ucsd.edu)
 * @brief defines MCU functions for motor control from MPU
 * @version 0.1
 * @date 2026-07-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef MOTORS_H
#define MOTORS_H

void motors_begin();  // pinMode + STBY high
void motors_drive(int left_pwm, int right_pwm);
void motors_stop();




#endif