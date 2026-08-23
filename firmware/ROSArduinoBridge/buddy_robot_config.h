/*********************************************************************
 * Buddy robot — chassis wiring options (esp2ros2).
 *
 * ROS always sends:  o <pwm_left_joint> <pwm_right_joint>
 *                    e  <left_joint_ticks> <right_joint_ticks>
 *
 * Buddy default (split cross):
 *   BUDDY_L298_ENCODER_CROSS 0 — encoders direct → left joint = physical left
 *                                wheel (+Y TF), right joint = physical right.
 *   BUDDY_L298_MOTOR_CROSS 1   — L298 motor outputs crossed on chassis:
 *                                left joint PWM  -> OUT3/4 (GPIO 32/33)
 *                                right joint PWM -> OUT1/2 (GPIO 18/19)
 *
 * Set both to 0 only if OUT1/2 and OUT3/4 motor wires match encoder sides
 * without crossing. Do not cross left_wheel_names in buddy controller.yaml.
 *
 * OUT1/2 and OUT3/4 use the same drive_hbridge() in motor_driver.ino.
 * See esp2ros2/docs/WIRING.md.
 *********************************************************************/
#ifndef BUDDY_ROBOT_CONFIG_H
#define BUDDY_ROBOT_CONFIG_H

#define BUDDY_L298_ENCODER_CROSS 0
#define BUDDY_L298_MOTOR_CROSS 1

/* Negate raw LEFT encoder counts in readEncoder(LEFT).
 * Must be 1 on Buddy: bench m 5 5 shows left ticks go negative while the wheel
 * drives forward. With INVERT 0 the velocity PID sees target=+5, input=-N →
 * perror explodes, ITerm winds up, and the left wheel spins away on turns.
 * After this is 1, set negate_left_encoder_odom=false on the Pi (no double flip). */
#define BUDDY_LEFT_ENCODER_INVERT 1

#endif
