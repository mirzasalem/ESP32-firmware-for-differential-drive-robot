/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE

#include "buddy_robot_config.h"

#ifdef POLOLU_VNH5019
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined L298_MOTOR_DRIVER
  void initMotorController() {
    pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
    pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
    pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACKWARD, 0);
  }

  /* Same H-bridge drive for OUT1/2 and OUT3/4 (matches working OUT1/2 behavior). */
  static void drive_hbridge(int pin_fwd, int pin_rev, unsigned char reverse, int spd) {
    if (reverse == 0) {
      analogWrite(pin_fwd, spd);
      analogWrite(pin_rev, 0);
    } else {
      analogWrite(pin_rev, spd);
      analogWrite(pin_fwd, 0);
    }
  }

  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
    int pin_fwd;
    int pin_rev;

    if (spd < 0) {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255) {
      spd = 255;
    }

    if (i == LEFT) {
      /* OUT1/2 — GPIO 18/19 */
      pin_fwd = LEFT_MOTOR_FORWARD;
      pin_rev = LEFT_MOTOR_BACKWARD;
    } else {
      /* OUT3/4 — GPIO 32/33, same logic as OUT1/2 */
      pin_fwd = RIGHT_MOTOR_FORWARD;
      pin_rev = RIGHT_MOTOR_BACKWARD;
    }
    drive_hbridge(pin_fwd, pin_rev, reverse, spd);
  }
  
  // Buddy chassis: see buddy_robot_config.h (BUDDY_L298_MOTOR_CROSS).
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
#if BUDDY_L298_MOTOR_CROSS
    setMotorSpeed(RIGHT, leftSpeed);   /* ROS left joint  -> OUT3/4 */
    setMotorSpeed(LEFT, rightSpeed);   /* ROS right joint -> OUT1/2 */
#else
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
#endif
  }
#elif defined CYTRON_MDD3A

  /* Include the Pololu library */
  #include "CytronMotorDriver.h"

  /* Configure the motor driver. */
  CytronMD motor_left(PWM_PWM, M1A, M1B);
  CytronMD motor_right(PWM_PWM, M2A, M2B);
  
  void initMotorController() {
  }
  
  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) motor_left.setSpeed(spd);
    else motor_right.setSpeed(spd);
  }

  /* A convenience function for setting both motor speeds */
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#else
  #error A motor driver must be selected!
#endif

#endif
