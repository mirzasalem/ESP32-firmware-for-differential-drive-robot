/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */

#include "buddy_robot_config.h"

#ifdef ARDUINO_ENC_COUNTER
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5
#endif

#ifdef ESP32_ENC_COUNTER
  /* Buddy: LEFT gearbox GPIO 26/27, RIGHT gearbox GPIO 16/17 */
  #define LEFT_ENC_A 26
  #define LEFT_ENC_B 27
  #define RIGHT_ENC_A 16
  #define RIGHT_ENC_B 17
#endif

#ifdef ESP32_ENC_COUNTER
  void setupEncoders();
#endif

long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

/* ROS joint order (matches serial "e" and diffdrive_arduino wheel_l_/wheel_r_) */
long readEncoderRosLeft(void);
long readEncoderRosRight(void);
void resetEncoderRosLeft(void);
void resetEncoderRosRight(void);
