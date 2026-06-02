/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  // ESP32 GPIO — change if these conflict with your wiring
  // Avoid: 16,17,26,27 (encoders), 0,3,45,46 (strap/USB)
  #define LEFT_MOTOR_FORWARD    18
  #define LEFT_MOTOR_BACKWARD   19
  #define RIGHT_MOTOR_FORWARD   32
  #define RIGHT_MOTOR_BACKWARD  33
  #define LEFT_MOTOR_ENABLE     25   // ENA — or jumper ENA high on module
  #define RIGHT_MOTOR_ENABLE    14   // ENB
#endif

#ifdef CYTRON_MDD3A

  /* Include the Pololu library */
  #include "CytronMotorDriver.h"
  
  #define M1A 18
  #define M1B 19
  #define M2A 32
  #define M2B 33

  extern CytronMD motor_left;
  extern CytronMD motor_right;
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
