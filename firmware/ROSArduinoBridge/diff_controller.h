/* Functions and type-defs for PID control.
 *
 * Taken mostly from Mike Ferguson's ArbotiX code.
 * Buddy: velocity PID ported from the joey_v1 I2C firmware. PWM is not floored at a
 * breakaway value — it starts near zero and the integral term raises it while the
 * wheel lags its target, so a wheel held back by load is pushed progressively
 * harder instead of being kicked. Acceleration shaping belongs to
 * diff_drive_controller on the Pi; this loop only tracks the commanded speed.
 */

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count
  int PrevInput;                 // last input (encoder delta)
  long ITerm;                    // integrated term, scaled by Ko
  int PrevTargetSign;            // sign of last non-zero target
  long output;                   // last motor setting
} SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters — sent from ROS via "u Kp:Kd:Ki:Ko" on activate.
 *
 * Ki is what ramps PWM: a shortfall of 1 tick/frame adds Ki/Ko PWM every 50 Hz
 * frame, so PWM climbs ~150/s at Ki=150 Ko=50 and keeps climbing while stalled.
 * Raise Ki to push through load sooner (turns under body weight), lower for gentler start.
 */
int Kp = 100;
int Kd = 40;
int Ki = 150;
int Ko = 50;

/* Coast-down step per frame while the closed loop is idle (m 0 0 or auto-stop). */
const int PWM_SLEW_IDLE = 8;

unsigned char moving = 0; // closed-loop PID active (m command)
unsigned char raw_pwm_active = 0; // open-loop o command — do not override in updatePID

static int clamp_pwm(long value)
{
  if (value > MAX_PWM) {
    return MAX_PWM;
  }
  if (value < -MAX_PWM) {
    return -MAX_PWM;
  }
  return (int)value;
}

static int slew_pwm(int target, int current, int max_up, int max_down)
{
  const int delta = target - current;
  if (delta > max_up) {
    return current + max_up;
  }
  if (delta < -max_down) {
    return current - max_down;
  }
  return target;
}

/*
 * Initialize PID variables to zero to prevent startup spikes
 * when turning PID on to start moving
 */
void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoderRosLeft();
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.output = 0;
   leftPID.PrevInput = 0;
   leftPID.ITerm = 0;
   leftPID.PrevTargetSign = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoderRosRight();
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
   rightPID.PrevTargetSign = 0;
}

/* Velocity PID. PWM comes from P/D trim plus an integral term that accumulates
 * for as long as the encoder delta stays below the commanded ticks per frame. */
void doPID(SetPointInfo * p) {
  const int input = p->Encoder - p->PrevEnc;
  const int target = (int)lround(p->TargetTicksPerFrame);

  if (target == 0) {
    p->PrevEnc = p->Encoder;
    p->PrevInput = input;
    p->ITerm = 0;
    p->PrevTargetSign = 0;
    p->output = 0;
    return;
  }

  /* A reversal must not inherit push accumulated in the other direction. */
  const int target_sign = (target > 0) ? 1 : -1;
  if (p->PrevTargetSign != 0 && p->PrevTargetSign != target_sign) {
    p->ITerm = 0;
  }
  p->PrevTargetSign = target_sign;

  const int perror = target - input;

  p->ITerm += (long)Ki * perror;
  const long i_limit = (long)MAX_PWM * Ko;
  if (p->ITerm > i_limit) {
    p->ITerm = i_limit;
  } else if (p->ITerm < -i_limit) {
    p->ITerm = -i_limit;
  }

  /* If encoder sign is wrong for this wheel, integral windup saturates PWM and
   * that side spins away on every turn. Clamp alone cannot fix it — fix wiring
   * or BUDDY_LEFT_ENCODER_INVERT so input and target share a sign when moving. */
  const long out =
    ((long)Kp * perror - (long)Kd * (input - p->PrevInput) + p->ITerm) / Ko;

  p->PrevEnc = p->Encoder;
  p->PrevInput = input;
  p->output = clamp_pwm(out);
}

static int applied_left_pwm = 0;
static int applied_right_pwm = 0;

void resetAppliedPwm() {
  applied_left_pwm = 0;
  applied_right_pwm = 0;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  leftPID.Encoder = readEncoderRosLeft();
  rightPID.Encoder = readEncoderRosRight();

  /* Open-loop PWM from "o" — leave motor outputs unchanged here. */
  if (raw_pwm_active) {
    return;
  }

  if (!moving){
    /* Track the encoders while idle so the first frame after "m" sees a real
     * one-frame delta rather than every count accumulated since the last stop. */
    leftPID.PrevEnc = leftPID.Encoder;
    rightPID.PrevEnc = rightPID.Encoder;
    leftPID.PrevInput = 0;
    rightPID.PrevInput = 0;
    leftPID.ITerm = 0;
    rightPID.ITerm = 0;
    leftPID.PrevTargetSign = 0;
    rightPID.PrevTargetSign = 0;

    const int prev_left = applied_left_pwm;
    const int prev_right = applied_right_pwm;
    applied_left_pwm = slew_pwm(0, applied_left_pwm, PWM_SLEW_IDLE, PWM_SLEW_IDLE);
    applied_right_pwm = slew_pwm(0, applied_right_pwm, PWM_SLEW_IDLE, PWM_SLEW_IDLE);
    /* Also write the frame that reaches zero, otherwise the last value the driver
     * ever saw is the final non-zero step and the bridge stays energized. */
    if (applied_left_pwm != 0 || applied_right_pwm != 0 ||
        prev_left != 0 || prev_right != 0) {
      setMotorSpeeds(applied_left_pwm, applied_right_pwm);
    }
    return;
  }

  doPID(&rightPID);
  doPID(&leftPID);

  applied_left_pwm = clamp_pwm(leftPID.output);
  applied_right_pwm = clamp_pwm(rightPID.output);

  setMotorSpeeds(applied_left_pwm, applied_right_pwm);
}
