/* Functions and type-defs for PID control.
 *
 * Taken mostly from Mike Ferguson's ArbotiX code.
 * Buddy: feedforward PWM + small P correction (not cumulative output).
 */

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count
  int PrevInput;                 // last input (encoder delta)
  int ITerm;                     // integrated term
  long output;                   // last motor setting
} SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters — sent from ROS via "u Kp:Kd:Ki:Ko" on activate */
int Kp = 8;
int Kd = 2;
int Ki = 0;
int Ko = 50;

/* Max PWM change per PID frame (~30 Hz). */
const int PWM_SLEW_PER_FRAME = 15;

/* Ignore ±1 tick/frame error when target is small (encoder noise). */
const int TICK_ERROR_DEADBAND = 1;

/* Feedforward: PWM at ~15 ticks/frame (typical teleop cruise). */
const int FF_MAX_TICKS = 15;
const int FF_MAX_PWM = 170;

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

static int slew_pwm(int target, int current)
{
  const int delta = target - current;
  if (delta > PWM_SLEW_PER_FRAME) {
    return current + PWM_SLEW_PER_FRAME;
  }
  if (delta < -PWM_SLEW_PER_FRAME) {
    return current - PWM_SLEW_PER_FRAME;
  }
  return target;
}

static int feedforward_pwm(double target_ticks)
{
  if (target_ticks == 0.0) {
    return 0;
  }
  const int sign = (target_ticks > 0.0) ? 1 : -1;
  const double mag = fabs(target_ticks);
  int pwm = (int)lround((mag * FF_MAX_PWM) / FF_MAX_TICKS);
  if (pwm > 0 && pwm < 40) {
    pwm = 40; /* L298 minimum useful duty */
  }
  return sign * pwm;
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

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoderRosRight();
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
}

/* Velocity PID: feedforward + P/D correction (absolute PWM, not cumulative). */
void doPID(SetPointInfo * p) {
  const int input = p->Encoder - p->PrevEnc;
  int perror = (int)lround(p->TargetTicksPerFrame) - input;

  if (abs(perror) <= TICK_ERROR_DEADBAND &&
      fabs(p->TargetTicksPerFrame) <= 3.0) {
    perror = 0;
  }

  const int ff = feedforward_pwm(p->TargetTicksPerFrame);
  int corr = (Kp * perror - Kd * (input - p->PrevInput)) / Ko;

  if (Ki != 0) {
    p->ITerm += Ki * perror;
    const int i_limit = MAX_PWM * Ko;
    if (p->ITerm > i_limit) {
      p->ITerm = i_limit;
    } else if (p->ITerm < -i_limit) {
      p->ITerm = -i_limit;
    }
    corr += p->ITerm / Ko;
  }

  p->PrevEnc = p->Encoder;
  p->PrevInput = input;
  p->output = clamp_pwm(ff + corr);
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
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) {
      resetPID();
    }
    applied_left_pwm = slew_pwm(0, applied_left_pwm);
    applied_right_pwm = slew_pwm(0, applied_right_pwm);
    if (applied_left_pwm != 0 || applied_right_pwm != 0) {
      setMotorSpeeds(applied_left_pwm, applied_right_pwm);
    }
    return;
  }

  doPID(&rightPID);
  doPID(&leftPID);

  const int target_left = clamp_pwm(leftPID.output);
  const int target_right = clamp_pwm(rightPID.output);
  applied_left_pwm = slew_pwm(target_left, applied_left_pwm);
  applied_right_pwm = slew_pwm(target_right, applied_right_pwm);

  setMotorSpeeds(applied_left_pwm, applied_right_pwm);
}
