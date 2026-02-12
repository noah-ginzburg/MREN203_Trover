
int EA = 5;
int I1 = 8;
int I2 = 11;
int EB = 6;
int I3 = 12;
int I4 = 13;

const byte SIGNAL_A = 10;
const byte SIGNAL_B = 9;
const byte SIGNAL_C = 2;
const byte SIGNAL_D = 3;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticksL = 0;
volatile long encoder_ticksR = 0;

double kp = 5;
double ki = 0.7 * kp;
double v_des = 0;
double w_des = 0;
double vLd, vRd;
double t_now, t_last;
double L = 0.2775;
double current_vL = 0;
double current_vR = 0;
double last_tickL = 0;
double last_tickR = 0;
double I_errorL = 0;
double I_errorR = 0;
double max_v = 0.95;

int l_pwm = 0;
int r_pwm = 0;

void decodeEncoderTicksL()
{
  if (digitalRead(SIGNAL_B) == LOW)
  {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticksL--;
  }
  else
  {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticksL++;
  }
}

void decodeEncoderTicksR()
{
  if (digitalRead(SIGNAL_D) == LOW)
  {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticksR--;
  }
  else
  {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticksR++;
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);

  pinMode(SIGNAL_A, INPUT);
  pinMode(SIGNAL_B, INPUT);
  pinMode(SIGNAL_C, INPUT);
  pinMode(SIGNAL_D, INPUT);

  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), decodeEncoderTicksL, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_C), decodeEncoderTicksR, RISING);

  delay(1000);
}

void loop()
{
  // put your main code here, to run repeatedly:
  t_now = millis();
  current_vL =
      (RHO * 2.0 * PI * (((double)encoder_ticksL - last_tickL) / (double)TPR) *
       1000.0 / (double)(t_now - t_last));
  current_vR = ((-1) * RHO * 2.0 * PI *
                (((double)encoder_ticksR - last_tickR) / (double)TPR) * 1000.0 /
                (double)(t_now - t_last));

  // current velocity = inverse of the left and right speed equatoin

  last_tickL = (double)encoder_ticksL;
  last_tickR = (double)encoder_ticksR;
  // Serial.println(current_vL);
  forward(current_vL, current_vR);

  // PWM command to the motor driver

  t_last = t_now;
  delay(100);
}

int PI_controller(double e_now, double e_int, double k_P, double k_I)
{
  int u; // u is the change in PWM to be applied
  u = (int)(k_P * e_now);

  // u = (int)(k_P * e_now + k_I * e_int);
  //  u = (u/max_v) *255;

  if (u > 255)
  {
    u = 255;
  }
  else if (u < -255)
  {
    u = -255;
  }

  // Serial.print("u = ");
  // Serial.println(u);
  // Serial.println(u);
  return u;
}

void forward(double vL, double vR)
{
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);
  digitalWrite(I3, LOW);
  digitalWrite(I4, HIGH);
  v_des = 0.95;
  w_des = 0;

  vLd = v_des - ((L * w_des) / 2);
  vRd = v_des + ((L * w_des) / 2);

  ///////////////////////////////
  double errorL = vLd - vL;
  double errorR = vRd - vR;
  I_errorL = I_errorL + errorL;
  I_errorR = I_errorR + errorR;
  Serial.print("current: ");
  Serial.print(vL);
  Serial.print("  desired: ");
  Serial.print(vLd);
  Serial.print("  error: ");
  Serial.println(errorL);

  l_pwm = l_pwm + PI_controller(errorL, I_errorL, kp,
                                ki); // current pwm + change in pwm
  r_pwm = r_pwm + PI_controller(errorR, I_errorR, kp,
                                ki); // current pwm + change in pwm
  if (l_pwm > 255)
  {
    l_pwm = 255;
  }
  else if (l_pwm < -255)
  {
    l_pwm = -255;
  }
  // analogWrite(EA, PI_controller(errorL, I_errorL, kp, ki, vL));
  // analogWrite(EB, PI_controller(errorR, I_errorR, kp, ki, vR));

  analogWrite(EA, l_pwm);
  analogWrite(EB, r_pwm);
}
