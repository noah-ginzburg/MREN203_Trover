
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

double kp = 200;
double ki = 0.5*kp;
double v_des = 0;
double w_des = 0;
double vLd, vRd;
double t_now, t_last;
const int L = 0.2775;

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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(EA,OUTPUT);
  pinMode(I1,OUTPUT);
  pinMode(I2,OUTPUT);
  pinMode(EB,OUTPUT);
  pinMode(I3,OUTPUT);
  pinMode(I4,OUTPUT);

  pinMode(SIGNAL_A, INPUT);
  pinMode(SIGNAL_B, INPUT);
  pinMode(SIGNAL_C, INPUT);
  pinMode(SIGNAL_D, INPUT);

  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), decodeEncoderTicksL, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_C), decodeEncoderTicksR, RISING);
  
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  t_now = millis();
  double current_vL = RHO*2.0 * PI * ((double)encoder_ticksL / (double)TPR) * 1000.0 / (double)(t_now - t_last);
  double current_vR = (-1)*RHO*2.0 * PI * ((double)encoder_ticksR / (double)TPR) * 1000.0 / (double)(t_now - t_last);
  
  forward(current_vL, current_vR);

  // PWM command to the motor driver
  
  t_last = t_now;
  delay(100);
}

short PI_controller(double e_now, double e_int, double k_P, double k_I){
  short u;
  u = (short)(k_P * e_now + k_I * e_int);
  if (u > 255){
    u = 255;
  }
  else if (u < -255){
    u = -255;
  }
  return u;
}


short P_controller(double e_now, double k_P){
  short u;
  u = (short)(k_P * e_now);
  if (u > 255){
    u = 255;
  }
  else if (u < -255){
    u = -255;
  }
  return u;
}

void forward(double vL, double vR){
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);
  digitalWrite(I3, LOW);
  digitalWrite(I4, HIGH);
  v_des = 200;
  w_des = 0;
  vLd = v_des - (2*w_des/L);
  vRd = v_des + (2*w_des/L);
  Serial.println(vLd);
  analogWrite(EA, P_controller((vLd - vL), kp));
  analogWrite(EB, P_controller((vRd - vR), kp));

}
