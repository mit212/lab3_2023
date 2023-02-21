/**********************************************************************************

  Arduino code to control a DC motor
  MIT Department of Mechanical Engineering, 2.004
  Rev. 1.0 -- 08/03/2020 H.C.
  Rev. 2.0 -- 08/25/2020 H.C.
  Rev. 3.0 -- 02/27/2022 H.C.

***********************************************************************************/

//#define OPEN_LOOP
#define VELOCITY_CONTROL
//#define POSITION_CONTROL

// ================================================================
// ===               PID FEEDBACK CONTROLLER                    ===
// ================================================================

// Provide PID controller gains here
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

// ================================================================
// ===               SETPOINT TYPE                              ===
// ================================================================

// Define setpoint

#define desired_vel 2.5         // desired velocity in rad/s
#define desired_pos 2*PI/3           // desired position in rad

#define square_period 3000      // square wave period in milli-seconds
#define square_on 1500          // square wave on time in milli-seconds

#define sine_amp PI/2         // sine wave amplitude
#define sine_freq_hz 0.5        // sine wave frequency in Hz

#define SQUARE_WAVE
//#define SINE_WAVE
//#define TIC

#define period_us 10000  // microseconds (1 sec = 1000000 us)

// ================================================================
// ===               SERIAL OUTPUT CONTROL                      ===
// ================================================================

// Switch between built-in Serial Plotter and Matlab streaming
// comment out both to disable serial output

#define PRINT_DATA
//#define MATLAB_SERIAL_READ

// ================================================================
// ===               DFROBOT MOTOR SHIELD DEFINITION            ===
// ================================================================

const int
// motor connected to M1
PWM_A   = 11,
DIR_A   = 13;

// ================================================================
// ===               ENCODER DEFINITION                         ===
// ================================================================

// The motor has a dual channel encoder and each has 12 counts per revolution (CPR)
// The effective resolution is 24 CPR with double decoding.
// With gear reduction, the output shaft has 98 * 48 = 2352 CPR

float C2Rad = 98 * 12 * 2 / (2 * PI); // TO DO: replace it with your conversion factor

// Encoder outputs to interrupt pins with channel A to D2 (INT0) and B to 5 (INT1)
const byte encoder0pinA = 2;
const byte encoder0pinB = 5;

long   count = 0;
volatile long counter = 0;    // Inrerrupt counter (position)

// ================================================================
// ===               VARIABLE DEFINITION                        ===
// ================================================================

#define vc2pwm 51.0    // pwm = 255 corresponds to 5v from pwm pin (255/5)

float wheel_pos = 0.0, wheel_vel = 0.0, pre_wheel_pos = 0.0, filt_vel = 0.0;
float error, sum_error = 0.0, d_error = 0.0, filt_d_error = 0.0, error_pre;
float alpha = 0.5;    // low pass filter constant
unsigned long timer;
double loop_time = 0.01;
float Pcontrol, Icontrol, Dcontrol;
float pwm;
float vc;
float set_point = desired_vel, sensor;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  Serial.begin(115200);
  #ifdef PRINT_DATA
  // assign signal labels
   Serial.println("Set_Point:,Sensor:,PWM_Voltage:");
   #endif

  // configure motor shield M1 outputs
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);

  // initialize encoder
  pinMode(encoder0pinA, INPUT);
  pinMode(encoder0pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0pinA), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0pinB), ISR_B, CHANGE);

  delay(10);    // delay 0.01 second
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (micros() - timer >= period_us) {
  timer = micros();
  count = counter;

  // wheel position and velocity from the encoder counts
  wheel_pos = count;
  wheel_pos = wheel_pos / C2Rad;    // convert to radians
  wheel_vel = (wheel_pos - pre_wheel_pos) / loop_time;
  filt_vel = alpha * wheel_vel + (1 - alpha) * filt_vel;
  pre_wheel_pos = wheel_pos;

  // ================================================================
  // ===                    GET SETPOINT                          ===
  // ================================================================

  // define set point as a square wave input
#ifdef SQUARE_WAVE
  if (millis() % square_period < square_on){
   #ifdef VELOCITY_CONTROL
    set_point = desired_vel;
  #endif
  #ifdef POSITION_CONTROL
    set_point = desired_pos;
  #endif
  }
  else
    set_point = 0.0;
#endif

  // define set point as a sine wave with an amplitude and frequency
#ifdef SINE_WAVE
  set_point = sine_amp * sin(sine_freq_hz * (2 * M_PI) * millis() / 1000.0);
#endif

#ifdef TIC
  set_point = millis() / 1000 * PI / 6;
#endif

  // ================================================================
  // ===                    CONTROLLER CODE                       ===
  // ================================================================

  // PID controller code
  #ifdef VELOCITY_CONTROL
    sensor = filt_vel;
  #endif
  #ifdef POSITION_CONTROL
    sensor = wheel_pos;
  #endif
  
  error = set_point - sensor;
  d_error = (error - error_pre) / loop_time;
  filt_d_error = alpha * d_error + (1 - alpha) * filt_d_error;
  error_pre = error;
  sum_error += error * loop_time;

  Pcontrol = error * Kp;
  Icontrol = sum_error * Ki;
  Dcontrol = d_error * Kd;    // use d_error to avoid delay

  Icontrol = constrain(Icontrol, -255, 255);    // anti-windup for integrator
  pwm =  Pcontrol + Icontrol + Dcontrol;

#ifdef OPEN_LOOP
  pwm = 150;
#endif

  // ================================================================
  // ===                    MOTOR PWM CODE                        ===
  // ================================================================

  vc = constrain(pwm / vc2pwm, -5, 5);

  // set the correct direction
  if (pwm > 0) {
    digitalWrite(DIR_A, HIGH);
  }
  else {
    digitalWrite(DIR_A, LOW);
  }

  // constrain pwm to +/- 230
  pwm = constrain(pwm, -255, 255);

  analogWrite(PWM_A, (int) abs(pwm));     // Set speed

#ifdef PRINT_DATA
  Serial.print(set_point);  Serial.print("\t");
  Serial.print(sensor); Serial.print("\t");
  //    Serial.print(filt_vel); Serial.print("\t");
  //    Serial.print(error);  Serial.print("\t");
  //    Serial.print(d_error);  Serial.print("\t");
  Serial.print(vc);  Serial.print("\t");
  //    Serial.print(pwm);  Serial.print("\t");
  //    Serial.print(loop_time,6); Serial.print("\t");
  Serial.print("\n");
#endif

#ifdef MATLAB_SERIAL_READ
  Serial.print(timer / 1000000.0);   Serial.print("\t");
  Serial.print(set_point);    Serial.print("\t");
  Serial.print(sensor);    Serial.print("\t");
  Serial.print(vc);
  Serial.print("\n");
#endif

  //  delay(8);
  }
  loop_time = (micros() - timer) / 1000000.0;  //compute actual sample time
}

// ================================================================
// ===                   Interrupt Service Routines             ===
// ================================================================

// Interrupt Service Routine for encoder A (pin 2)
void ISR_A() {
  boolean stateA = digitalRead(encoder0pinA) == HIGH;
  boolean stateB = digitalRead(encoder0pinB) == HIGH;
  if ((!stateA && stateB) || (stateA && !stateB)) counter++;
  else counter--;
}

// Interrupt Service Routine for encoder B (pin 3)
void ISR_B() {
  boolean stateA = digitalRead(encoder0pinA) == HIGH;
  boolean stateB = digitalRead(encoder0pinB) == HIGH;
  if ((!stateA && !stateB) || (stateA && stateB)) counter++;
  else counter--;
}
