#include <IMU_lib2.h>
#include <CF_lib3.h>
#include <RobotOrekaria_lib.h>
#include <dogm_7036.h>

//PINEN KONFIGURAZIOA
#define drv1DIR 12
#define drv1STEP 11
#define drv1MS1 10
#define drv1MS2 9
#define drv1MS3 8
#define drv2DIR 6
#define drv2STEP 5
#define drv2MS1 4
#define drv2MS2 1
#define drv2MS3 0

//MOTORRAK
#define MICROSTEPS 16
#define ZERO_SPEED 65535
#define VEL_MAX 500
#define MAX_ACCEL 7

int16_t speed_M1, speed_M2; 
int16_t motor1;
int16_t motor2;

//TIMERRAK
#define SET(x,y) (x|=(1<<y))
#define CLR(x,y) (x&=(~(1<<y)))
float tiempoActual,tiempo;

volatile float t3, t4;

//PID KONTROLA
double Setpoint, Input, Output;
double Kp=300, Ki=215, Kd=213;
float integralSum, PID_errorSum;
float errorAnterior, errorAnt;
#define ITERM_MAX_ERROR 25   
#define ITERM_MAX 8000 

//ENKODER BIRAKARIA
const int encoderPin4 = 19;
const int encoderPin5 = 18;
const int encoderSwitchPin = 13;

volatile int lastEncoded = 0;
volatile double encoderValue = 0;
volatile bool flag = false;

int lastMSB = 0;
int lastLSB = 0;

int aux = 0;

//IMU
float anguloActual;

//LIBURUTEGIEN IRAKURKETARAKO
IMU_lib2 imu_lib2;
CF_lib3 cf_lib3;
dogm_7036 DOG;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
 
  pinMode(drv1DIR, OUTPUT);
  pinMode(drv1STEP, OUTPUT);
  pinMode(drv1MS1, OUTPUT);
  pinMode(drv1MS2, OUTPUT);
  pinMode(drv1MS3, OUTPUT);
  
  digitalWrite(drv1DIR, HIGH);
  digitalWrite(drv1STEP, LOW);
  digitalWrite(drv1MS1, HIGH);
  digitalWrite(drv1MS2, HIGH);
  digitalWrite(drv1MS3, HIGH);

  pinMode(drv2DIR, OUTPUT);
  pinMode(drv2STEP, OUTPUT);
  pinMode(drv2MS1, OUTPUT);
  pinMode(drv2MS2, OUTPUT);
  pinMode(drv2MS3, OUTPUT);
  
  digitalWrite(drv2DIR, HIGH);
  digitalWrite(drv2STEP, LOW);
  digitalWrite(drv2MS1, HIGH);
  digitalWrite(drv2MS2, HIGH);
  digitalWrite(drv2MS3, HIGH);
  
  imu_lib2.begin();
  int* accel = imu_lib2.Read_Accel();
  int* gyro = imu_lib2.Read_Gyro();
  int* noise = imu_lib2.Get_Noise();
  cf_lib3.begin(accel);

  Setpoint = 1.9;
  
  LCD_init(&DOG, Kp, Ki, Kd, Setpoint, Input, Output);

  pinMode(encoderPin4, INPUT_PULLUP); 
  pinMode(encoderPin5, INPUT_PULLUP);
  pinMode(encoderSwitchPin, INPUT_PULLUP);

  //on interrupt 4 (pin 19), or interrupt 5 (pin 18) 
  attachInterrupt(4, updateEncoder, CHANGE); 
  attachInterrupt(5, updateEncoder, CHANGE);  

  while(aux<5)
    Param();
  if(aux>4) { 
    aux=0;
    DOG.cursor_onoff(false);
  }

  for(int i=0;i<100;i++)
  {
      float tiempoAnterior = tiempoActual;
      tiempoActual = micros();
      float tiempoPasado = (tiempoActual - tiempoAnterior);                 
      tiempo  = tiempoPasado / 1000;                                       
      anguloActual = Input;                                            //Angulo inicial.
  }
  
  set_timer3_interrupt();
  set_timer4_interrupt();
  t3 = micros();
  t4 = micros();
}

void loop() {
  Param();
  if(aux>4) { 
    aux=0;
    DOG.cursor_onoff(false);
  }
  
  int* accel = imu_lib2.Read_Accel();
  int* gyro = imu_lib2.Read_Gyro();
  int* noise = imu_lib2.Get_Noise();
  Input= cf_lib3.Compute_Roll(accel, *gyro, *(noise+3), 0.98);

  float tiempoAnterior = tiempoActual;
  tiempoActual = micros();
  float tiempoPasado = (tiempoActual - tiempoAnterior);                 //tiempo pasado desde ciclo anterior en microsegundos
  tiempo  = tiempoPasado / 1000;  
  //Serial.println(tiempo);//tiempo del ciclo en milisegundos
  anguloActual = Input;

  float Output = PID (anguloActual, Setpoint, Kp, Kd, Ki, tiempo);

  IO_update(&DOG, Input, Output, aux);
  
  if ((Input < 55) && (Input > -55)) // Is robot ready (upright?)
  {
    // NORMAL MODE
    motor1= Output;
    motor2= Output;
    motor1 = constrain(motor1, -VEL_MAX, VEL_MAX);
    motor2 = constrain(motor2, -VEL_MAX, VEL_MAX);
    setMotorSpeedM1(motor1);
    setMotorSpeedM2(motor2);
  }else{   // Robot not ready (flat), angle > 70º => ROBOT OFF
    setMotorSpeedM1(0);
    setMotorSpeedM2(0);
    integralSum = 0;
    PID_errorSum = 0;
  }
}
void set_timer3_interrupt() {
  cli(); // disable global interrupts
  TCCR3A &= (~(_BV(COM3A1)) & ~(_BV(COM3A0)));
  TCCR3A &= (~(_BV(WGM30)) & ~(_BV(WGM31)));
  
  TCCR3B |= _BV(WGM32);
  TCCR3B &= ~(_BV(WGM33));
  
  TCCR3B |= ((_BV(CS32))|(_BV(CS30)));
  TCCR3B &= ~(_BV(CS31));

  OCR3A = 65535; 
  
  TCNT3 = 0;
  TIMSK3 |= 1<<OCIE3A; // Enable Timer2 interrupt
 
  sei(); // enable interrupts
}

void set_timer4_interrupt() {
  cli(); // disable global interrupts
  TCCR4A &= (~(_BV(COM4A1)) & ~(_BV(COM4A0)));
  TCCR4A &= (~(_BV(WGM40)) & ~(_BV(WGM41)));
  
  TCCR4B |= _BV(WGM42);
  TCCR4B &= ~(_BV(WGM43));
  
  TCCR4B |= ((_BV(CS42))|(_BV(CS40)));
  TCCR4B &= ~(_BV(CS41));

  OCR4A = 65535; 
  
  TCNT4 = 0;
  TIMSK4 |= 1<<OCIE4A; // Enable Timer2 interrupt
 
  sei(); // enable interrupts
}

ISR(TIMER3_COMPA_vect) {
  cli();
  /*
  float now = micros();
  Serial.print((now-t3));
  t3 = now;
  */
  SET(PORTE, 3); // STEP MOTOR 1 (pin 5 --> PE3)
  delay_1us();
  CLR(PORTE, 3);
  
  /*SET(PORTB, 5); // STEP MOTOR 2 (pin 11 --> PB5)
  delay_1us();
  CLR(PORTB, 5); */
  
  sei();
}

ISR(TIMER4_COMPA_vect) {
  cli();
  /*float now = micros();
  Serial.print(" "); Serial.println((now-t4));
  t4 = now;
  */
  SET(PORTB, 5); // STEP MOTOR 2 (pin 11 --> PB5)
  delay_1us();
  CLR(PORTB, 5); 
  
  sei();
}

void delay_1us() {
  __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
}

void Param() {
  if(!digitalRead(encoderSwitchPin)) 
    Param_select(&DOG, Setpoint, Kp, Ki, Kd, &encoderValue, &aux);
  
  if(flag) {
    Param_update(&DOG, &Setpoint, &Kp, &Ki, &Kd, encoderValue, aux);
    PIDSetTunings(Kp, Ki, Kd);
    flag = false;
  }
}

// ISR de interrupts 4 y 5

void updateEncoder() {
  int MSB = digitalRead(encoderPin4); //MSB = most significant bit
  int LSB = digitalRead(encoderPin5); //LSB = least significant bit
  
  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
    encoderValue += 0.1;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue -= 0.1;

  lastEncoded = encoded; //store this value for next time

  flag = true;
}

void setMotorSpeedM1(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M1 - tspeed) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;
    
  speed = speed_M1 * 16; // Adjust factor from control output speed to real motor speed in steps/second

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
  }
  else if (speed > 0)
  {
    timer_period = 15625 / speed; // 2Mhz timer
    SET(PORTB, 6); // DIR Motor 1 (Forward)--> pin12
  }
  else
  {
    timer_period = 15625 / -speed;
    CLR(PORTB, 6); // Dir Motor 1 --> pin12
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;

  OCR3A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT3 > OCR3A)
    TCNT3 = 0;
}

void setMotorSpeedM2(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M2 - tspeed) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;

  speed = speed_M2 * 16; // Adjust factor from control output speed to real motor speed in steps/second

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
  }
  else if (speed > 0)
  {
    timer_period = 15625 / speed; // 2Mhz timer
    CLR(PORTH, 3);   // Dir Motor2 (Forward) --> pin6
  }
  else
  {
    timer_period = 15625 / -speed;
    SET(PORTH, 3);  // DIR Motor 2 --> pin6
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;

  OCR4A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT4 > OCR4A)
    TCNT4 = 0;
}

float PID (float anguloActual, float anguloDeseado, float Kp, float Kd, float Ki, float tiempo) {
  float errorActual;
  float salida;
  float output;
  
  errorActual = anguloDeseado - anguloActual;
  float proporcional = errorActual * Kp * 0.1;
  integralSum = integralSum + errorActual ;
  float integral = Ki * integralSum * tiempo * 0.001;
  integral = constrain (integral, -VEL_MAX, VEL_MAX);
  float derivativo = Kd * (errorActual - errorAnterior) / tiempo ;
  errorAnterior = errorActual;
  output = constrain((proporcional - derivativo + integral), -VEL_MAX, VEL_MAX);
  //if (tiempo > 10000) {Serial.print("KP:");Serial.print(proporcional);Serial.print("    Ki:");Serial.print(integral);Serial.print("   KD:");Serial.print( derivativo);Serial.print("   OUT:");Serial.println(output);
 // }
  return (output);
}


void PIDSetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
   
   double SampleTimeInSec = ((double)tiempo)/1000;  
   Kp = Kp;
   Ki = Ki * SampleTimeInSec;
   Kd = Kd / SampleTimeInSec;
 
  if((drv1DIR ==LOW )&& (drv2DIR==LOW))
   {
      Kp = (0 - Kp);
      Ki = (0 - Ki);
      Kd = (0 - Kd);
   }
}


