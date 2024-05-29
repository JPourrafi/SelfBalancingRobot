#include <Arduino.h>
#include <TinyMPU6050.h>
#include <PID_v1.h>

//MPU var
MPU6050 mpu (Wire);
int Y;

//PID
#define PID_MIN_LIMIT -255 
#define PID_MAX_LIMIT 255  
#define PID_SAMPLE_TIME_IN_MILLI 10 

double Setpoint, Input, Output;
double Kp=10, Ki=0, Kd=0.02;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Motor
unsigned const int EN1=3;
unsigned const int IN11=5;
unsigned const int IN12=6;

unsigned const int EN2=9;
unsigned const int IN21=8;
unsigned const int IN22=7;


int min_motorSpeed = 25;

void motorrotation(int motors/*,motors2*/){
  if(motors < 0){
    digitalWrite(IN11, HIGH);
    digitalWrite(IN12, LOW);

    digitalWrite(IN21, HIGH);
    digitalWrite(IN22, LOW);
  }
  else if(motors >= 0){
    digitalWrite(IN11, LOW);
    digitalWrite(IN12, HIGH);

    digitalWrite(IN21, LOW);
    digitalWrite(IN22, HIGH);
  }
  /*if(motors2 < 0){
    digitalWrite(IN21, HIGH);
    digitalWrite(IN22, LOW);
  }
  else if(motors2 >= 0){
  digitalWrite(IN21, LOW);
  digitalWrite(IN22, HIGH);
  }*/
  motors = abs(motors) + min_motorSpeed;
  //motors2 = abs(motors2) + min_motorSpeed;

  motors = constrain(motors, min_motorSpeed, 255);
  //motors2 = constrain(motors2, min_motorSpeed, 255);
    
  analogWrite(EN1,motors);
  analogWrite(EN2, motors); 
}

void setup() {
  //MPU
  mpu.Initialize();
  Serial.begin(115200);
  mpu.Calibrate();
  mpu.Execute();
  Y = mpu.GetAngY();

  // ءMotor setup
  pinMode(IN11, OUTPUT);
  pinMode(IN12, OUTPUT);
  pinMode(IN21, OUTPUT);
  pinMode(IN22, OUTPUT);

 /* digitalWrite(IN11, HIGH);
  digitalWrite(IN12, LOW);
  digitalWrite(IN21, HIGH);
  digitalWrite(IN22, LOW);
*/
  pinMode(EN1, OUTPUT);
  pinMode(EN1, OUTPUT);

  motorrotation(0);

  //PID
  myPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  Input = Y;
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu.Execute();
  Y = mpu.GetAngY();
  Input = Y;
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.Compute();
  motorrotation(Output);

  // Debugging prints
  Serial.print("Angle: ");
  Serial.print(Y);
  Serial.print(" Output: ");
  Serial.println(Output);

  delay(10);
  //delay(PID_SAMPLE_TIME_IN_MILLI);

}
