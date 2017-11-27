#include<PID_v1.h> //PID control library
#include<NewPing.h> //Ultrasound library


//Kill Switch
const int kill_pin = 13; //Pin for e-stop button
const int kill_delay = 10; //Number of milliseconds the kill button must be held
bool E-STOPPED; //Stores wether the robot has been disabled by the kill switch


//Drive
const int L_Direx1 = 3;            // pin sets direction of left motor, connected to A-In1/Phase
const int L_Direx2 = 4;            // pin sets direction of left motor, connected to A-In2/Phase
const int L_Speed = 5;            // pin sets speed of left motor, connected to A-In2/Enable
const int R_Direx1 = 7;            // pin sets direction of right motor, connected to B-In1/Phase
const int R_Direx2 = 8;            // pin sets direction of left motor, connected to B-In2/Phase
const int R_Speed = 9;            // pin sets speed of right motor, connected to B-In2/Enable

//Ultrasound
const int sensor_0 = 10; //pin for ultrasound sensor 1
const int sensor_1 = 11; //pin for ultrasound sensor 2
const int MAX_DISTANCE = 200; //maximum range (in cm) for ultrasound

NewPing front_sonar(sensor_0, sensor_0, MAX_DISTANCE); //forward ultrasound sensor
NewPing back_sonar(sensor_1, sensor_1, MAX_DISTANCE);  //reverse ultrasound sensor

void driveSetup(){
  //Set all drive pins to output
  pinMode(L_Direx1, OUTPUT);
  pinMode(L_Direx2, OUTPUT);
  pinMode(R_Direx1, OUTPUT);
  pinMode(R_Direx2, OUTPUT);
  pinMode(L_Speed, OUTPUT);
  pinMode(R_Speed, OUTPUT);
}

void killSetup(){
  pinMode(kill_pin, INPUT);
  E-STOPPED = false;  
}

void setup() {
  driveSetup();
  killSetup();
}
 
void driveTank(double left, double right){
  //Set direction
  if(left>0){
    digitalWrite(L_Direx1, HIGH);
    digitalWrite(L_Direx2, LOW);
  }else{
    digitalWrite(L_Direx1, LOW);
    digitalWrite(L_Direx2, HIGH);
  }
  if(right>0){
    digitalWrite(R_Direx1, HIGH);
    digitalWrite(R_Direx2, LOW);
  }else{
    digitalWrite(R_Direx1, LOW);
    digitalWrite(R_Direx2, HIGH);
  }

  //Convert % power to a range 0-255
  left = abs(left) * 255.0;
  right = abs(right) * 255.0;

  //Drive the motors
  analogWrite(L_Speed, left); 
  analogWrite(R_Speed, right); 
}

void wait(double seconds){
  delay(seconds * 1000);
}

void driveDuration(double left, double right, double seconds){
  driveTank(left, right);
  
  wait(seconds);
  
  driveTank(0,0);
}

void loop() {
  if(!E-STOPPED){ 
    //Main loop code here 
  }
  if(digitalRead(kill_pin)==LOW){
    delay(kill_delay);
    if(digitalRead(kill_pin)==LOW){
      E-STOPPED = true;
    }
  }
}
