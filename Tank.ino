#include <PID_v1.h> //PID control library
#include <NewPing.h> //Ultrasound library
#include <I2Cdev.h> //IC2 Bus library (gyro)
#include <MPU6050_6Axis_MotionApps20.h> //gyro library
#include <Wire.h> //Wire library for IC2

//Kill Switch
const int kill_pin = 6; //Pin for e-stop button
const int kill_delay = 10; //Number of milliseconds the kill button must be held
bool E_STOPPED; //Stores wether the robot has been disabled by the kill switch

//Drive
const int L_Direx1 = 3;  // pin sets direction of left motor, connected to A-In1/Phase
const int L_Direx2 = 4;  // pin sets direction of left motor, connected to A-In2/Phase
const int L_Speed = 5;   // pin sets speed of left motor, connected to A-In2/Enable
const int R_Direx1 = 7;  // pin sets direction of right motor, connected to B-In1/Phase
const int R_Direx2 = 8;  // pin sets direction of left motor, connected to B-In2/Phase
const int R_Speed = 9;   // pin sets speed of right motor, connected to B-In2/Enable


//Motion Processing Unit
MPU6050 mpu; //mpu object itself
const int MPU_FAIL = 271828; //arbitrary value to identify gyro failure

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {mpuInterrupt = true;}
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//Ultrasound
const int sensor_0 = 10; //pin for ultrasound sensor 1
const int sensor_0a =11;  //pin for ultrasound sensor 1
const int sensor_1 = 12; //pin for ultrasound sensor 2
const int sensor_1a =13; //pin for ultrasound sensor 2
const int MAX_DISTANCE = 200; //maximum range (in cm) for ultrasound
const int MIN_DISTANCE = 20; //distance in cm at which the robot should turn around

NewPing front_sonar(sensor_0a, sensor_0, MAX_DISTANCE); //forward ultrasound sensor
NewPing back_sonar(sensor_1a, sensor_1, MAX_DISTANCE);  //reverse ultrasound sensor

//Pressure Plates -the first number represents the analog pin, the +14 makes them digital
const int pFront = 1 +14;
const int pBack = 0 +14;
const int pLeft = 2 +14;
const int pRight = 3 +14;

//Drive control variables
bool forward; //Stores whether or not the robot is going forward
int velocity; //Stores the target speed of the robot
int lastReading; //System time in milliseconds of the last gyro reading
bool isClear; //Stores if the robot is already moving along a straight path
int lastUpdate; //Stores the last time the robot told the serial its status

//PID coefficients for driving straight
const double p_s = 10000.0;
const double i_s = 10000.0;
const double d_s = 10000.0;
//PID coefficients for turns
const double p_t = 1.0;
const double i_t = 0.0;
const double d_t = 0.0;
//PID control values
double set, in, out;
PID pid_s(&set, &in, &out, p_s, i_s, d_s, DIRECT);
PID pid_t(&set, &in, &out, p_t, i_t, d_t, DIRECT);  

void driveSetup(){
  //Set all drive pins to output
  pinMode(L_Direx1, OUTPUT);
  pinMode(L_Direx2, OUTPUT);
  pinMode(R_Direx1, OUTPUT);
  pinMode(R_Direx2, OUTPUT);
  pinMode(L_Speed, OUTPUT);
  pinMode(R_Speed, OUTPUT);

  forward = true; //robot should start moving forward
  velocity = 179; //base forward velocity (not maxed out so that PID has room to change values)
  lastReading = 0; //Ensures that a gyro reading will be taken at the first opportunity
  lastUpdate = 0; //Ensures that the user will be updated at the earliest opportunity
  isClear = true; //Stores if the robot is moving forward unobstructed
}

void buttonSetup(){
  pinMode(kill_pin, INPUT);
  pinMode(pFront, INPUT);
  pinMode(pBack, INPUT);
  pinMode(pLeft, INPUT);
  pinMode(pRight, INPUT);
  E_STOPPED = false;  
}

//Gyro setup code comes from the examples provided with the MPU6050 library
void mpuSetup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    /*
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    */

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void pidSetup(){

  in = readYaw();
  set = 0;
  out = 1.0;
  
  pid_t.SetMode(AUTOMATIC);
  pid_s.SetMode(AUTOMATIC);

  pid_s.SetSampleTime(100);

  pid_t.SetOutputLimits(-255,255);
  pid_s.SetOutputLimits(-76,76);
  
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setting up drive");
  driveSetup();
  Serial.println("Setting up buttons");
  buttonSetup();
  Serial.println("Setting up MPU");
  //TODO mpuSetup();
  
  //Delay 10 seconds for gyro to stabilize
  
  /*for(int i = 0; i<10; i++){
    Serial.println("Robot will move in " + String(10-i) + " seconds");
    delay(1000);
  }*/

  Serial.println("Starting!");
  pidSetup(); //Must occur after gyro setup for setpoint initialization
  Serial.println("PID Started");
}

//Uses code provided with the MPU6050 examples
float readYaw(){
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize){}

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        //Output yaw reading
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        return ypr[0]* 180/M_PI;
    }
    //If no yaw was calculated, return a failure indicating value
    return MPU_FAIL;
}
 
void driveTank(int left, int right){
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

  //Make value positive
  left = abs(left);
  right = abs(right);

  //Drive the motors
  analogWrite(L_Speed, left); 
  analogWrite(R_Speed, right); 
}

void turn(bool isReversed){
  //TODO!!! Wut
  const double GOOD_DIF = 0.1;
  
  set = (45.0 * (isReversed? -255 : 255)) + readYaw();
  in = readYaw();
  
  while(abs(set-out)>GOOD_DIF){
    //pid_t.Compute();
    driveTank(out,-out);
  }
}

void loop() {
  if(!E_STOPPED){
    //If any of the buttons are being triggered, turn away from that direction
    //If ultrasound in the direction of movement is too close, turn around
    //Then, just drive straight

    //TODO- test and configure ultrasound, buttons, turning
    
    //Turn at side collision
    if(digitalRead(pLeft)==LOW){ 
      //turn(!forward);
      Serial.println("LEFT COLLIDE");
      isClear = false;
    }
    if(digitalRead(pRight)==LOW){
      //turn(forward);
      Serial.println("RIGHT COLLIDE");
      isClear=false;
    }
    
    //Change direction at front/back interaction
    if(front_sonar.ping_cm()<MIN_DISTANCE
       ||digitalRead(pFront)==LOW){
      Serial.println("FRONT COLLIDE");
      forward = false;
    }
    if(back_sonar.ping_cm()<MIN_DISTANCE
       ||digitalRead(pBack)==LOW){
      Serial.println("BACK COLIDE");
      forward = true;
    }
    
    

    //Drive straight

    //If we just started going straight, store the current angle as the target
    /*
    if(!isClear){
      set = readYaw();
      isClear = true;
    }
    */
    
    //Update the gyro reading
    double angle = readYaw();
    in = angle;
    
    velocity = forward? abs(velocity) : -abs(velocity);
    pid_s.Compute();
    int left = velocity + out;
    int right = velocity - out;
    
    driveTank(left, right);    

    //Update the serial with drive status every 2 seconds
    int currentTime = millis();
    if(currentTime - lastUpdate > 2000){
      /*Serial.print("Driving with power ");
      Serial.print("("+String(left)+", "+String(right)+")"); //=> "(%left, %right)"
      Serial.println("With angle "+String(angle)+" and target "+String(angle));
      Serial.println("Offset! " + String(out));*/
    }
   
  }else{
    Serial.println("EMERGENCY STOPPED: Restart Robot");
  }

  
  if(digitalRead(kill_pin)==LOW){
    delay(kill_delay);
    if(digitalRead(kill_pin)==LOW){
      E_STOPPED = true;
    }
  }

}
