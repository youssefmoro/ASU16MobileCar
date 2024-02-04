#include<SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_EULER

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float new_value, dist; 
int angle;
String string_move_distance, readvoice;
int move_distance, diff, rest_angle, rest_dist;
int LED=8;
int LED2=5;
int readangle;


//FOR IR SENSORS
int irL=7;
int irC=3;
int irR=4;
int L,C,R;

int sensor[3]={0,0,0};

char current_state;
int low_speed=200;

//MOTOR PINS
int input1=12;
int input2=13;
int input3=A3;
int input4=A2;

//ENABLE PINS
int enableA=6;
int enableB=A0;
int enableB2=9;
int enableA2=A1;

//ULTRA SONIC PINS 
int trig=11;
int echo=10;
char data, new_data;
long duration,distance, fwd_distance;

void stop_car(){
  digitalWrite(enableA,HIGH);
  digitalWrite(enableB,HIGH);
  digitalWrite(enableB2,HIGH);
  digitalWrite(enableA2,HIGH);
  
  digitalWrite(input1,LOW);
  digitalWrite(input2,LOW);
  digitalWrite(input3,LOW);
  digitalWrite(input4,LOW);
  }
 
 //For Line Follower
 void forward(){
    analogWrite(enableB2,250);   //Left Motor Speed
    analogWrite(enableB,250);   //Left Motor Speed
    analogWrite(enableA,250);  //Right Motor Speed
    analogWrite(enableA2,250);  //Right Motor Speed

    digitalWrite(input1,HIGH);
    digitalWrite(input2,LOW);
    digitalWrite(input3,HIGH);
    digitalWrite(input4,LOW);    
  }

void Left(){
    analogWrite(enableB2,252);   //Left Motor Speed
    analogWrite(enableB,252);   //Left Motor Speed
    analogWrite(enableA,252);  //Right Motor Speed
    analogWrite(enableA2,252);  //Right Motor Speed

    digitalWrite(input1,HIGH);
    digitalWrite(input2,LOW);
    digitalWrite(input3,LOW);
    digitalWrite(input4,HIGH);    
  }

void Right(){
    analogWrite(enableB2,252);   //Left Motor Speed
    analogWrite(enableB,252);   //Left Motor Speed
    analogWrite(enableA,252);  //Right Motor Speed
    analogWrite(enableA2,252);  //Right Motor Speed

    digitalWrite(input1,LOW);
    digitalWrite(input2,HIGH);
    digitalWrite(input3,HIGH);
    digitalWrite(input4,LOW);    
  }

 //For Easy Driving
 void fwd_car(){
  digitalWrite(enableA,HIGH);
  digitalWrite(enableB,HIGH);
  digitalWrite(enableB2,HIGH);
  digitalWrite(enableA2,HIGH);
    
  digitalWrite(input1,HIGH);
  digitalWrite(input2,LOW);
  digitalWrite(input3,HIGH);
  digitalWrite(input4,LOW);
  }

 void bwd_car(){
  digitalWrite(enableA,HIGH);
  digitalWrite(enableB,HIGH);
  digitalWrite(enableB2,HIGH);
  digitalWrite(enableA2,HIGH);
   
  digitalWrite(input1,LOW);
  digitalWrite(input2,HIGH);
  digitalWrite(input3,LOW);
  digitalWrite(input4,HIGH);
  }
 void turn_right(){
  digitalWrite(enableA,HIGH);
  digitalWrite(enableB,HIGH);
  digitalWrite(enableB2,HIGH);
  digitalWrite(enableA2,HIGH);
  
  digitalWrite(input1,LOW);
  digitalWrite(input2,HIGH);
  digitalWrite(input3,HIGH);
  digitalWrite(input4,LOW); 
    }
   void turn_left(){
  digitalWrite(enableA,HIGH);
  digitalWrite(enableB,HIGH);
  digitalWrite(enableB2,HIGH);
  digitalWrite(enableA2,HIGH);
  
  digitalWrite(input1,HIGH);
  digitalWrite(input2,LOW);
  digitalWrite(input3,LOW);
  digitalWrite(input4,HIGH);
    
    }

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

   mpu.initialize();

    
   devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(38);
    mpu.setYGyroOffset(-91);
    mpu.setZGyroOffset(46);
    mpu.setZAccelOffset(1682); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {

        mpu.setDMPEnabled(true);

        
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    
    else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

      
  //FOR IR SENSORS
  pinMode(irL,INPUT);
  pinMode(irC,INPUT);
  pinMode(irR,INPUT);
  
  //FOR MOTOR
  pinMode(input1,OUTPUT);
  pinMode(input2,OUTPUT);
  pinMode(input3,OUTPUT);
  pinMode(input4,OUTPUT);
  
  //FOR UTRA SONIC
  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);

  pinMode(LED,OUTPUT);
  pinMode(LED2,OUTPUT);
}

void read_sensor_data(){
  sensor[0]=digitalRead(irR);
  sensor[1]=digitalRead(irC);
  sensor[2]=digitalRead(irL); 
  }

void mpu_calibration(){
      int equal_counter=1;
      float compare_value=0;
      while(1){
        // if programming failed, don't try to do anything
        if (!dmpReady) return;
    
        // wait for MPU interrupt or extra packet(s) available
        while (!mpuInterrupt && fifoCount < packetSize) {
        }
    
        // reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
    
        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
    
        // check for overflow
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            Serial.println(F("FIFO overflow!"));
    
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } 
        
        else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
    
    // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetEuler(euler, &q);
                new_value=euler[0]*180/M_PI;
                Serial.print("euler\t");
                Serial.println(new_value);
    
        }
        
        if(new_value==compare_value){
            equal_counter=equal_counter+1;
            compare_value=new_value;
          }
          
        else{
            equal_counter=0;
            compare_value=new_value;
          }
          
        if(equal_counter==6){
            break;
          } 
      }
      return;  
  }

// SQUARE SIDE LENGTH
void straight(int dummy_dist){
    for(int st=0; st<2; st++){
        digitalWrite(trig,LOW);
        delayMicroseconds(2);
        digitalWrite(trig,HIGH);
        delayMicroseconds(10);
        digitalWrite(trig,LOW); 
    
        duration=pulseIn(echo,HIGH);
        distance=duration/58;
        delay(300);
    }

    Serial.print("DISTANCE = ");
    Serial.println(distance);
    
    diff=distance-dummy_dist;

    while(distance>diff){
      
        fwd_car();
        
        digitalWrite(trig,LOW);
        delayMicroseconds(2);
        digitalWrite(trig,HIGH);
        delayMicroseconds(10);
        digitalWrite(trig,LOW); 
    
        duration=pulseIn(echo,HIGH);
        distance=duration/58;
        Serial.print("DISTANCE YA BEEH = ");
        Serial.println(distance);      
      }
    stop_car();
}

//TURN A 90 DEGREE FOR THE SQUARE SHAPE
void right_angle(int dummy_angle){
        dist=new_value+dummy_angle;
        
        if(dist>179){
            rest_angle=179-(new_value);
            rest_dist=(new_value)+(rest_angle);
      
            while(new_value<rest_dist){
                  digitalWrite(LED,HIGH);
                  
                  analogWrite(enableB2,190);   //Left Motor Speed
                  analogWrite(enableB,190);   //Left Motor Speed
                  analogWrite(enableA,190);  //Right Motor Speed
                  analogWrite(enableA2,190);  //Right Motor Speed
              
                  digitalWrite(input1,LOW);
                  digitalWrite(input2,HIGH);
                  digitalWrite(input3,HIGH);
                  digitalWrite(input4,LOW);    
            
                 // if programming failed, don't try to do anything
                if (!dmpReady) return;
            
                // wait for MPU interrupt or extra packet(s) available
                while (!mpuInterrupt && fifoCount < packetSize) {

                }
            
                // reset interrupt flag and get INT_STATUS byte
                mpuInterrupt = false;
                mpuIntStatus = mpu.getIntStatus();
            
                // get current FIFO count
                fifoCount = mpu.getFIFOCount();
            
                // check for overflow
                if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                    // reset so we can continue cleanly
                    mpu.resetFIFO();
                    Serial.println(F("FIFO overflow!"));
            
                // otherwise, check for DMP data ready interrupt (this should happen frequently)
                } 
                
                else if (mpuIntStatus & 0x02) {
                    // wait for correct available data length, should be a VERY short wait
                    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
            
                    // read a packet from FIFO
                    mpu.getFIFOBytes(fifoBuffer, packetSize);
                    
                    // track FIFO count here in case there is > 1 packet available
                    // (this lets us immediately read more without waiting for an interrupt)
                    fifoCount -= packetSize;
            
                        // display Euler angles in degrees
                        mpu.dmpGetQuaternion(&q, fifoBuffer);
                        mpu.dmpGetEuler(euler, &q);
                        new_value=euler[0]*180/M_PI;
                  
                        }
            }
            
            digitalWrite(LED,LOW);
            stop_car();
            delay(200);
            
            while(new_value>0){
                  digitalWrite(LED,HIGH);
                  digitalWrite(LED2,HIGH);
                  
                  analogWrite(enableB2,190);   //Left Motor Speed
                  analogWrite(enableB,190);   //Left Motor Speed
                  analogWrite(enableA,190);  //Right Motor Speed
                  analogWrite(enableA2,190);  //Right Motor Speed
              
                  digitalWrite(input1,LOW);
                  digitalWrite(input2,HIGH);
                  digitalWrite(input3,HIGH);
                  digitalWrite(input4,LOW);    
            
                      // if programming failed, don't try to do anything
                if (!dmpReady) return;
            
                // wait for MPU interrupt or extra packet(s) available
                while (!mpuInterrupt && fifoCount < packetSize) {
                }
            
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
                } 
                
                else if (mpuIntStatus & 0x02) {
                    // wait for correct available data length, should be a VERY short wait
                    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
            
                    // read a packet from FIFO
                    mpu.getFIFOBytes(fifoBuffer, packetSize);
                    
                    // track FIFO count here in case there is > 1 packet available
                    // (this lets us immediately read more without waiting for an interrupt)
                    fifoCount -= packetSize;
            
            // display Euler angles in degrees
                        mpu.dmpGetQuaternion(&q, fifoBuffer);
                        mpu.dmpGetEuler(euler, &q);
                        new_value=euler[0]*180/M_PI;
                  
                }
              }
              digitalWrite(LED,LOW);
              digitalWrite(LED2,LOW);
              
              stop_car();
              delay(200);
      
              rest_angle= dist-179;
              dist=(new_value)+(rest_angle);
              
              while(new_value<dist){
                      digitalWrite(LED2,HIGH);
                      
                      analogWrite(enableB2,190);   //Left Motor Speed
                      analogWrite(enableB,190);   //Left Motor Speed
                      analogWrite(enableA,190);  //Right Motor Speed
                      analogWrite(enableA2,190);  //Right Motor Speed
                  
                      digitalWrite(input1,LOW);
                      digitalWrite(input2,HIGH);
                      digitalWrite(input3,HIGH);
                      digitalWrite(input4,LOW);    
                
                          // if programming failed, don't try to do anything
                    if (!dmpReady) return;
                
                    // wait for MPU interrupt or extra packet(s) available
                    while (!mpuInterrupt && fifoCount < packetSize) {
                    }
                
                    // reset interrupt flag and get INT_STATUS byte
                    mpuInterrupt = false;
                    mpuIntStatus = mpu.getIntStatus();
                
                    // get current FIFO count
                    fifoCount = mpu.getFIFOCount();
                
                    // check for overflow 
                    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                        // reset so we can continue cleanly
                        mpu.resetFIFO();
                        Serial.println(F("FIFO overflow!"));
                
                    // otherwise, check for DMP data ready interrupt (this should happen frequently)
                    } 
                    
                    else if (mpuIntStatus & 0x02) {
                        // wait for correct available data length, should be a VERY short wait
                        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
                
                        // read a packet from FIFO
                        mpu.getFIFOBytes(fifoBuffer, packetSize);
                        
                        // track FIFO count here in case there is > 1 packet available
                        // (this lets us immediately read more without waiting for an interrupt)
                        fifoCount -= packetSize;
                
                // display Euler angles in degrees
                            mpu.dmpGetQuaternion(&q, fifoBuffer);
                            mpu.dmpGetEuler(euler, &q);
                            new_value=euler[0]*180/M_PI;
                      
                    }
                }
                digitalWrite(LED2,LOW);
                stop_car();
                delay(500);
          }
      
      
        else{
              while(new_value<dist){
                
                  analogWrite(enableB2,190);   //Left Motor Speed
                  analogWrite(enableB,190);   //Left Motor Speed
                  analogWrite(enableA,190);  //Right Motor Speed
                  analogWrite(enableA2,190);  //Right Motor Speed
              
                  digitalWrite(input1,LOW);
                  digitalWrite(input2,HIGH);
                  digitalWrite(input3,HIGH);
                  digitalWrite(input4,LOW);    
            
                      // if programming failed, don't try to do anything
                if (!dmpReady) return;
            
                // wait for MPU interrupt or extra packet(s) available
                while (!mpuInterrupt && fifoCount < packetSize) {
                }
            
                // reset interrupt flag and get INT_STATUS byte
                mpuInterrupt = false;
                mpuIntStatus = mpu.getIntStatus();
            
                // get current FIFO count
                fifoCount = mpu.getFIFOCount();
            
                // check for overflow
                if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                    // reset so we can continue cleanly
                    mpu.resetFIFO();
                    Serial.println(F("FIFO overflow!"));
            
                // otherwise, check for DMP data ready interrupt (this should happen frequently)
                } 
                
                else if (mpuIntStatus & 0x02) {
                    // wait for correct available data length, should be a VERY short wait
                    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
            
                    // read a packet from FIFO
                    mpu.getFIFOBytes(fifoBuffer, packetSize);
                    
                    // track FIFO count here in case there is > 1 packet available
                    // (this lets us immediately read more without waiting for an interrupt)
                    fifoCount -= packetSize;
            
            // display Euler angles in degrees
                        mpu.dmpGetQuaternion(&q, fifoBuffer);
                        mpu.dmpGetEuler(euler, &q);
                        new_value=euler[0]*180/M_PI;
                  
                }
              }
            
              stop_car();
              delay(500);
        }
      
  }

//////////////////////////////////////////////////////////////////

  
void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()==0){}
  data=Serial.read();
  Serial.print(">>>DATA : ");
  Serial.println(data);

  while(data=='F'){
    digitalWrite(trig,LOW);
    delayMicroseconds(2);
    digitalWrite(trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig,LOW); 

    duration=pulseIn(echo,HIGH);
    distance=duration/58;
    Serial.print("DISTANCE YA BEEH = ");
    Serial.println(distance); 

    if(distance<=15){
        stop_car();
        data=0;
        break;
      }
      
    if(distance>10){
      fwd_car();
      new_data=Serial.read();
      Serial.print("FWD DATA= ");
      Serial.println(new_data);

      if(new_data=='L' || new_data=='R' || new_data=='B' || new_data=='0'){
          break;
        }
      }
    }

   if(data=='0' || data=='S'){
      stop_car();
    }

  if(data=='R'){
      turn_right();
    }

  if(data=='L'){
      turn_left();
    }
  if(data=='B'){
      bwd_car();
    }

    
  while(data=='Z'){
      read_sensor_data();
      new_data=Serial.read();
      if(new_data=='0'){
          break;
        }
        
  if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)){
      Left();
      current_state='L';
    }
    
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)){
      Left();
      current_state='L';
    }

  //Forward
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==1)){
      forward(); 
      current_state='F';   
    }

  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)){
      Right();
      current_state='R';
    }
    
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)){
      Right();
      current_state='R';
    }

  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)){
      forward();
      current_state='F';
    }
    
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)){
      if(current_state=='L'){
          Left();
        }
        
       if(current_state=='R'){
          Right();
        }
        
      if(current_state=='F'){
          forward();
        }               
    }
  }
  
  while(data=='A'){
   digitalWrite(LED,HIGH);
      // To Exit the Accurate driving mode
      while(Serial.available()==0){}
      new_data=Serial.read();
      if(new_data=='0'){
          digitalWrite(LED,LOW);
          break;
        }
        
  stop_car();
  digitalWrite(LED,LOW);
  mpu_calibration();
  
  digitalWrite(LED2,HIGH);
  Serial.print("Enter the angle: ");
  while(Serial.available()==0){}
  angle=Serial.parseInt();
  digitalWrite(LED2,LOW);
  
  dist=new_value+angle;
  Serial.print("ANGLE= ");
  Serial.println(angle);
  Serial.print("CURRENT POSITION= ");
  Serial.println(new_value);  
  Serial.print("DISTENATION= ");
  Serial.println(dist);
  
  if(dist>179){
      rest_angle=179-(new_value);
      rest_dist=(new_value)+(rest_angle);

      while(new_value<rest_dist){
            digitalWrite(LED,HIGH);
            
            analogWrite(enableB2,190);   //Left Motor Speed
            analogWrite(enableB,190);   //Left Motor Speed
            analogWrite(enableA,190);  //Right Motor Speed
            analogWrite(enableA2,190);  //Right Motor Speed
        
            digitalWrite(input1,LOW);
            digitalWrite(input2,HIGH);
            digitalWrite(input3,HIGH);
            digitalWrite(input4,LOW);    
      
                // if programming failed, don't try to do anything
          if (!dmpReady) return;
      
          // wait for MPU interrupt or extra packet(s) available
          while (!mpuInterrupt && fifoCount < packetSize) {
            
          }
      
          // reset interrupt flag and get INT_STATUS byte
          mpuInterrupt = false;
          mpuIntStatus = mpu.getIntStatus();
      
          // get current FIFO count
          fifoCount = mpu.getFIFOCount();
      
          // check for overflow
          if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
              // reset so we can continue cleanly
              mpu.resetFIFO();
              Serial.println(F("FIFO overflow!"));
      
          // otherwise, check for DMP data ready interrupt (this should happen frequently)
          } 
          
          else if (mpuIntStatus & 0x02) {
              // wait for correct available data length, should be a VERY short wait
              while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      
              // read a packet from FIFO
              mpu.getFIFOBytes(fifoBuffer, packetSize);
              
              // track FIFO count here in case there is > 1 packet available
              // (this lets us immediately read more without waiting for an interrupt)
              fifoCount -= packetSize;
      
      // display Euler angles in degrees
                  mpu.dmpGetQuaternion(&q, fifoBuffer);
                  mpu.dmpGetEuler(euler, &q);
                  new_value=euler[0]*180/M_PI;
            
                  }
      }
      
      digitalWrite(LED,LOW);
      stop_car();
      delay(200);
      
      while(new_value>0){
            digitalWrite(LED,HIGH);
            digitalWrite(LED2,HIGH);
            
            analogWrite(enableB2,190);   //Left Motor Speed
            analogWrite(enableB,190);   //Left Motor Speed
            analogWrite(enableA,190);  //Right Motor Speed
            analogWrite(enableA2,190);  //Right Motor Speed
        
            digitalWrite(input1,LOW);
            digitalWrite(input2,HIGH);
            digitalWrite(input3,HIGH);
            digitalWrite(input4,LOW);    
      
                // if programming failed, don't try to do anything
          if (!dmpReady) return;
      
          // wait for MPU interrupt or extra packet(s) available
          while (!mpuInterrupt && fifoCount < packetSize) {
            
          }
      
          // reset interrupt flag and get INT_STATUS byte
          mpuInterrupt = false;
          mpuIntStatus = mpu.getIntStatus();
      
          // get current FIFO count
          fifoCount = mpu.getFIFOCount();
      
          // check for overflow
          if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
              // reset so we can continue cleanly
              mpu.resetFIFO();
              Serial.println(F("FIFO overflow!"));
      
          // otherwise, check for DMP data ready interrupt (this should happen frequently)
          } 
          
          else if (mpuIntStatus & 0x02) {
              // wait for correct available data length, should be a VERY short wait
              while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      
              // read a packet from FIFO
              mpu.getFIFOBytes(fifoBuffer, packetSize);
              
              // track FIFO count here in case there is > 1 packet available
              // (this lets us immediately read more without waiting for an interrupt)
              fifoCount -= packetSize;
      
      // display Euler angles in degrees
                  mpu.dmpGetQuaternion(&q, fifoBuffer);
                  mpu.dmpGetEuler(euler, &q);
                  new_value=euler[0]*180/M_PI;
            
          }
        }
        digitalWrite(LED,LOW);
        digitalWrite(LED2,LOW);
        
        stop_car();
        delay(200);

        rest_angle= dist-179;
        dist=(new_value)+(rest_angle);
        
        while(new_value<dist){
                digitalWrite(LED2,HIGH);
                
                analogWrite(enableB2,190);   //Left Motor Speed
                analogWrite(enableB,190);   //Left Motor Speed
                analogWrite(enableA,190);  //Right Motor Speed
                analogWrite(enableA2,190);  //Right Motor Speed
            
                digitalWrite(input1,LOW);
                digitalWrite(input2,HIGH);
                digitalWrite(input3,HIGH);
                digitalWrite(input4,LOW);    
          
                    // if programming failed, don't try to do anything
              if (!dmpReady) return;
          
              // wait for MPU interrupt or extra packet(s) available
              while (!mpuInterrupt && fifoCount < packetSize) {
                
              }
          
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
              } 
              
              else if (mpuIntStatus & 0x02) {
                  // wait for correct available data length, should be a VERY short wait
                  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
          
                  // read a packet from FIFO
                  mpu.getFIFOBytes(fifoBuffer, packetSize);
                  
                  // track FIFO count here in case there is > 1 packet available
                  // (this lets us immediately read more without waiting for an interrupt)
                  fifoCount -= packetSize;
          
          // display Euler angles in degrees
                      mpu.dmpGetQuaternion(&q, fifoBuffer);
                      mpu.dmpGetEuler(euler, &q);
                      new_value=euler[0]*180/M_PI;
                
              }
          }
          digitalWrite(LED2,LOW);
          stop_car();
          delay(1000);
    }


  else{
        while(new_value<dist){
          
            analogWrite(enableB2,190);   //Left Motor Speed
            analogWrite(enableB,190);   //Left Motor Speed
            analogWrite(enableA,190);  //Right Motor Speed
            analogWrite(enableA2,190);  //Right Motor Speed
        
            digitalWrite(input1,LOW);
            digitalWrite(input2,HIGH);
            digitalWrite(input3,HIGH);
            digitalWrite(input4,LOW);    
      
                // if programming failed, don't try to do anything
          if (!dmpReady) return;
      
          // wait for MPU interrupt or extra packet(s) available
          while (!mpuInterrupt && fifoCount < packetSize) {
 
          }
      
          // reset interrupt flag and get INT_STATUS byte
          mpuInterrupt = false;
          mpuIntStatus = mpu.getIntStatus();
      
          // get current FIFO count
          fifoCount = mpu.getFIFOCount();
      
          // check for overflow
          if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
              // reset so we can continue cleanly
              mpu.resetFIFO();
              Serial.println(F("FIFO overflow!"));
      
          // otherwise, check for DMP data ready interrupt (this should happen frequently)
          } 
          
          else if (mpuIntStatus & 0x02) {
              // wait for correct available data length, should be a VERY short wait
              while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      
              // read a packet from FIFO
              mpu.getFIFOBytes(fifoBuffer, packetSize);
              
              // track FIFO count here in case there is > 1 packet available
              // (this lets us immediately read more without waiting for an interrupt)
              fifoCount -= packetSize;
      
      // display Euler angles in degrees
                  mpu.dmpGetQuaternion(&q, fifoBuffer);
                  mpu.dmpGetEuler(euler, &q);
                  new_value=euler[0]*180/M_PI;
            
          }
        }
      
        stop_car();
        delay(1000);
  }
   
  //Waiting 1 second then
  //Start to input forward distance

for(int ss=0; ss<2; ss++){
    digitalWrite(trig,LOW);
    delayMicroseconds(2);
    digitalWrite(trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig,LOW); 

    duration=pulseIn(echo,HIGH);
    distance=duration/58;
}

    Serial.print("DISTANCE= ");
    Serial.println(distance);
    
    digitalWrite(LED2,HIGH);
    Serial.println("Enter forward distance to cover: ");
    while(Serial.available()==0){}
    string_move_distance=Serial.readString();
    move_distance=string_move_distance.toInt();
    digitalWrite(LED2,LOW);
    diff=distance-move_distance;

    while(distance>diff){
      
        analogWrite(enableB2,170);   //Left Motor Speed
        analogWrite(enableB,170);   //Left Motor Speed
        analogWrite(enableA,170);  //Right Motor Speed
        analogWrite(enableA2,170);  //Right Motor Speed
    
        digitalWrite(input1,HIGH);
        digitalWrite(input2,LOW);
        digitalWrite(input3,HIGH);
        digitalWrite(input4,LOW); 
        
        digitalWrite(trig,LOW);
        delayMicroseconds(2);
        digitalWrite(trig,HIGH);
        delayMicroseconds(10);
        digitalWrite(trig,LOW); 
    
        duration=pulseIn(echo,HIGH);
        distance=duration/58;
        Serial.print("DISTANCE YA BEEH = ");
        Serial.println(distance);      
      }
      
      stop_car();
      delay(1000);
      
      //Wait 1 second then 
      //Enter the value of backward distance
 for(int zz=0; zz<2; zz++){     
    digitalWrite(trig,LOW);
    delayMicroseconds(2);
    digitalWrite(trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig,LOW); 

    duration=pulseIn(echo,HIGH);
    distance=duration/58;
 }

    Serial.print("DISTANCE = ");
    Serial.println(distance);
     
    digitalWrite(LED2,HIGH);
    Serial.println("Enter backward distance to cover: ");
    while(Serial.available()==0){}
    string_move_distance=Serial.readString();
    move_distance=string_move_distance.toInt();
    digitalWrite(LED2,LOW);
    diff=distance+move_distance;

    while(distance<diff){
      
        analogWrite(enableA,160);
        analogWrite(enableB,160);
        analogWrite(enableB2,160);
        analogWrite(enableA2,160);
         
        digitalWrite(input1,LOW);
        digitalWrite(input2,HIGH);
        digitalWrite(input3,LOW);
        digitalWrite(input4,HIGH); 
        
        digitalWrite(trig,LOW);
        delayMicroseconds(2);
        digitalWrite(trig,HIGH);
        delayMicroseconds(10);
        digitalWrite(trig,LOW); 
    
        duration=pulseIn(echo,HIGH);
        distance=duration/58;
        Serial.print("DISTANCE YA BEEH = ");
        Serial.println(distance);      
      }    

    stop_car();
    delay(2000);      
    
    }

  while(data=='D'){
      //FIRST TURN
      straight(150); 
      mpu_calibration();     
      right_angle(87);
      // SECOND TURN
      straight(150); 
      mpu_calibration();     
      right_angle(80);
      // THIRD TURN
      straight(150); 
      mpu_calibration();     
      right_angle(70);
      // FOURTH TURN
      straight(150); 

      digitalWrite(LED,HIGH);
      stop_car();
      delay(2000);
      digitalWrite(LED,LOW);
      break;
  
  }

  while(data=='V'){
      digitalWrite(LED,HIGH);
      digitalWrite(LED2,HIGH);
      while(Serial.available()==0){}
      readvoice=Serial.readString();
      Serial.println(readvoice);

      if(readvoice=="forward"){
          fwd_car();
        }
        
      if(readvoice=="stop"){
          stop_car();
        }
        
      if(readvoice=="back"){
          bwd_car();
        }
        
     if(readvoice=="easy driving"){
        digitalWrite(LED,LOW);
        digitalWrite(LED2,LOW);
        break;
      }

    if(readvoice=="rotate"){
      digitalWrite(LED,LOW);
      while(Serial.available()==0){}
      readangle=Serial.parseInt();
      mpu_calibration();     
      right_angle(readangle);  
      }
   
  }

}
