// Show hardware needed, put it on a table
// In video name the hardware (animation in video)
// Show users how to install these libraries
// 6 Axis Intertial Module
// Roll Pitch Yaw and Acceleration // Explain this
#include "I2Cdev.h" // Just a protocol 
#include "RF24.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
// changed here 
#define INTERRUPT_PIN 2  // MPU6050 sends an interrupt to collect data
#define LED_PIN 13 // Tells if the communication to the MPU is ongoing


byte addresses[][6] = {"00001"};
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
char data = 'X';
unsigned long currentMillis;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
boolean lock1 = false;
boolean lock2 = false;
boolean lock3 = false;
int f1 = 3;
int f2 = 4;
int state1,state2;
int x,y,z;
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
int interval = 1000;           // interval at which to blink (milliseconds)

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
String pMove ="";


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void finger_sensor_setup(){
    pinMode(f1,INPUT);
    pinMode(f2,INPUT);
}

void motion_sensor_setup(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU2000050 connection successful") : F("MPU2000050 connection failed"));
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(51);
    mpu.setYGyroOffset(8);
    mpu.setZGyroOffset(21);
    mpu.setXAccelOffset(1150); 
    mpu.setYAccelOffset(-50); 
    mpu.setZAccelOffset(1020000); 
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU2000050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println();
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void motion_sensor_wait(){
      while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
      }
}

void motion_sensor_update_values(){
      // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        
        x = aaWorld.x;
        y = aaWorld.y;
        z = aaWorld.z;
        Serial.print(x);//Blue
        Serial.print("\t"); 
        Serial.print(y);//Red
        Serial.print("\t");
        Serial.print(z);//Green
        Serial.println("\t");
    }
}

void make_decision_and_send(){
      if (lock1 == true && x<15000 && y<15000 && z<15000) {
             if (currentMillis - previousMillis >= 1500) {
                  previousMillis = currentMillis;
                      lock2 = false;
                      lock1 = false;
            }
        }
       else if(lock2==false){
        if(y >= 20000 && y>x && y>z){
          pMove = "POW 2";
          data = 'C';
          lock1 = true;
          lock2 = true;
          previousMillis = currentMillis;
          interval = 1000;
          radio_send(data);
        }
          else if(z >= 20000 && z>y && z>x){
          pMove = "POW 1";
          data = 'B';
          lock1 = true;
          lock2 = true;
          previousMillis = currentMillis;
          interval = 700;
          radio_send(data);
        }
        else if(x >= 20000 && x>y && x>z){
          pMove = "PUNCH!";
          data = 'A';
          lock1 = true;
          lock2 = true;
          previousMillis = currentMillis;
          interval = 500;
          radio_send(data);
        }
      else if(state1==0 && state2==0){
        pMove = "CHARGE";
          data = 'D';
          lock1 = true;
          lock2 = true;
          previousMillis = currentMillis;
          interval = 500;
          radio_send(data);
      }
   }
}

void setup() {
    finger_sensor_setup();
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    motion_sensor_setup();
    radio_setup();
}



// ================================================================helo ?
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  currentMillis  = millis();
  state1 = digitalRead(f1);
  state2 = digitalRead(f2);
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  motion_sensor_wait();
  motion_sensor_update_values(); 
  make_decision_and_send();
  
  //Serial.println(pMove);
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}
