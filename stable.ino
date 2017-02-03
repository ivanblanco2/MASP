//--- Servo Connection
// BROWN - gnd
// red - 5v
// yellow - d11 Servo 1

//--- MPU Connection
// VCC - 5v
// GND - GND
// SCL - A5 (w/ 10k PuR)
// SDA - A4 (w/ 10k PuR)
// INT - D2 (not used)

#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
//=========================

/* Some functions */
void move_s(int dir, int servo/*6-11*/, int deg);
void x_rot(int deg);
void y_rot(int deg);

#define LED_PIN 13
bool blinkState = true;
                //orientation 
Servo Servo1;   //  
Servo Servo2;   // 
Servo Servo3;   //
Servo Servo4;   //
Servo Servo5;   //
Servo Servo6;   //


int Servo1Pos = 0;
int Servo2Pos = 0;
int Servo3Pos = 0;
int Servo4Pos = 0;
int Servo5Pos = 0;
int Servo6Pos = 0;


float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;

// define MPU
MPU6050 mpu; 

// MPU control/status vars
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
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// relative ypr[x] usage based on sensor orientation when mounted, e.g. ypr[PITCH]
#define PITCH   1     // defines the position within ypr[x] variable for PITCH; may vary due to sensor orientation when mounted
#define ROLL  2     // defines the position within ypr[x] variable for ROLL; may vary due to sensor orientation when mounted
#define YAW   0     // defines the position within ypr[x] variable for YAW; may vary due to sensor orientation when mounted

// ================================================================
//                       INITIAL SETUP                       
// ================================================================

void setup()
{
  Servo1.attach(6);  // attaches the servo on D11 to the servo object
  Servo2.attach(7);  //
  Servo3.attach(8);
  Servo4.attach(9);
  Servo5.attach(10);
  Servo6.attach(11);
  
  Servo1.write(60);
  Servo2.write(120);
  Servo3.write(60);
  Servo4.write(120);
  Servo5.write(60);
  Servo6.write(120);

  // join I2C bus
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial);   

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP"));
  devStatus = mpu.dmpInitialize();


  // CALIBRATED OFFSETS 
  /*mpu.setXGyroOffset(118);
  mpu.setYGyroOffset(-44);
  mpu.setZGyroOffset(337);
  mpu.setXAccelOffset(-651);
  mpu.setYAccelOffset(670);
  mpu.setZAccelOffset(1895);
  */
  
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(86);
  mpu.setZGyroOffset(-85);
  //mpu.setXAccelOffset(0);
  //mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(1788);
  
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP"));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)"));
    mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    Serial.print(F("DMP Initialization failed code = "));
    Serial.println(devStatus);
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

}



// ================================================================
//                     MAIN PROGRAM LOOP                    
// ================================================================

void loop(void)
{
  processAccelGyro();
} 
// ================================================================
//                     PROCESS ACCEL/GYRO IF AVAILABLE       
// ================================================================

void processAccelGyro()
{

  // Get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }

  if (mpuIntStatus & 0x02)
  {
    // check for correct available data length
    if (fifoCount < packetSize)
      return; //  fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    fifoCount -= packetSize;

    // flush buffer to prevent overflow
    mpu.resetFIFO();

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpuPitch = ypr[PITCH] * 180 / M_PI;
    mpuRoll = ypr[ROLL] * 180 / M_PI;
    mpuYaw  = ypr[YAW] * 180 / M_PI;

    // flush buffer to prevent overflow
    mpu.resetFIFO();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    // flush buffer to prevent overflow
    mpu.resetFIFO();
    
    Servo1.write(mpuRoll + 30);
    Servo6.write(-mpuRoll - 90);

    Servo4.write(mpuRoll + 30);
    Servo5.write(-mpuRoll - 90);
    
    ///y_rot(mpuRoll);
    ///x_rot(mpuPitch);
    
    // flush buffer to prevent overflow
    mpu.resetFIFO();

  } 
} 

//1-DOF (Rotate over x axis- or y-pos rotate
//This will set B (9/10), C (11, 6))- as base- where C is the right of B
// 
enum dir{
up = 1,
down = 0,
};

void y_rot(int deg){
  //Move B - if deg is pos, move B up
  move_s(up, 9, deg);
  move_s(up, 10, deg);

  //Move C - if deg is pos, move C down
  move_s(down, 6, deg);
  move_s(down, 11, deg); 
}

void x_rot(int deg){
  //Move A - if deg is pos, move A down
  move_s(down, 7, deg);
  move_s(down, 8, deg);
  //Move B - if deg is pos, move B up
  move_s(up, 9, deg);
  move_s(up, 10, deg);
  //Move C - if deg is pos, move 
  move_s(up, 6, deg);
  move_s(up, 11, deg);
}


void move_s(int dir, int servo/*6-11*/, int deg){
  //Format: dir? (up) : (down)
  //Meaning: Is direction up? yes- do this : no- do this
  switch(servo){
    case 6:
      Servo1.write(dir? -deg: deg);
      Serial.println("Servo1pos:  "+Servo1Pos); 
      break;
    case 7:
      Servo2.write(dir? deg: -deg);
      Serial.println("Servo2pos:  "+Servo2Pos); 
      break;
    case 8:
      Servo3.write(dir? -deg: deg);
      Serial.println("Servo3pos:  "+Servo3Pos); 
      break;
    case 9:
      Servo4.write(dir? deg: -deg);
      Serial.println("Servo4pos:  "+Servo4Pos); 
      break;
    case 10:
      Servo5.write(dir? -deg: deg);
      Serial.println("Servo5pos:  "+Servo5Pos); 
      break;
    case 11:
      Servo6.write(dir? deg: -deg);
      Serial.println("Servo6pos:  "+Servo6Pos); 
      break;
    default:
      printf("Human error found!!! ALERT! ALERT!\n");
      break;
  } // end of switch-case
} // end of move()




