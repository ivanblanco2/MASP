
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

enum direction{
up = 1,
down = 0,
};

/* Some functions declared*/
void move_s(direction dir, int servo/*6-11*/, int deg);
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

//degree position of the servos
int deg;
int deg1 = 90;
int deg2 = 90;
int deg3 = 90;
int deg4 = 90;  
int deg5 = 90;
int deg6 = 90;
int degPitch;
int degRoll;
int degYaw;
int setupFlag = 1;
int aaPrev = 0;
int vel = 0;

int mpuPitch = 0;
int mpuRoll = 0;
int mpuYaw = 0;

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
  
  Servo1.write(deg1);   //1-No.6
  Servo2.write(deg2);   //2-No.7
  Servo3.write(deg3);   //3-No.8
  Servo4.write(deg4);   //4-No.9
  Servo5.write(deg5);   //5-No.10
  Servo6.write(deg6);   //6-No.11



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
  
  //mpu.setXGyroOffset(400);
  //mpu.setYGyroOffset(-44);
  //mpu.setZGyroOffset(337);
  //mpu.setXAccelOffset(-651);
  //mpu.setYAccelOffset(670);
  //mpu.setZAccelOffset(1788);
  
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
    aaPrev = aaWorld.z;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


    mpuPitch = ((ypr[PITCH]) * 180 / M_PI)/1 - 11;
    mpuRoll = ((ypr[ROLL]) * 180 / M_PI)/1 +1;
    mpuYaw  = ((ypr[YAW]) * 180 / M_PI)/1-30;

    // flush buffer to prevent overflow
    mpu.resetFIFO();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    // flush buffer to prevent overflow
    mpu.resetFIFO();
    
    
    /*
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    This is where we can test out movements
    Setup:
      deg1 = 60;   90  (up is plus)
      deg2 = 120;  90
      deg3 = 60;   90
      deg4 = 120;  90
      deg5 = 60;   90
      deg6 = 120;  90
    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    */

    if (setupFlag == 1) {
      x_rot(((3.0) * (degPitch = mpuPitch))/1);   //A
      y_rot(((3.0) * (degRoll = mpuRoll))/1);    //B-C
      degRoll = mpuRoll;
      degPitch = mpuPitch;
      setupFlag = 0;
    }
    
    x_rot(((3.0) * (mpuPitch - degPitch))/1);
    y_rot(((3.0) * (mpuRoll - degRoll))/1);
    //z_a(((10.0) * (aaWorld.z/512-1))/1);
    degRoll = mpuRoll;
    degPitch = mpuPitch;

    //vel = vel + aaWorld.z/2058;
    //Serial.print("Roll :");
    //Serial.print(mpuRoll);
    //Serial.print("   Pitch :   ");
    //Serial.println(mpuPitch);
    //Serial.print("   Yaw :   ");
    //Serial.println(mpuYaw);
    //Serial.print("aaWorld: ");
    //Serial.print( aaWorld.z/2058  );
    //Serial.print("  velo: ");
    //Serial.println(vel);
    //Serial.print("  ");
    //Serial.println(degRoll);
    //Serial.println(M_PI);
    Serial.print("accel: ");
    Serial.println(10.0 * aaWorld.z/2058);
  
  delay (20);
    
  //Check Servo positions  
  /*Serial.print("d6: ");
  Serial.print(deg1); 
  Serial.print("d7: ");
  Serial.print(deg2);   
  Serial.print("d8: ");
  Serial.print(deg3);   
  Serial.print("d9: ");
  Serial.print(deg4);   
  Serial.print("d10: ");
  Serial.print(deg5);   
  Serial.print("d11: ");
  Serial.println(deg6);*/
  
    // flush buffer to prevent overflow
    mpu.resetFIFO();
  } 
} 

//1-DOF (Rotate over x axis- or y-pos rotate
//This will set B (9/10), C (11, 6))- as base- where C is the right of B
// 

//For now, we'll focus on the most important 3 DOF
// X Movements
void x_rot(int deg){
  //Pitching - rotating towards positive y region over x-axis [towards A]
  //Move A - if deg is pos, move A down
  move_s(down, 7, deg);
  move_s(down, 8, deg);
  //Move B - if deg is pos, move B up
  move_s(up, 9, deg);
  move_s(up, 10, deg);
  //Move C - if deg is pos, move C up
  move_s(up, 6, deg);
  move_s(up, 11, deg);
}
// END OF X Movements

// Y Movements
void y_rot(int deg){
  //Rolling - rotating towards positive x region over y-axis [towards C]
  //Move B - if deg is pos, move B up
  move_s(up, 9, deg);
  move_s(up, 10, deg);

  //Move C - if deg is pos, move C down
  move_s(down, 6, deg);
  move_s(down, 11, deg); 
}
// END OF Y Movements

// Z Movements
//displacement over z-axis
void z_a(int accel){
  //Top ceiling
  if ((deg1 + accel) > 180) {accel = 180 - deg1;}
  if ((deg2 + accel) > 180) {accel = 180 - deg2;}
  if ((deg3 + accel) > 180) {accel = 180 - deg3;}
  if ((deg4 + accel) > 180) {accel = 180 - deg4;}
  if ((deg5 + accel) > 180) {accel = 180 - deg5;}
  if ((deg6 + accel) > 180) {accel = 180 - deg6;}
  
  //Bottom flooring
  if ((deg1 + accel) < 0) {accel = 0 - deg1;}
  if ((deg2 + accel) < 0) {accel = 0 - deg2;}
  if ((deg3 + accel) < 0) {accel = 0 - deg3;}
  if ((deg4 + accel) < 0) {accel = 0 - deg4;}
  if ((deg5 + accel) < 0) {accel = 0 - deg5;}
  if ((deg6 + accel) < 0) {accel = 0 - deg6;}
  
  Servo1.write(deg1 + accel);
  Servo2.write(deg2 + accel);
  Servo3.write(deg3 + accel);
  Servo4.write(deg4 + accel);
  Servo5.write(deg5 + accel);
  Servo6.write(deg6 + accel);
}
// END OF Z Movements

int guard(int deg){
  if (deg > 180) {deg = 180;}
  if (deg < 0) {deg = 0;)
  return deg;
}


void move_s(direction dir, int servo/*6-11*/, int deg){
  //Format: dir? (up) : (down)
  //Meaning: Is direction up? yes- do this : no- do this
  switch(servo){
    case 6: //left
      if (dir == up) deg1 = guard(deg1 + deg);
      else if (dir == down) deg1 = guard(deg1 - deg);
      Servo1.write(deg1);
      //Serial.print(" Servo1 :    "); 
      //Serial.print(deg1);
      //Serial.println("Servo1pos:  "+deg1); 
      break;
    case 7: //right
      if (dir == up) deg2 = guard(deg2 - deg);
      else if (dir == down) deg2 = guard(deg2 + deg);
      Servo2.write(deg2);
      //Serial.print(" Servo2 :    "); 
      //Serial.print(deg2);
      //Serial.println("Servo2pos:  "+deg2); 
      break;
    case 8: //left
      if (dir == up) deg3 = guard(deg3 + deg);
      else if (dir == down) deg3 = guard(deg3 - deg);
      Servo3.write(deg3);
      //Serial.print(" Servo3 :    "); 
      //Serial.print(deg3);
      //Serial.println("Servo3pos:  "+deg3); 
      break;
    case 9: //right
      if (dir == up) deg4 = guard(deg4 - deg);
      else if (dir == down) deg4 = guard(deg4 + deg);
      Servo4.write(deg4);
      //Serial.print(" Servo4 :    "); 
      //Serial.print(deg4);
     // Serial.println("Servo4pos:  "+deg4); 
      break;
    case 10: //left
      if (dir == up) deg5 = guard(deg5 + deg);
      else if (dir == down) deg5 = guard(deg5 - deg);
      Servo5.write(deg5);
      //Serial.print(" Servo5 :    "); 
      //Serial.print(deg5);
      //Serial.println("Servo5pos:  "+deg5); 
      break;
    case 11: //right
      if (dir == up) deg6 = guard(deg6 - deg);
      else if (dir == down) deg6 = guard(deg6 + deg);
      Servo6.write(deg6);
      //Serial.print(" Servo6 :    "); 
      //Serial.print(deg6);
      //Serial.println("Servo6pos:  "+deg6); 
      break;
    default:
      printf("Unknwon Servo: Bad-coder found!!! ALERT! ALERT!\n");
      break;
  } // end of switch-case
} // end of move()

