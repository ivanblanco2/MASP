
 /* Read Jostick
  * ------------
  *
  * Reads two analog pins that are supposed to be
  * connected to a jostick made of two potentiometers
  *
  * We send three bytes back to the comp: one header and two
  * with data as signed bytes, this will take the form:
  *    Jxy\r\n
  *
  * x and y are integers and sent in ASCII 
  * 
  * http://www.0j0.org | http://arduino.berlios.de
  * copyleft 2005 DojoDave for DojoCorp
  */
  
// Controlling movements of two servos using a Joystick
// Joystick's x- and y- axes are connected to A0 and A1 analog pins of Arduino.
// Servos are connectd to PWM Pins 9 and 10.
// By Explore Labs
#include <Servo.h>
#include "Wire.h"

enum direction{
up = 1,
down = 0,
};

Servo s1, s2, s3, s4, s5, s6;
int servoPos = 0;
int deg1 = 90;
int deg2 = 90;
int deg3 = 90;
int deg4 = 90;
int deg5 = 90;
int deg6 = 90;
int prev_xmap, prev_ymap;
int first = 1;
int joyX = A0; // analog pin used to connect the X - axis of Joystick
int joyY = A1; // analog pin used to connect the Y - axis of Joystick
int x, y; // variables to read the values from the analog pins 
int xmap, ymap;
int offsetx;
int offsety;
void move_s(direction dir, int servo/*6-11*/, int deg);
int guard(int deg);
void x_rot(int deg);
void y_rot(int deg);

void setup() {
  // put your setup code here, to run once:
  s1.attach(6);
  s2.attach(7);
  s3.attach(8);
  s4.attach(9);
  s5.attach(10);
  s6.attach(11);
  
  s1.write(90);
  s2.write(90);
  
  s3.write(90);
  s4.write(90);
  s5.write(90);
  s6.write(90);
  Serial.begin(115200);
  
  delay(1000);
  offsetx = abs(((analogRead(joyX) - 512 ) * 2));
  offsety = abs(((analogRead(joyY) - 512 ) * 2));
}



void loop(){ 
  x = analogRead(joyX);    // reads the value of the Joystick's X - axis (value between 0 and 1023) 
  y = analogRead(joyY);    // reads the value of the Joystick's Y - axis (value between 0 and 1023) 
  xmap = map(x, offsetx, 1023, 0, 180); // scale it to use with the servo b/w 900 usec to 2100 usec
  ymap = map(y, offsety, 1023, 0, 180); // scale it to use with the servo b/w 900 usec to 2100 usec

  if (first == 1){
    prev_xmap = xmap;
    prev_ymap = ymap;
    first = 0;
  }
  
  x_rot(xmap - prev_xmap);
  y_rot(ymap - prev_ymap);

  prev_xmap = xmap;
  prev_ymap = ymap;

  Serial.print("xmapping- ");
  Serial.print( xmap );
  Serial.print("ymapping- ");
  Serial.println( ymap);

  delay (200);
  Serial.print("d6: ");
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
  Serial.println(deg6); 

}



//For now, we'll focus on the most important 3 DOF
// X Movements
void x_rot(int deg){
  //Pitching - rotating towards positive y region over x-axis
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
  //Rolling - rotating towards positive x region over y-axis
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
void z_up(int deg){
  move_s(up, 6, deg);
  move_s(up, 7, deg);
  move_s(up, 8, deg);  
  move_s(up, 9, deg);
  move_s(up, 10, deg);  
  move_s(up, 11, deg);
}
// END OF Z Movements

int guard(int deg){
  if ((deg > 180)? 180 : deg);
  else if ((deg < 0)? 0 : deg);
  return deg;
}


void move_s(direction dir, int servo/*6-11*/, int deg){
  //Format: dir? (up) : (down)
  //Meaning: Is direction up? yes- do this : no- do this
  switch(servo){
    case 6: //left
      if (dir == up) deg1 = guard(deg1 + deg);
      else if (dir == down) deg1 = guard(deg1 - deg);
      s1.write(deg1);
      //Serial.println("Servo1pos:  "+deg1); 
      break;
    case 7: //right
      if (dir == up) deg2 = guard(deg2 - deg);
      else if (dir == down) deg2 = guard(deg2 + deg);
      s2.write(deg2);
      //Serial.println("Servo2pos:  "+deg2); 
      break;
    case 8: //left
      if (dir == up) deg3 = guard(deg3 + deg);
      else if (dir == down) deg3 = guard(deg3 - deg);
      s3.write(deg3);
      //Serial.println("Servo3pos:  "+deg3); 
      break;
    case 9: //right
      if (dir == up) deg4 = guard(deg4 - deg);
      else if (dir == down) deg4 = guard(deg4 + deg);
      s4.write(deg4);
     // Serial.println("Servo4pos:  "+deg4); 
      break;
    case 10: //left
      if (dir == up) deg5 = guard(deg5 + deg);
      else if (dir == down) deg5 = guard(deg5 - deg);
      s5.write(deg5);
      //Serial.println("Servo5pos:  "+deg5); 
      break;
    case 11: //right
      if (dir == up) deg6 = guard(deg6 - deg);
      else if (dir == down) deg6 = guard(deg6 + deg);
      s6.write(deg6);
      //Serial.println("Servo6pos:  "+deg6); 
      break;
    default:
      printf("Unknwon Servo: Bad-coder found!!! ALERT! ALERT!\n");
      break;
  } // end of switch-case
} // end of move()
