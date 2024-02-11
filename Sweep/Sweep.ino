 /* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
Servo servo2;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  servo2.attach(10);
  Serial.begin(9600);
}

void loop()
{
  if( Serial.available() > 0) read_keypress();
}

/************************************/
void read_keypress()
{
  int ch = Serial.read();
  Serial.print(ch);

  switch( ch ) {
    case '1':  subprogram1();  break;
    default:   break;
  }
}

void subprogram1() {
  while( !Serial.available() ) {
    delay(200);
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      servo2.write(180 - pos);
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      servo2.write(180 - pos);
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  Serial.read();
}
