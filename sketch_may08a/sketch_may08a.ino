#include Servo.h

const int brushlesspin = 2;

void setup() {
  brushless.attach(brushlesspin, 1000, 2000);

  brushless.write(1000);

  delay(2000);

  brushless.write(2000);

  delay(2000);

  brushless.write(1000);
}

void loop() {
  // 0% to 50% throttle in 5 seconds, then ramp it back down to 0% throttle in 5

  //0 to 50 in 5 seconds
  for(int i = 0; i <= 500; i++){
    brushless.write(1000+i);
    delay(10);
  }
  //back to 0 in 5
  for(int i = 0; i <= 500; i++){
    brushless.write(1500-i);
    delay(10);
  }
  
}
