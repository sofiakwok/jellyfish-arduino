int fsrPin = 15;     // the FSR/voltage divider is connected to 15
int fsrReading;     // the analog reading from the voltage divider
int greenPin =  14;

unsigned long endTime;
unsigned long sensorTime;
unsigned long duration;
bool hasBeenPressed;

void setup(void) {
  Serial.begin(9600); 
  pinMode(greenPin, OUTPUT);  
}

void loop(void) {
  fsrReading = analogRead(fsrPin);
  Serial.print("analog reading = ");
  Serial.println(fsrReading);     // the raw analog reading
  
  if (!hasBeenPressed && fsrReading > 300){ // sensor hasn't been pressed but is now
    sensorTime = millis();
    hasBeenPressed = true;
  }
  if (hasBeenPressed && fsrReading < 300){ //has been pressed but isn't any longer
    endTime = millis();
    duration = endTime - sensorTime;  // tells you how long the sensor detected pressure for
    Serial.print("duration of pressure on the sensor: ");
    Serial.println(duration);
    digitalWrite(greenPin, HIGH); //LED lights up after the sensor has been pressed and pressure is no longer on it
    delay(50);
    digitalWrite(greenPin, LOW); //getting the LED to turn off
    hasBeenPressed = false;
  }
  delay(50);
}
