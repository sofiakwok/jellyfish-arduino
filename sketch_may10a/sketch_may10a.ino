const int stppin = 3;
const int stppin = 2;
const int ms1 = 4;
const int ms2  =5;
const int ms3 = 6;

int fast[] = {0, 0, 0};
int medium[] = {0, 1, 0};
int slow[] = {1, 1, 1};

void setup() {
  pinMode(dirpin, OUTPUT);
  pinMode(stppin, OUTPUT);
  pinMode(ms1, OUTPUT);
  pinMode(ms2, OUTPUT);
  pinMode(ms3, OUTPUT);
  setspeed(slow);
}

void loop() {
  // 5 rotations at low speed, then 5 rotations in the other direction at high speed
  digitalWrite(dirpin, LOW);
  movestep(500);
  digitalWrite(dirpin, HIGH);
  setspeed(fast);
  movestep(500)
}

void movestep(int steps){
  for(int i = 0; i < steps; i++){
    digitalWrite(stppin, HIGH);
    delay(10);
    digitalWrite(stppin, LOW);
    delay(10);
  }
}

void setspeed(int ms[3]){
  digitalWrite(ms1, ms[0]);
  digitalWrite(ms2, ms[1]);
  digitalWrite(ms3, ms[2]);
}
