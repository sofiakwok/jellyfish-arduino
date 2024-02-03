#include <Servo.h>
#include <Math.h>

Servo stroke;  // for controlling stroke motion (symmetrical)
Servo fin1;
Servo fin2;
//beta_1 is left (looking from below), beta_2 is right 
double beta_1;
double beta_2;
//alpha: angle offset of rudders from fins (radians)
//left = negative, right = positive
double alpha_1 = 0;
double alpha_2 = 0;
double beta_1_offset = 5;
double beta_2_offset = 7;
//theta: angle of middle servo, controls fin angles
int theta = 0;

char receivedChar;
bool startLoop = false;
bool newData = false;

void setup() {
  // put your setup code here, to run once:
  stroke.attach(9); // for controlling theta
  fin1.attach(10); // for controlling rudder 1
  fin2.attach(11); // for controlling rudder 2
  stroke.write(0);
  fin1.write(180 - beta_1_offset); //because fin1 is flipped
  fin2.write(0 + beta_2_offset);
  Serial.begin(9600);
}

void loop() {
  recvOneChar();
    // stroke continuously scans from 0 to 180 degrees
    // fins actually go from 0 to 90 because of the 2:1 gear ratio
  while(startLoop){
    Serial.print("\n Loop 1 \n");
    for(theta = 0; theta < 170; theta++)  
    {       
      recvOneChar();
      showNewData();            
      stroke.write(theta);
      update_rudders(theta, alpha_1, alpha_2);
      Serial.print(beta_1);
      Serial.print(" ");
      fin1.write(180 - beta_1 - beta_1_offset);
      fin2.write(beta_2 + beta_2_offset);                
      delay(15);                   
    } 
    Serial.print("\n Loop 2 \n");
    for(theta = 170; theta > 0; theta--)    
    {
      recvOneChar();
      showNewData();                             
      stroke.write(theta);
      update_rudders(theta, alpha_1, alpha_2);
      Serial.print(beta_1);
      Serial.print(" ");
      fin1.write(180 - beta_1 - beta_1_offset);
      fin2.write(beta_2 + beta_2_offset);           
      delay(15);       
    }
  }
}

void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        startLoop = true;
        newData = true;
    }
}

void showNewData() {
  int threshold = 90;
  if (newData == true) {
      Serial.print("This just in ... ");
      Serial.println(receivedChar);
      if (receivedChar == 's'){
        alpha_1 -= 0.175;
      } else if (receivedChar == 'w')
      {
        alpha_1 += 0.175;
      } else if (receivedChar == 'i')
      {
        alpha_2 += 0.175;
      } else if (receivedChar == 'k')
      {
        alpha_2 -= 0.175;
      }
      if (abs(alpha_1) >= threshold) {
        int sign = (alpha_1 > 0) - (alpha_1 < 0);
        alpha_1 = threshold * sign;
      }
      if (abs(alpha_2) >= threshold) {
        int sign = (alpha_2 > 0) - (alpha_2 < 0);
        alpha_2 = threshold * sign;
      }
      newData = false;
  }
}

void update_rudders(double theta, double alpha_1, double alpha_2){
  bool left;
  beta_1 = beta_calc(alpha_1, theta, left=true);
  //Serial.print((String)"(alpha_1: " + alpha_1 + " theta: " + theta + " beta_1: " + beta_1 ")");
  beta_2 = beta_calc(alpha_2, theta, left=false);
}

double beta_calc(double alpha_deg, double theta_deg, bool left){
  //convert degrees to radians and account for theta gear ratio
  double theta = theta_deg * 3.1415/180/2;
  double alpha = alpha_deg * 3.1415/180;
  //Serial.print((String)"(theta: " + theta + " alpha: " + alpha + ")");
  //all measurements in inches and taken from Solidworks
  double d = 3.052717; // length of outer servo attachment to steer rudders 
  double l = 0.483608; // length of rudders
  double fin_len = 2.15178;
  
  // offset of servo from fin rotational axis (m_1 = x, m_2 = y)
  // assumes fin rotational axis is at (0, 0)
  double m_1 = 0;
  double m_2 = 0;
  //given a desired rudder angle (alpha) calculate beta (fin servo angle)
  double x_1 = 0;
  double y_1 = 0;
  double x_2 = 0;
  double y_2 = 0;

  double a = 0;
  double b = 0;
  double c = 0;
  double top = 0;
  double bottom = 0;

  if (left){
    m_1 = -0.153543;
    m_2 = 0.405512;
    x_1 = -fin_len*sin(theta);
    y_1 = -fin_len*cos(theta);
    x_2 = (l + fin_len)*sin(theta);//l*sin(alpha - theta) + x_1;
    y_2 = -(l + fin_len)*cos(theta);//-l*cos(alpha - theta) + y_1;
    a = pow(4*l*m_1 - 4*l*x_2, 2);
    b = pow(d, 2) - pow(l, 2) + 2*l*m_2 - 2*l*y_2 - pow(m_1, 2) + 2*m_1*x_2 - pow(m_2, 2) + 2*m_2*y_2 - pow(x_2, 2) - pow(y_2, 2);
    c = pow(d, 2) - pow(l, 2) - 2*l*m_2 + 2*l*y_2 - pow(m_1, 2) + 2*m_1*x_2 - pow(m_2, 2) + 2*m_2*y_2 - pow(x_2, 2) - pow(y_2, 2);
    top = sqrt(a - 4*b*c) - 2*l*m_1 + 2*l*x_2; 
    bottom = pow(d, 2) - pow(l, 2) + 2*l*m_2 - 2*l*y_2 - pow(m_1, 2) + 2*m_1*x_2 - pow(m_2, 2) + 2*m_2*y_2 - pow(x_2, 2) - pow(y_2, 2);
  } else {
    m_1 = 0.074803;
    m_2 = 0.405512;
    x_1 = fin_len*sin(theta);
    y_1 = -fin_len*cos(theta);
    x_2 = l*sin(alpha + theta) + x_1;
    y_2 = -l*cos(alpha + theta) + y_1;
    a = pow(4*l*x_2 - 4*l*m_1, 2);
    b = pow(d, 2) - pow(l, 2) + 2*l*m_2 - 2*l*y_2 - pow(m_1, 2) + 2*m_1*x_2 - pow(m_2, 2) + 2*m_2*y_2 - pow(x_2, 2) - pow(y_2, 2);
    c = pow(d, 2) - pow(l, 2) - 2*l*m_2 + 2*l*y_2 - pow(m_1, 2) + 2*m_1*x_2 - pow(m_2, 2) + 2*m_2*y_2 - pow(x_2, 2) - pow(y_2, 2);
    top = sqrt(a - 4*b*c) + 2*l*m_1 - 2*l*x_2; 
    bottom = pow(d, 2) - pow(l, 2) + 2*l*m_2 - 2*l*y_2 - pow(m_1, 2) + 2*m_1*x_2 - pow(m_2, 2) + 2*m_2*y_2 - pow(x_2, 2) - pow(y_2, 2);
  }
  double beta = 2*(atan(0.5*top/bottom));
  //convert back to degrees 
  double beta_deg = 180/3.1415*beta;
  return beta_deg;
}
