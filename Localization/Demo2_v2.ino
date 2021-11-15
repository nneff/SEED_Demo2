#include <math.h>
#include <Wire.h>
#define SLAVE_ADDRESS 0x04
//Motor
#define FlagIndicator 12
#define Enable 4
#define RMotorDir 7
#define RMotorSpeed 9
#define LMotorDir 8
#define LMotorSpeed 10
//Encoder
#define pinRA 2 //Yellow
#define pinRB 5 //White
//Blue -> GND
//Green -> VDD
#define pinLA 3 //Yellow
#define pinLB 6 //White
#define VCC 13
long stateRB, stateRA, prevRB, stateLB, stateLA, prevLB;
int countRight, countLeft = 0;
bool detected = false;
bool driving = false;
byte data[10];
double angle, distance;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(Enable, OUTPUT);
  digitalWrite(Enable, HIGH);
  pinMode(FlagIndicator, INPUT);

  pinMode(RMotorDir, OUTPUT);
  pinMode(RMotorSpeed, OUTPUT);
  pinMode(LMotorDir, OUTPUT);
  pinMode(LMotorSpeed, OUTPUT);

  pinMode(VCC, OUTPUT);
  digitalWrite(VCC, HIGH);

  attachInterrupt(digitalPinToInterrupt(pinRA), readEncoderRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinLA), readEncoderLeft, CHANGE);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
}

void loop() {

  if (driving == true) {
    if (abs(distance) < 18.0) {
      Turn(-angle*PI/360.0);
    }
    else {
      Turn(-angle*PI/180.0);
    }
    //delay(500);
    Drive(distance/12.0);
    //delay(500);
    driving = false;
  }
  if (detected == false) {
    Turn(15*PI/180.0);
    delay(500);
  }
  //Serial.println("test");


}




void Drive(double Distance){
  countRight = 0;
  countLeft = 0;
  double DesiredAngleRight = 4.0 * Distance;
  double DesiredAngleLeft = -4.0 * Distance;
  double DiffRight, DiffLeft, SpeedVRight, SpeedVLeft, PWMR, PWML, CurrentAngleRight, CurrentAngleLeft, DiffRL;
  double kp = 4;
  double kpd = 20;
  DiffRight = DesiredAngleRight;
  DiffLeft = DesiredAngleLeft;
  //Serial.println(DiffRight);
  while (DiffRight > 0.1 or DiffLeft > 0.1) {
    CurrentAngleRight =  (countRight / 3200.0) * (360.0) * (PI / 180.0); //count right gets more negative as we drive forwards
    CurrentAngleLeft =  (countLeft / 3200.0) * (360.0) * (PI / 180.0); //count left gets more pos as we drive forwards
    DiffRight = DesiredAngleRight + CurrentAngleRight;
    //Serial.println(DiffRight);
    DiffLeft = DesiredAngleLeft + CurrentAngleLeft;
    DiffRL = CurrentAngleRight + CurrentAngleLeft;
    SpeedVRight = -DiffRight * kp;
    Serial.println(SpeedVRight);
    SpeedVLeft = DiffLeft * kp;
    
    if (SpeedVRight > 4) {
      SpeedVRight = 4;
    }
    else if (SpeedVRight < -4) {
      SpeedVRight = -4;
    }
    if (SpeedVLeft > 4) {
      SpeedVLeft = 4;
    }
    else if (SpeedVLeft < -4) {
      SpeedVLeft = -4;
    }
    SpeedVRight = SpeedVRight - DiffRL * kpd;
    SpeedVLeft = SpeedVLeft + DiffRL * kpd;

    if (SpeedVRight > 0) {
      digitalWrite(RMotorDir, LOW);
    }
    else {
      digitalWrite(RMotorDir, HIGH);
    }
    if (SpeedVLeft > 0) {
      digitalWrite(LMotorDir, LOW);
    }
    else {
      digitalWrite(LMotorDir, HIGH);
    }

    SpeedVRight = abs(SpeedVRight);
    SpeedVLeft = abs(SpeedVLeft);
    PWMR = (SpeedVRight * 255.0) / 7.2;
    PWML = (SpeedVLeft * 255.0) / 7.2;

    analogWrite(RMotorSpeed, PWMR);
    analogWrite(LMotorSpeed, PWML);
  }
  analogWrite(RMotorSpeed, 0);
  analogWrite(LMotorSpeed, 0);
}


void Turn(double Angle){
  countRight = 0;
  countLeft = 0;
  double phi = 11.0/3.0;
  double TurnAngle = Angle * phi;
  double DesiredAngleRight = TurnAngle / 2.0;
  double DesiredAngleLeft = TurnAngle / 2.0;
  double DiffRight, DiffLeft, SpeedVRight, SpeedVLeft, PWMR, PWML, CurrentAngleRight, CurrentAngleLeft, DiffRL;
  double kp = 10;
  double kpd = 5;
  DiffRight = DesiredAngleRight;
  DiffLeft = DesiredAngleLeft;
  Serial.println(DiffRight);
  while (abs(DiffRight) > 0.1 or abs(DiffLeft) > 0.1) {
    Serial.println("loop");
    CurrentAngleRight =  (countRight / 3200.0) * (360.0) * (PI / 180.0); //count right gets more negative as we drive forwards
    CurrentAngleLeft =  (countLeft / 3200.0) * (360.0) * (PI / 180.0); //count left gets more pos as we drive forwards
    DiffRight = DesiredAngleRight + CurrentAngleRight;
    DiffLeft = DesiredAngleLeft + CurrentAngleLeft;
    DiffRL = -CurrentAngleRight + CurrentAngleLeft;
    Serial.println(DiffRL);
    SpeedVRight = -DiffRight * kp;
    SpeedVLeft = DiffLeft * kp;
    
    if (SpeedVRight > 2) {
      SpeedVRight = 2;
    }
    else if (SpeedVRight < -2) {
      SpeedVRight = -2;
    }
    if (SpeedVLeft > 2.1) {
      SpeedVLeft = 2.1;
    }
    else if (SpeedVLeft < -2.1) {
      SpeedVLeft = -2.1;
    }
    SpeedVRight = SpeedVRight + DiffRL * kpd;
    SpeedVLeft = SpeedVLeft + DiffRL * kpd;
    if (SpeedVRight > 0) {
      digitalWrite(RMotorDir, LOW);
    }
    else {
      digitalWrite(RMotorDir, HIGH);
    }
    if (SpeedVLeft > 0) {
      digitalWrite(LMotorDir, LOW);
    }
    else {
      digitalWrite(LMotorDir, HIGH);
    }

    SpeedVRight = abs(SpeedVRight);
    SpeedVLeft = abs(SpeedVLeft);
    PWMR = (SpeedVRight * 255.0) / 7.2;
    PWML = (SpeedVLeft * 255.0) / 7.2;

    analogWrite(RMotorSpeed, PWMR);
    analogWrite(LMotorSpeed, PWML);
  }
  analogWrite(RMotorSpeed, 0);
  analogWrite(LMotorSpeed, 0);
  Serial.println("out of loop");
}








//Receiving Data
//void receiveData() {
//  while (Wire.available()) {
//    byte fromrpi;
//    fromrpi = Wire.read();
//    Serial.println(fromrpi);
//  }
//}
void receiveData() {
  Serial.println("receive");
  int i = 0;
  while (Wire.available()) {
    data[i] = Wire.read();
    if (i == 1) {
      angle = (data[i]*(180.0/255.0))-90.0;
    }
    if (i == 2) {
      distance = data[i]*24.0/255.0;
    }
    Serial.println(angle);
    i++;
  }
  if (detected == true) {
    driving = true;
  }
  detected = true;
  
}

//Encoder ISR (right)
void readEncoderRight() {
  stateRB = digitalRead(pinRB);
  stateRA = digitalRead(pinRA);

  if (prevRB != stateRB) {
    if (stateRA != stateRB) {
      countRight = countRight - 2;
    } else {
      countRight = countRight + 2;
    }
  }
  prevRB = stateRB;
}

//Encoder ISR (left)
void readEncoderLeft() {
  stateLB = digitalRead(pinLB);
  stateLA = digitalRead(pinLA);

  if (prevLB != stateLB) {
    if (stateLA != stateLB) {
      countLeft = countLeft - 2;
    } else {
      countLeft = countLeft + 2;
    }
  }
  prevLB = stateLB;
}
