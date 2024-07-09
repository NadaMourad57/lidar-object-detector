#include <TFLI2C.h>

#include <AccelStepper.h>
#include "avr/wdt.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include "avr/wdt.h"

TFLI2C Lidar;

int16_t Lidar_dist;
int16_t LidarAddr = TFL_DEF_ADR;



#define DIR_PIN1 3
#define STEP_PIN1 2
#define ENABLE_PIN1 4
#define DIR_PIN2 6
#define STEP_PIN2 5
#define ENABLE_PIN2 7
#define limitleft 8
#define limitright 9
#define BUZZER 12
 #define proximityup A0
 #define proximitydown A1

#define STEPS_PER_REV 3200


int z = 0;
float angle = 0;
int direction = 1;
boolean detect=true;
boolean caliberated=false;

AccelStepper stepper1(1, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(1, STEP_PIN2, DIR_PIN2);

SoftwareSerial Bluetooth(10, 11);
char Data;
void sendData(String transmitData) {
  Bluetooth.println(transmitData);
}


void setup()

{

  Bluetooth.begin(9600);
  stepper1.setMaxSpeed(1600);
  stepper1.setAcceleration(3000);

pinMode(proximityup,INPUT);
pinMode(proximitydown,INPUT);


  pinMode(ENABLE_PIN1, OUTPUT);


  digitalWrite(ENABLE_PIN1, HIGH);


  stepper1.setCurrentPosition(0);
  stepper2.setMaxSpeed(1600);


  stepper2.setAcceleration(3000);


  pinMode(ENABLE_PIN2, OUTPUT);
  digitalWrite(ENABLE_PIN2, HIGH);
  stepper2.setCurrentPosition(0);


  pinMode(limitleft, INPUT_PULLUP);
  pinMode(limitright, INPUT_PULLUP);


  Wire.begin(); 
  Serial.begin(115200);
//  Calibrate();
}


void loop() {
  if(!caliberated){
    Calibrate();
    delay(1000);
  }
  checkserial();
  moveCCW();
  delay(1000);
  moveCW();
  delay(1000);
  // if (z == 32) {
  //   moveToGround();
  // }
  moveVertical(7936);
}


void scan() {

  if (Lidar.getData(Lidar_dist, LidarAddr)) {
    angle += 0.1125;
    if (Lidar_dist < 15 && detect) {
      buzz();
      detect=false; 
    Bluetooth.print(Lidar_dist);
    Bluetooth.print(",");
    Bluetooth.print(angle);
    Bluetooth.print(",");
    Bluetooth.println(z);
    }
   
  }
}


void Calibrate() {
  stepper1.move(3200);
  stepper1.setSpeed(1600);
  while (digitalRead(limitright) != LOW && stepper1.distanceToGo()!=0) {
    stepper1.runSpeedToPosition();
  }
  angle = 0;
  caliberated=true;
}

void moveCW() {

  stepper1.move(3200);
  stepper1.setSpeed(1600);

  while (digitalRead(limitright) != LOW && caliberated) {
    scan();
    checkserial();
    stepper1.runSpeedToPosition();
  }
}


void moveCCW() {
  stepper1.move(-3200);
  stepper1.setSpeed(1600);
  while ( digitalRead(limitleft) != LOW && caliberated) {
    checkserial();
    stepper1.runSpeedToPosition();
  }
  angle = 0;
}



void moveVertical(int steps) {
  stepper2.move(steps*direction);
  stepper2.setSpeed(1600);
  // Serial.println(digitalRead(proximity));
  while (stepper2.distanceToGo() != 0&& digitalRead(proximityup)!=HIGH&&digitalRead(proximitydown)!=HIGH ) {
    stepper2.runSpeedToPosition();
    checkserial();
  }
  z += (2*direction);

  if(digitalRead(proximityup)==HIGH||digitalRead(proximitydown)==HIGH){
    Serial.println("proximity");
    direction=-1*direction;
    moveStepVertical(steps);
  }
  detect=true;

}


void moveStepVertical(int steps ){
    stepper2.move(steps*direction);
  stepper2.setSpeed(1600);
   while (stepper2.distanceToGo() != 0 ) {
    stepper2.runSpeedToPosition();
    checkserial();
  }
  z += (2*direction);

}


void moveToGround() {
  direction = -1;
  moveVertical(111104);
  direction=1;
}

void checkserial(){
     if(Bluetooth.available()){
        Data=Bluetooth.read();


        if(Data==('1')){
        digitalWrite(ENABLE_PIN1, LOW);
        digitalWrite(ENABLE_PIN2, LOW);
        sendData("emergency stop");
        }
        if(Data==('2')){
           direction = 1;
            sendData("going up");
        }
        if(Data==('3')){
           direction = -1;
            sendData("going down");
        }
        if(Data==('4')){
             digitalWrite(ENABLE_PIN1, HIGH);
             digitalWrite(ENABLE_PIN2, HIGH);
             restart();
             sendData("start");
        }

    }

}


void restart(){
    wdt_enable(WDTO_15MS);
    while (1) {}
  }


void buzz() {
  tone(BUZZER, 440);
  delay(500);
  noTone(BUZZER);
}
