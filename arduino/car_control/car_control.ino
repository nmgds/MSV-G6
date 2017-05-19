#include <Servo.h>
#include <Smartcar.h>

#define INFRA_SIDE_1_PIN A5
#define INFRA_SIDE_2_PIN A4
#define INFRA_BACK_PIN A2
#define ULTRA_SIDE_PIN 0x70
#define ULTRA_FRONT_PIN 0x73
#define steerPinRC 3
#define escPinRC 5
#define servoPin 9
#define escPin 6
#define ODOMETER_PIN 2
#define TRIGGER_PIN_SENSOR_FRONT 12
#define ECHO_PIN_SENSOR_FRONT 13
#define TRIGGER_PIN_SENSOR_SIDE 7
#define ECHO_PIN_SENSOR_SIDE 8


Servo steering;
Servo motors;
GP2D120 infraSide1, infraSide2, infraBack;
//SRF08 ultrasoundSide, ultrasoundFront;
SR04 frontSensor, sideSensor;
Odometer encoder;

int rcControllerFlag;
int steer;
int drive;
String input;


  byte mask = 31; // mask for 0001 1111
  int steerValue = 60;
  byte steeringAmount;
  int speedValue = 1500;
  byte speedAmount;

  int currentSteering = steerValue;
  int currentSpeed = speedValue;



void setup() {
  // put your setup code here, to run once:
  steering.attach(servoPin);
  motors.attach(escPin);
  rcControllerFlag = 1;
  infraSide1.attach(INFRA_SIDE_1_PIN);
  infraSide2.attach(INFRA_SIDE_2_PIN);
  infraBack.attach(INFRA_BACK_PIN);
  //ultrasoundSide.attach(ULTRA_SIDE_PIN);
  //ultrasoundFront.attach(ULTRA_FRONT_PIN);
  frontSensor.attach(TRIGGER_PIN_SENSOR_FRONT,ECHO_PIN_SENSOR_FRONT);
  sideSensor.attach(TRIGGER_PIN_SENSOR_SIDE, ECHO_PIN_SENSOR_SIDE);
  steering.write(60);
  motors.writeMicroseconds(1500);
  attachInterrupt(digitalPinToInterrupt(3), rcControllerInterrupt, RISING);
  encoder.attach(ODOMETER_PIN);
  encoder.begin();
  Serial.begin(9600);
}
void loop() {
  // put your main code here, to run repeatedly:
  //checkRC();
 // steer = pulseIn(steerPinRC, HIGH);
 // drive = pulseIn(escPinRC, HIGH);

//  Serial.println(steer);
//  if (steer == 0)
//  {    
   steering.write(currentSteering);
    motors.writeMicroseconds(currentSpeed);
    
    if(Serial.available() > 0){
      byte cmd = (byte)Serial.read();
      if(bitRead(cmd, 7) == 1){
          steerCar(cmd);
      }
      else{
        moveCar(cmd);
      }
    }
  
//  else {
//    remoteControl();
//  }
/*
  if(Serial.available() > 0){
    String cmd = Serial.readStringUntil('\n');
    steering.write(60+cmd.toInt());

    
  }
  */
  sendIRUSValues();
  delay(10);
}
void checkRC() {

  Serial.println(pulseIn(steerPinRC, HIGH, 25000));

}

void steerCar(byte command){

  steerValue = 60;
  steeringAmount = command & mask;

  if (steeringAmount > 30){
    steeringAmount = 30;
  }

  if(bitRead(command, 5) == 1){ //positive value
    steerValue = steerValue + steeringAmount;
    
  }
  else{ //negative value
    steerValue = steerValue - steeringAmount;
    
  }
  currentSteering = steerValue;
}

void moveCar(byte command){

  speedValue = 1500;
  speedAmount = command & mask;

  if(bitRead(command, 5) == 1){ //positive value
    if (speedAmount > 13){
      speedAmount = 13;
    }
     speedValue = speedValue + (speedAmount * 10);
  }
  else{ //negative value
    if(speedAmount > 30){
      speedAmount = 30;
    }
    speedValue = speedValue - (speedAmount * 10);
    
  }
  currentSpeed = speedValue;
}



void rcControllerInterrupt() {
  rcControllerFlag = 1;
}

void remoteControl() {

  if (steer <= 1150) {
    steering.write(90);
  }
  else if (steer <= 1300) {
    steering.write(80);
  }
  else if (steer <= 1450) {
    steering.write(70);
  }
  else if (steer <= 1550) {
    steering.write(60);
  }
  else if (steer <= 1700) {
    steering.write(50);
  }
  else if (steer <= 1850) {
    steering.write(40);
  }
  else {
    steering.write(30);
  }

  // The car moves backward
  if (drive < 1200) {
    //
    motors.writeMicroseconds(1105);
  }
  else if (drive < 1300) {
    motors.writeMicroseconds(1250);
  }
  // The car moves forward
  else if (drive > 1600) {
    motors.writeMicroseconds(1630);
  }
    // the car does not drive
  else {
    motors.writeMicroseconds(1500);
  }

}

void sendIRUSValues() {

  int infraSide1Val = infraSide1.getDistance();
  int infraSide2Val = infraSide2.getDistance();
  int infraBackVal = infraBack.getDistance();
  int wheelVal = encoder.getDistance();
//  int ultrasoundFrontValue = frontSensor.getDistance();
//  int ultrasoundSideValue = ultrasoundSide.getDistance();


//  Serial.print(ultrasoundSide.getDistance());
//  Serial.print(',');
//  Serial.print(ultrasoundFront.getDistance());
//  Serial.print(',');

  
  Serial.write(0x01);
  Serial.write(sideSensor.getDistance());
  Serial.print('\n');
  Serial.write(0x02);
  Serial.write(frontSensor.getDistance());
  Serial.print('\n');
  Serial.write(0x04);
  Serial.write(infraSide1.getDistance());
  Serial.print('\n');
  Serial.write(0x08);
  Serial.write(infraSide2.getDistance());
  Serial.print('\n');
  Serial.write(0x10);
  Serial.write(infraBack.getDistance());
  Serial.print('\n');
  Serial.write(0x20);
  Serial.write(encoder.getDistance());
  Serial.print('\n');

}



