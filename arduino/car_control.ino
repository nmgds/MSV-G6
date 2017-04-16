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
Servo steering;
Servo motors;
GP2D120 infraSide1, infraSide2, infraBack;
SRF08 ultrasoundSide, ultrasoundFront;
int rcControllerFlag;
int steer;
int drive;
String input;
void setup() {
  // put your setup code here, to run once:
  steering.attach(servoPin);
  motors.attach(escPin);
  rcControllerFlag = 1;
  infraSide1.attach(INFRA_SIDE_1_PIN);
  infraSide2.attach(INFRA_SIDE_2_PIN);
  infraBack.attach(INFRA_BACK_PIN);
  ultrasoundSide.attach(ULTRA_SIDE_PIN);
  ultrasoundFront.attach(ULTRA_FRONT_PIN);
  steering.write(60);
  motors.write(1500);
  attachInterrupt(digitalPinToInterrupt(3), rcControllerInterrupt, RISING);
  Serial.begin(9600);
}
void loop() {
  // put your main code here, to run repeatedly:
  //checkRC();
  steer = pulseIn(steerPinRC, HIGH);
  drive = pulseIn(escPinRC, HIGH);



  if (steer == 0)
  {
    steering.write(60);
    motors.write(1500);

  }
  else {
    remoteControl();
  }

}
void checkRC() {

  Serial.println(pulseIn(steerPinRC, HIGH, 25000));

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

  if (drive < 1200) {
    motors.writeMicroseconds(1105);
  }
  else if (drive < 1300) {
    motors.writeMicroseconds(1250);
  }
  else if (drive > 1600) {
    motors.writeMicroseconds(1630);
  }
  else {
    motors.writeMicroseconds(1500);
  }

}



