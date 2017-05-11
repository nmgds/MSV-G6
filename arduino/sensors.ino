#include <Smartcar.h>

#define ULTRA_SIDE_PIN 0x73
#define ULTRA_FRONT_PIN 0x70


#define INFRA_SIDE_1_PIN A5
#define INFRA_SIDE_2_PIN A4
#define INFRA_BACK_PIN A2

#define ODOMETER_PIN 18


SRF08 ultrasoundSide, ultrasoundFront;
GP2D120 infraSide1, infraSide2, infraBack;
Odometer encoder;

int lastWheelVal = 0;
byte empty = 0x0;

void setup() {
  // put your setup code here, to run once:
  ultrasoundSide.attach(ULTRA_SIDE_PIN);
  ultrasoundFront.attach(ULTRA_FRONT_PIN);
  infraSide1.attach(INFRA_SIDE_1_PIN);
  infraSide2.attach(INFRA_SIDE_2_PIN);
  infraBack.attach(INFRA_BACK_PIN);
  encoder.attach(ODOMETER_PIN);
  encoder.begin();
  
 Serial.begin(115200);
 Serial.println("out of setup");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  sendIRUSValues();

}

String sendIRUSValues() {

  int infraSide1Val = infraSide1.getDistance();
  int infraSide2Val = infraSide2.getDistance();
  int infraBackVal = infraBack.getDistance();
  int wheelVal = encoder.getDistance();
  int ultrasoundFrontValue = ultrasoundFront.getDistance();
  if (ultrasoundFrontValue > 63) {
    ultrasoundFrontValue = 63;
  }
    int ultrasoundSideValue = ultrasoundSide.getDistance();
  if (ultrasoundSideValue > 63) {
    ultrasoundSideValue = 63;
  }
  

  byte wheel = empty;
  byte iSide1 = empty;
  byte iSide2 = empty;
  byte iBack = empty;
  byte uFront = empty;
  byte uSide = empty;

  //infrared side 1
  bitSet(iSide1, 5);
  iSide1 = iSide1 | infraSide1Val;

  //infrared side 2
  bitSet(iSide2, 6);
  iSide2 = iSide2 | infraSide2Val;

  //infrared back
  bitSet(iBack, 5);
  bitSet(iBack, 6);
  iBack = iBack | infraBackVal;

  //wheel encoder
  if(wheelVal > lastWheelVal){
    wheel = wheel | (wheelVal - lastWheelVal);
    lastWheelVal = wheelVal;
    Serial.println("Wheel encoder:");
    Serial.println(wheel);  
  }


  //ultrasounds
  bitSet(uFront, 7);
  uFront = uFront | ultrasoundFrontValue;
  
  bitSet(uSide, 7);
  bitSet(uSide, 6);
  uSide = uSide | ultrasoundSideValue;

  Serial.println("IR side 1:");
  Serial.println(iSide1);
  Serial.println("IR side 2:");
  Serial.println(iSide2);
  Serial.println("IR back:");
  Serial.println(iBack);
  Serial.println("US front:");
  Serial.println(uFront);
  Serial.println("US side:");
  Serial.println(uSide);

}