#include <Servo.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

#define servo1 0
#define servo5 4
#define servo2 1
#define servo3 2


//const int servo1 = 3;      //servo 1
//const int servo2 = 5;       //servo 2
const int joyX = A0;
const int joyY = A1;

const int mapping[5][2] = {
  {0,540},
  {0,540},
  {0,540},
  {0,540},
  {0,540}
};

int servoVal1;
int servoVal2;


//Servo myservo1;
//Servo myservo2;

void setup() {
//  myservo1.attach(servo1); 
//  myservo2.attach(servo2);
  Serial.begin(9600);
  driver.begin();
  driver.setPWMFreq(50);
  driver.setPWM(servo1, 0, 0);
  driver.setPWM(servo5, 0, 0);
  delay(3000);
}


void loop(){
    servoVal1 = analogRead(joyX);  

    // Kalibrasi Joystick X
    servoVal1 += 12;

    servoVal1 = map(servoVal1, 00, 1023, -4, 4);
    Serial.println("======================");
    Serial.println(servoVal1);   

//    myservo2.write(servoVal);                      

    servoVal2 = analogRead(joyY);           
    servoVal2 = map(servoVal2, 00, 1023, -4, 4);   
    Serial.println(servoVal2);  

//    myservo1.write(servoVal);                   

    for (int S1value = mapping[0][0]; S1value <= mapping[0][1]; S1value+=servoVal1) {
      driver.setPWM(servo1, 0, S1value);
      delay(10);
    }

    for (int S5value = mapping[4][0]; S5value <= mapping[4][1]; S5value+=servoVal2) {
      driver.setPWM(servo5, 0, S5value);
      delay(10);
    }
  
    delay(2000);
}
