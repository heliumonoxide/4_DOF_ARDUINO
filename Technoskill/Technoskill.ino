#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

#define servo1 2 // Rotasi Bawah
#define servo2 3 // Capit
#define servo3 4 // Kiri (diliat dari belakang)
#define servo4 5 // Kanan (diliat dari belakang)

unsigned long startTime = millis();
float time_dif = 0;

const int joyX1 = A0;
const int joyY1 = A1;
const int joyX2 = A2;
const int joyY2 = A3; 

float joyvalX1;
float joyvalY1;
float joyvalX2;
float joyvalY2;

float start_degree_1 = 0; // HARUS DIGANTI TERLEBIH DAHULU
float start_degree_2 = 0; // HARUS DIGANTI TERLEBIH DAHULU
float start_degree_3 = 0; // HARUS DIGANTI TERLEBIH DAHULU
float start_degree_4 = 0; // HARUS DIGANTI TERLEBIH DAHULU

float current_deg_servo_1;
float current_deg_servo_2;
float current_deg_servo_3;
float current_deg_servo_4;

const int mapping[4][2] = {
  {110,525}, // Dimapping dulu
  {100,535}, // Dimapping dulu
  {100,535}, // Dimapping dulu untuk kirir
  {100,535} // Dimapping dulu
};

void setup() {
  Serial.begin(9600);
  driver.begin();
  driver.setPWMFreq(50);   
  
  //driver.setPWM(servo1, 0, start_degree_1); 
  //driver.setPWM(servo2, 0, start_degree_2); 
  //driver.setPWM(servo3, 0, start_degree_3); 
  //driver.setPWM(servo4, 0, start_degree_4);
   
  current_deg_servo_1 = start_degree_1;
  current_deg_servo_2 = start_degree_2;
  current_deg_servo_3 = start_degree_3;
  current_deg_servo_4 = start_degree_4;
  
  delay(1000);
  
  startTime = millis();
}
void loop(){
    unsigned long currentTime = millis();
    joyvalX1 = analogRead(joyX1);  
    
    // Kalibrasi Joystick X
    joyvalX1 += 12;

    joyvalX1 = map(joyvalX1, 00, 658, -50, 50);
    Serial.println("======================");
    Serial.println(joyvalX1);     
                   

    joyvalY1 = analogRead(joyY1);           
    joyvalY1 = map(joyvalY1, 00, 658, -50, 50);   
    Serial.println(joyvalY1);  

    joyvalX2 = analogRead(joyX2);  

    // Kalibrasi Joystick X
    joyvalX2 += 12;

    joyvalX2 = map(joyvalX2, 00, 658, -50, 50);
    Serial.println("======================");
    //Serial.println(joyvalX2);                        

    joyvalY2 = analogRead(joyY2);           
    joyvalY2 = map(joyvalY2, 00, 658, -50, 50);   
    //Serial.println(joyvalY2);

    time_dif = float(currentTime - startTime)/1000;
    
    float dx1 = joyvalX1 * time_dif;
    float dy1 = joyvalY1 * time_dif;
    float dx2 = joyvalX2 * time_dif;
    float dy2 = joyvalY2 * time_dif;

    if((current_deg_servo_1 >= mapping[0][0] && current_deg_servo_1 < mapping[0][1]) || (current_deg_servo_1 > mapping[0][1] && joyvalX1 < 0) || (current_deg_servo_1 < mapping[0][0] && joyvalX1 > 0)) {
      current_deg_servo_1 = current_deg_servo_1 + dx1;
    }

    delay(20);

    if((current_deg_servo_2 >= mapping[1][0] && current_deg_servo_2 < mapping[1][1]) || (current_deg_servo_2 > mapping[1][1] && joyvalY1 < 0) || (current_deg_servo_2 < mapping[1][0] && joyvalY1 > 0)) {
      current_deg_servo_2 = current_deg_servo_2 + dy1;
    }

    delay(20);
    
    if((current_deg_servo_3 >= mapping[2][0] && current_deg_servo_3 < mapping[2][1]) || (current_deg_servo_3 > mapping[2][1] && joyvalX2 < 0) || (current_deg_servo_3 < mapping[2][0] && joyvalX2 > 0)){
      current_deg_servo_3 = current_deg_servo_3 + dx2;
    }

    delay(20);

    if((current_deg_servo_4 >= mapping[3][0] && current_deg_servo_4 < mapping[3][1]) || (current_deg_servo_4 > mapping[3][1] && joyvalY2 < 0) || (current_deg_servo_4 < mapping[3][0] && joyvalY2 > 0)) {
      current_deg_servo_4 = current_deg_servo_4 + dy2;
    }

    delay(20);
    
    Serial.println("time dx 1:" +String(dx1));
    Serial.println("servo bawah:" +String(current_deg_servo_1));
    driver.setPWM(servo1, 0, current_deg_servo_1);
    driver.setPWM(servo2, 0, current_deg_servo_2);
    driver.setPWM(servo3, 0, current_deg_servo_3);
    driver.setPWM(servo4, 0, current_deg_servo_4);
    
    startTime = currentTime;


    
}
