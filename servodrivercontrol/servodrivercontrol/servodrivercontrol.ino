/*servo motor driver board control
   Home Page
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver srituhobby = Adafruit_PWMServoDriver();

#define servoMIN 150
#define servoMAX 470
#define servoMID 300

int servo1 = 0; 
int servo2 = 2;
int servo3 = 3;

int ii = 450;

void setup() {
  Serial.begin(9600);
  srituhobby.begin();
  srituhobby.setPWMFreq(50);
}

void loop() {

  for(ii = 450; ii > 350; ii--)
  {
    srituhobby.setPWM(servo1, 0, ii);
    Serial.println(servo1);
    
    srituhobby.setPWM(servo2, 0, ii);
    Serial.println(servo2);
    
    srituhobby.setPWM(servo3, 0, ii);
    Serial.println(servo3);
    delay(50);
  }
  
  for(ii = 350; ii < 450; ii++)
  {
    srituhobby.setPWM(servo1, 0, ii);
    Serial.println(servo1);
    
    srituhobby.setPWM(servo2, 0, ii);
    Serial.println(servo2);
 
    srituhobby.setPWM(servo3, 0, ii);
    Serial.println(servo3);
    delay(50);
  }
  
}
