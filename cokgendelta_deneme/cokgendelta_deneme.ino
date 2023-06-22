
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver myservodriver = Adafruit_PWMServoDriver();

#define servoMIN 130
#define servoMAX 530
// servolarin 0 derece oldugu konum 270 pwm olarak elle hesaplandi
// boylece servomin ve servomax acilarini -63 ve servomaxi 117 olarak hesapladik

int servo1 = 0; 
int servo2 = 2;
int servo3 = 3;

//int ii = 450;

// circle variables
float rc = 30; // radius of the circle
float b0 = 100, b1 = 110, b2 = 196, b7 = 42, h0 = 500, h7 = 0; // dimensions of robot
// number of points on the circle
const int numPoints = 6; 
float tt = 2*M_PI/numPoints; // time step for circle
//float numPoints = 2 * M_PI / tt; // number of points on the circle
float teta[numPoints]; // angle
float p[3][numPoints]; // position

float beta[3] = {0, 2*M_PI/3, -2*M_PI/3};
float u1[3] = {1, 0, 0}, u2[3] = {0, 1, 0}, u3[3] = {0, 0, 1}; // unit vectors
float phi_k[3], Yk[3];
float th_k[3][numPoints];
float th_k0[3];
float theta_1[numPoints] , theta_2[numPoints], theta_3[numPoints];
float theta1 , theta2, theta3;
float rk[3][numPoints];
float rk1, rk2, rk3;
float anglesin, anglecos;
float Fk;
float sigma1 = 1, sigma2 = -1;
int count = 0;
// float a = 50; // one length of the square in mm
// float b;

void setup() {
  Serial.begin(9600);
  myservodriver.begin();
  myservodriver.setPWMFreq(50);

  //Inverse kinematics
  // Calculate the circle points
  for (int in = 0; in < numPoints; in++) {
    teta[in] = -M_PI + in * tt;
    p[0][in] = rc * cos(teta[in]);
    p[1][in] = rc * sin(teta[in]);
    p[2][in] = 320;
  // movement of square
  // b = (a*sqrt(2))/2;
  // p[0][0] = -b, p[1][0] = b, p[2][0] = 320;
  // p[0][1] = b, p[1][1] = b, p[2][1] = 320;
  // p[0][2] = b, p[1][2] = -b, p[2][2] = 320;
  // p[0][3] = -b, p[1][3] = -b, p[2][3] = 320;

  for (int i = 0; i < 3; i++) {
    float Rot3b[3][3] = {cos(-beta[i]), -sin(-beta[i]), 0, sin(-beta[i]), cos(-beta[i]), 0, 0, 0, 1};

    for (int j = 0; j < 3; j++) {
      rk[j][in] = Rot3b[j][0]*p[0][in] + Rot3b[j][1]*p[1][in] + Rot3b[j][2]*p[2][in] - u1[j]*(b0-b7) - u3[j]*(h0-h7);
    }

    rk1 = rk[0][in]*u1[0] + rk[1][in]*u1[1] + rk[2][in]*u1[2];
    rk2 = rk[0][in]*u2[0] + rk[1][in]*u2[1] + rk[2][in]*u2[2];
    rk3 = rk[0][in]*u3[0] + rk[1][in]*u3[1] + rk[2][in]*u3[2];

    anglesin = rk2;
    anglecos = sigma1*sqrt(pow(b2, 2) - pow(rk2, 2));
    phi_k[i] = atan2(anglesin, anglecos)*180/M_PI;

    Fk = (pow(rk1, 2) + pow(rk2, 2) + pow(rk3, 2) + pow(b1, 2) - pow(b2, 2))/(2*b1);
    
    th_k0[i] = -atan2(rk3, rk1)*180/M_PI;
    Yk[i]  = atan2(sqrt(pow(rk1 , 2) + pow(rk3 , 2) - pow(Fk , 2)), Fk )*180/M_PI;
    th_k[i][in] = th_k0[i] + sigma2*Yk[i];

    //thstr_k[i][in] = -atan2(sigma1*(rk3[in]+b1*sin(th_k[i][in]*M_PI/180)), sigma1*(rk1[in]-b1*cos(th_k[i][in]*M_PI/180)))*180/M_PI;
    //thprime_k[i][in] = thstr_k[i][in] - th_k[i][in];
    }
    theta_1[in] = map(th_k[0][in], -63, 117 ,servoMIN, servoMAX);
    theta_2[in] = map(th_k[1][in], -63, 117 ,servoMIN, servoMAX);
    theta_3[in] = map(th_k[2][in], -63, 117 ,servoMIN, servoMAX);
  }
 
  // Serial.print("theta 1 yeni:");
  // Serial.println(theta_1[0]);
  // Serial.print("theta 2 yeniii:");
  // Serial.println(theta_2[0]);
  // Serial.print("theta 3 yeniii:");
  // Serial.println(theta_3[0]);
  // Serial.println(th_k[0][0]);
  // Serial.println(th_k[1][0]);
  // Serial.println(th_k[2][0]);

}

void loop() {

//  for(int ix = 0; ix < numPoints; ix++) { //use numPoints instead of 8 later on
//
//    myservodriver.setPWM(servo1, 0, theta_1[ix]);
//    myservodriver.setPWM(servo2, 0, theta_2[ix]);
//    myservodriver.setPWM(servo3, 0, theta_3[ix]);
//
//    Serial.println(th_k[0][ix]);
//    Serial.println(th_k[1][ix]);
//    Serial.println(th_k[2][ix]);
//    delay(500); // remove the delay later on
//    
//    // Serial.println(theta_1[ix]);
//    // Serial.println(theta_2[ix]);
//    // Serial.println(theta_3[ix]);
//
//  }

}
