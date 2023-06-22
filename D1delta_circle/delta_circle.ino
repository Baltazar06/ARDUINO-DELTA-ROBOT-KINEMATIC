
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver srituhobby = Adafruit_PWMServoDriver();

#define servoMIN 130
#define servoMAX 530
// servolarin 0 derece oldugu konum 270 pwm olarak elle hesaplandi
// boylece servomin ve servomax acilarini -63 ve servomaxi 117 olarak hesapladik

int servo1 = 0; 
int servo2 = 2;
int servo3 = 3;

int ii = 450;

#include <math.h>

// circle variables
float rc = 30; // radius of the circle
float tt = 1; // time step

//float numPoints = 2 * M_PI / tt; // number of points on the circle
int numPoints = 2 * M_PI / tt; // number of points on the circle

float teta[numPoints]; // angle
float p[3][numPoints]; // position
//float xc[numPoints];
//float yc[numPoints];
//float zc[numPoints];
//float lx = numPoints;

float b0 = 100, b1 = 110, b2 = 196, b7 = 42, h0 = 500, h7 = 0;
float beta[3] = {0, 2*M_PI/3, -2*M_PI/3};
float u1[3] = {1, 0, 0}, u2[3] = {0, 1, 0}, u3[3] = {0, 0, 1};
//float p[3] = {0, -30, 320};
float phi_k[3][numPoints] = {0}, th_k[3][numPoints]  = {0}, Yk[3][numPoints]  = {0};
float th_k0[3][numPoints] = {0};
float theta_1[numPoints] , theta_2[numPoints], theta_3[numPoints];
float theta1 , theta2, theta3;
float rk1[numPoints], rk2[numPoints], rk3[numPoints];
float anglesin[numPoints], anglecos[numPoints];
float Fk[numPoints];
//float thstr_k[3][numPoints] = {0}, thprime_k[3][numPoints] = {0};

void setup() {
  Serial.begin(9600);
  srituhobby.begin();
  srituhobby.setPWMFreq(50);

  //Inverse kinematics
  //for(int ii =0; ii<lx ;ii++){
     // Calculate the circle points
  for (int in = 0; in < numPoints; in++) {
    teta[in] = -M_PI + in * tt;
    //xc[in] = rc * cos(teta[in]);
    //yc[in] = rc * sin(teta[in]);
    //zc[in] = 320;
    p[0][in] = rc * cos(teta[in]);
    p[1][in] = rc * sin(teta[in]);
    p[2][in] = 320;
  
  for (int i = 0; i < 3; i++) {
    float Rot3b[3][3] = {cos(-beta[i]), -sin(-beta[i]), 0, sin(-beta[i]), cos(-beta[i]), 0, 0, 0, 1};
    float rk[3][in] = {0};

    for (int j = 0; j < 3; j++) {
      rk[j][in] = Rot3b[j][0]*p[0][in] + Rot3b[j][1]*p[1][in] + Rot3b[j][2]*p[2][in] - u1[j]*(b0-b7) - u3[j]*(h0-h7);
    }

    rk1[in] = rk[0][in]*u1[0] + rk[1][in]*u1[1] + rk[2][in]*u1[2];
    rk2[in] = rk[0][in]*u2[0] + rk[1][in]*u2[1] + rk[2][in]*u2[2];
    rk3[in] = rk[0][in]*u3[0] + rk[1][in]*u3[1] + rk[2][in]*u3[2];

    float sigma1 = 1;
    anglesin[in] = rk2[in];
    anglecos[in] = sigma1*sqrt(pow(b2, 2) - pow(rk2[in], 2));
    phi_k[i][in] = atan2(anglesin[in], anglecos[in])*180/M_PI;

    float sigma2 = -1;
    Fk[in] = (pow(rk1[in], 2) + pow(rk2[in], 2) + pow(rk3[in], 2) + pow(b1, 2) - pow(b2, 2))/(2*b1);
    
    th_k0[i][in] = -atan2(rk3[in], rk1[in])*180/M_PI;
    Yk[i][in]  = atan2(sqrt(pow(rk1[in] , 2) + pow(rk3[in] , 2) - pow(Fk[in] , 2)), Fk[in] )*180/M_PI;
    th_k[i][in] = th_k0[i][in] + sigma2*Yk[i][in];
    
    //thstr_k[i][in] = -atan2(sigma1*(rk3[in]+b1*sin(th_k[i][in]*M_PI/180)), sigma1*(rk1[in]-b1*cos(th_k[i][in]*M_PI/180)))*180/M_PI;
    //thprime_k[i][in] = thstr_k[i][in] - th_k[i][in];
    }
    theta_1[in] = map(th_k[0][in], -63, 117 ,servoMIN, servoMAX);
    theta_2[in] = map(th_k[1][in], -63, 117 ,servoMIN, servoMAX);
    theta_3[in] = map(th_k[2][in], -63, 117 ,servoMIN, servoMAX);
  }
 

  //Serial.print(thprime_k[1]);
  //Serial.print("theta 1:");
  //Serial.println(th_k[0]);
  //Serial.print("theta 2:");
  //Serial.println(th_k[1]);
  //Serial.print("theta 3:");
  //Serial.println(th_k[2]);

  //theta_1 = map(th_k[0], -63, 117 ,servoMIN, servoMAX);
  //theta_2 = map(th_k[1], -63, 117 ,servoMIN, servoMAX);
  //theta_3 = map(th_k[2], -63, 117 ,servoMIN, servoMAX);

  Serial.print("theta 1:");
  Serial.println(theta_1[0]);
  Serial.print("theta 2:");
  Serial.println(theta_2[0]);
  Serial.print("theta 3:");
  Serial.println(theta_3[0]);

  //float angle_theta[3] = {theta_1, theta_2, theta_3};
  //Serial.println(theta_1[0]);
  //Serial.println(theta_2[0]);
  //Serial.println(theta_3[0]);
}

void loop() {
  
  //    //Serial.println(th1);
  // //Serial.println(theta_1);
  // srituhobby.setPWM(servo1, 0, theta_1);
  // Serial.println(theta_1);

  // srituhobby.setPWM(servo2, 0, theta_2);
  // Serial.println(theta_2);

  // srituhobby.setPWM(servo3, 0, theta_3);
  // Serial.println(theta_3);
  // delay(3000);
  

  // theta1 = 341, theta2=314, theta3 = 363;
  //    //Serial.println(th1);
  // //Serial.println(theta_1);
  // srituhobby.setPWM(servo1, 0, theta1);
  // Serial.println(theta1);

  // srituhobby.setPWM(servo2, 0, theta2);
  // Serial.println(theta2);

  // srituhobby.setPWM(servo3, 0, theta3);
  // Serial.println(theta3);
  // delay(3000);
}
