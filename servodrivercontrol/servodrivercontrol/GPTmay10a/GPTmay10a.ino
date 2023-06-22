#include <math.h>

float b0 = 100, b1 = 110, b2 = 196, b7 = 42, h0 = 500, h7 = 0;
float beta[3] = {0, 2*M_PI/3, -2*M_PI/3};
float u1[3] = {1, 0, 0}, u2[3] = {0, 1, 0}, u3[3] = {0, 0, 1};
float p[3] = {0, 0, 400};
float phi_k[3] = {0}, th_k[3] = {0}, thstr_k[3] = {0}, Yk[3] = {0};
float th_k0[3] = {0}, thprime_k[3] = {0};

void setup() {
  Serial.begin(9600);

    //Inverse kinematics
    for (int i = 0; i < 3; i++) {
      float Rot3b[3][3] = {cos(-beta[i]), -sin(-beta[i]), 0, sin(-beta[i]), cos(-beta[i]), 0, 0, 0, 1};
      float rk[3] = {0};

      for (int j = 0; j < 3; j++) {
        rk[j] = Rot3b[j][0]*p[0] + Rot3b[j][1]*p[1] + Rot3b[j][2]*p[2] - u1[j]*(b0-b7) - u3[j]*(h0-h7);
      }

      float rk1 = rk[0]*u1[0] + rk[1]*u1[1] + rk[2]*u1[2];
      float rk2 = rk[0]*u2[0] + rk[1]*u2[1] + rk[2]*u2[2];
      float rk3 = rk[0]*u3[0] + rk[1]*u3[1] + rk[2]*u3[2];

      float sigma1 = 1;
      float anglesin = rk2;
      float anglecos = sigma1*sqrt(pow(b2, 2) - pow(rk2, 2));
      phi_k[i] = atan2(anglesin, anglecos)*180/M_PI;

      float sigma2 = -1;
      float Fk = (pow(rk1, 2) + pow(rk2, 2) + pow(rk3, 2) + pow(b1, 2) - pow(b2, 2))/(2*b1);
      
      th_k0[i] = -atan2(rk3, rk1)*180/M_PI;
      Yk[i] = atan2(sqrt(pow(rk1, 2) + pow(rk3, 2) - pow(Fk, 2)), Fk)*180/M_PI;
      th_k[i] = th_k0[i] + sigma2*Yk[i];
      
      thstr_k[i] = -atan2(sigma1*(rk3+b1*sin(th_k[i]*M_PI/180)), sigma1*(rk1-b1*cos(th_k[i]*M_PI/180)))*180/M_PI;
      thprime_k[i] = thstr_k[i] - th_k[i];
    }
    //Serial.println(thprime_k[1]);
    Serial.println(thprime_k[0]);
    Serial.println(thprime_k[1]);
    Serial.println(thprime_k[2]);
    Serial.println(th_k[0]);
    Serial.println(th_k[1]);
    Serial.println(th_k[2]);
    
}

void loop() {

}
