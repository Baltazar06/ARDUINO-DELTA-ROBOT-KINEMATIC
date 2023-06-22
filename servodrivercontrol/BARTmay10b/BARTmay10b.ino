void setup() {


#include <math.h>

// Define geometric parameters
const int b0 = 100;
const int b1 = 110;
const int b2 = 196;
const int b7 = 42;
const int h0 = 500;
const int h7 = 0;

// Define unit vectors
const int u1x = 1;
const int u1y = 0;
const int u1z = 0;
const int u2x = 0;
const int u2y = 1;
const int u2z = 0;
const int u3x = 0;
const int u3y = 0;
const int u3z = 1;

// Define desired p vector as a effector position
const int p_x = 0;
const int p_y = 0;
const int p_z = 400;

// Define function to solve for unknown angles
void solve_angles(int *phi_k, int *th_k, int *thstr_k, int *Yk, int *th_k0, int *thprime_k) {
  // Define rotation matrix for each leg (B1,B2,B3 respectively)
  int Rot3b[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Rot3b[i][j] = cos(-beta[i]) * u1[j] + sin(-beta[i]) * u2[j];
    }
  }

  // First we define the rk position vector of leg from point Bk to Ak
  int rk[3];
  for (int i = 0; i < 3; i++) {
    rk[i] = Rot3b[i][0] * p_x + Rot3b[i][1] * p_y + Rot3b[i][2] * p_z - b0 * u1[i] - h0 * u3[i];
  }

  // Then we will be using unit vectors of rk to define unkown angles
  int rk1 = rk[0];
  int rk2 = rk[1];
  int rk3 = rk[2];

  // Define unknown angles with solve function
  // all angles are in degree
  // to define phi_k angle
  int sigma1 = 1; // sigma_prime
  int anglesin = rk2; // represents the sin(phi_k)
  int anglecos = sigma1 * sqrt(b2 * b2 - rk2 * rk2); // represents cos(phi_k)
  phi_k[i] = atan2(anglesin, anglecos) * 180 / M_PI; // to define phi_k angle

  // to define th_k angle
  int sigma2 = -1; // sigma k
  int Fk = ((rk1 * rk1) + (rk2 * rk2) + (rk3 * rk3) + (b1 * b1) - (b2 * b2)) / (2 * b1);

  // version from the book
  th_k0[i] = -atan2(rk3, rk1) * 180 / M_PI; // to define theta0 angle in degree
  Yk[i] = atan2(sqrt(rk1 * rk1 + rk3 * rk3 - Fk * Fk), Fk) * 180 / M_PI;
  th_k[i] = th_k0[i] + sigma2 * Yk[i]; // th_k angle

  // to define thstr_k angle
  thstr_k[i] = -atan2(sigma1 * (rk3 + b1 * sin(th_k[i] * M_PI / 180)), sigma1 * (rk1 - b1 * cos(th_k[i] * M_PI / 180))) * 180 / M_PI;
  thprime_k[i] = thstr

  

}

void loop() {
  
}
