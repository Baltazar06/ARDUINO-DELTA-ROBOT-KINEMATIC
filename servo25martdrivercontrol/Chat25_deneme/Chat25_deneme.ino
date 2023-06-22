void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

// Define the length of the arms and the position of the base
float l1 = 100.0;
float l2 = 100.0;
float l3 = 100.0;
float base_x = 0.0;
float base_y = 0.0;
float base_z = 0.0;

// Define the position and orientation of the end effector
float x = 50.0;
float y = 75.0;
float z = 100.0;
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

// Define the tolerance for the solution
float tol = 0.001;

// Define the maximum number of iterations
int max_iter = 1000;

// Initialize the joint angles to zero
float theta1 = 0.0;
float theta2 = 0.0;
float theta3 = 0.0;

// Define the pins for the servo motors
int servo1_pin = 9;
int servo2_pin = 10;
int servo3_pin = 11;

// Define the calibration values for the servo motors
int servo1_min = 0;
int servo1_max = 180;
int servo2_min = 0;
int servo2_max = 180;
int servo3_min = 0;
int servo3_max = 180;

// Define the servo objects
Servo servo1;
Servo servo2;
Servo servo3;

void setup() {
  // Attach the servo objects to the pins
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);

  // Set the initial position of the servo motors
  servo1.write(map(theta1, 0, 360, servo1_min, servo1_max));
  servo2.write(map(theta2, 0, 360, servo2_min, servo2_max));
  servo3.write(map(theta3, 0, 360, servo3_min, servo3_max));
}

void loop() {
  // Calculate the initial position of the end effector
  float y1 = l1 * sin(theta1);
  float z1 = l1 * cos(theta1);

  float y2 = l2 * sin(theta2);
  float z2 = l2 * cos(theta2);

  float y3 = l3 * sin(theta3);
  float z3 = l3 * cos(theta3);

  float p = (z1 + z2 + z3) / 3.0;
  float q = sqrt( pow(y1 - base_y, 2) + pow(z1 - base_z, 2) );

  float a = sqrt( pow(l2,2) - pow(l3,2) );
  float b = sqrt( pow(q,2) + pow(a - (p - base_x),2) );

  float alpha = atan2( a - (p - base_x), q );
  float beta = atan2( z1 - base_z, y1 - base_y );

  float x_calc = b * sin(alpha + beta);
  float y_calc = b * cos(alpha + beta);
  float z_calc = p + base_x - a * cos(alpha);

  // Calculate the error between the desired and calculated end effector positions
  float error = sqrt( pow(x - x_calc, 2) + pow(y - y_calc, 2) + pow(z - z_calc, 2) );

  // Initialize the iteration counter
  int iter = 0;

 
