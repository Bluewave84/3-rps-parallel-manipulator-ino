#include <AccelStepper.h>
#include <TMCStepper.h>
#include <math.h>

// Define pin connections
#define EN_PIN           8 // Enable pin
#define DIR_PIN_1        5 // Direction pin for motor 1
#define STEP_PIN_1       2 // Step pin for motor 1
#define DIR_PIN_2        6 // Direction pin for motor 2
#define STEP_PIN_2       3 // Step pin for motor 2
#define DIR_PIN_3        7 // Direction pin for motor 3
#define STEP_PIN_3       4 // Step pin for motor 3

#define R_SENSE 0.11f // Sense resistor value

// Create TMC2209 driver instances
TMC2209Stepper driver1(&Serial1, R_SENSE);
TMC2209Stepper driver2(&Serial1, R_SENSE);
TMC2209Stepper driver3(&Serial1, R_SENSE);

// Create AccelStepper instances for each motor
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);

// Define manipulator dimensions
const float L1 = 100.0; // Length of first link
const float L2 = 100.0; // Length of second link

void setup() {
  Serial.begin(115200); // Initialize serial communication
  Serial1.begin(115200); // Initialize serial communication for TMC2209

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable the drivers

  // Initialize TMC2209 drivers
  driver1.begin();
  driver1.toff(5);
  driver1.rms_current(600); // Set motor current
  driver1.microsteps(16);

  driver2.begin();
  driver2.toff(5);
  driver2.rms_current(600); // Set motor current
  driver2.microsteps(16);

  driver3.begin();
  driver3.toff(5);
  driver3.rms_current(600); // Set motor current
  driver3.microsteps(16);

  // Set max speed and acceleration for each stepper
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);
}

void loop() {
  // Example control loop for the 3 RPS Parallel Manipulator

  // Specify target end-effector position (x, y, z)
  float x = 50.0;
  float y = 50.0;
  float z = 50.0;

  // Calculate joint angles using inverse kinematics
  float theta1, theta2, theta3;
  inverseKinematics(x, y, z, theta1, theta2, theta3);

  // Convert joint angles to stepper motor positions
  long pos1 = angleToSteps(theta1);
  long pos2 = angleToSteps(theta2);
  long pos3 = angleToSteps(theta3);

  // Move each stepper motor to the calculated position
  stepper1.moveTo(pos1);
  stepper2.moveTo(pos2);
  stepper3.moveTo(pos3);

  // Run the steppers to the target positions
  stepper1.run();
  stepper2.run();
  stepper3.run();
}

// Function to calculate inverse kinematics
void inverseKinematics(float x, float y, float z, float &theta1, float &theta2, float &theta3) {
  // Placeholder calculations for inverse kinematics
  // Replace with actual calculations for your manipulator
  theta1 = atan2(y, x);
  theta2 = atan2(z, sqrt(x*x + y*y));
  theta3 = atan2(y, x); // Example placeholder calculation
}

// Function to convert angle to stepper motor steps
long angleToSteps(float angle) {
  // Placeholder conversion for angle to steps
  // Replace with actual conversion based on your stepper motor configuration
  return angle * 100;
}