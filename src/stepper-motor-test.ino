//***Stepper Motor Test Code BY Aaed Musa***
//-----------------------------------------
#define EN_PIN           8 // Enable pin
#define DIR_PIN_1        5 // Direction pin for motor 1
#define STEP_PIN_1       2 // Step pin for motor 1
#define DIR_PIN_2        6 // Direction pin for motor 2
#define STEP_PIN_2       3 // Step pin for motor 2
#define DIR_PIN_3        7 // Direction pin for motor 3
#define STEP_PIN_3       4 // Step pin for motor 3

//libraries
#include <AccelStepper.h>
#include <MultiStepper.h>

//stepper motors
AccelStepper stepperA(1, STEP_PIN_1, DIR_PIN_1);  //(driver type, STEP, DIR) Driver A
AccelStepper stepperB(1, STEP_PIN_2, DIR_PIN_2);  //(driver type, STEP, DIR) Driver B
AccelStepper stepperC(1, STEP_PIN_3, DIR_PIN_3);  //(driver type, STEP, DIR) Driver C
MultiStepper steppers;           // Create instance of MultiStepper

//stepper motor variables
int pos[3] = {400, 400, 400};                            // An array to store the target positions for each stepper motor
int ENA = EN_PIN;                           //enable pin for the drivers

void setup() {
  //Set iniial maximum speed value for the steppers (steps/sec)
  stepperA.setMaxSpeed(200);
  stepperB.setMaxSpeed(200);
  stepperC.setMaxSpeed(200);
  // Adding the steppers to the steppersControl instance for multi stepper control
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);
  //Enable pin
  pinMode(ENA, OUTPUT);    //define enable pin as output
  digitalWrite(ENA, LOW);  //sets the drivers on initially
  delay(1000);             //small delay to allow the user to reset the platform
  //Movemement
  steppers.moveTo(pos);  // Calculates the required speed for all motors
  steppers.runSpeedToPosition();  // blocks until all steppers reach their target position
}
void loop(){
}
