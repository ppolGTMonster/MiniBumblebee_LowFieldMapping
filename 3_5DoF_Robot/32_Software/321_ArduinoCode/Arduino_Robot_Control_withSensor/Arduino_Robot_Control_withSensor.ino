/**********************************************************************/
//    ARDUINO_ROBOT_CONTROL_WITHSENSOR to control the 5DoF robotic arm
//     
//    Here the Robotic Arm "TinkerCAD Braccio" was used with a self-developed Driver Shield
//
//    In order to improve the performance a self developed Library "BraccioV2slowed" was used
//    which is based on the published library "BraccioV2" from Lukas Severinghaus:
//      * https://www.arduino.cc/reference/en/libraries/bracciov2/
//      * https://github.com/kk6axq/BraccioV2
//    (Licensed under GNU GPL v3 license)
//
//    Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany
//    MIT LICENSED
//    Have fun guys!
/**********************************************************************/

// 1) INCLUDE & DEFINES

// Defining Robotic Arm related objects/variables
#include <Servo.h>            // standard Arduino library (can be installed through lib manager)
#include "BraccioV2slowed.h"  // new library (see libs folder in github)
#define GRIPPER_POS 78        // The Braccio has a gripper (6th DOF), which is not connected in our system -> use the standard value here
#define WRIST_ROTATION 74     // The Wrist rot angle is set once during the setup to adjust the rotation as horizontal as possible. Then its kept constant

// Used Pins on Power Drive Shield 
#define RELAIS_POWER 14       // Relais Switch on/off
#define STS_LED 13            // Status LED

// 2) Generate Objects
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

Braccio arm;

// 3) Define used variables
bool update = false;
String readString = "";

// min/max angle (Degree) for every jpint - determined during setup 
int base_min = 0;
int base_max = 180;
int shoulder_min = 14;
int shoulder_max = 166;
int ellbow_min = 0;
int ellbow_max = 180;
int wrist_min = 0;
int wrist_max = 180;

// StartUp-Position: Must be found before measurements. Sensor should be postioned already inside the Magnet
// During the startup the servo motors are not powered (Relais disconnects the power)
// Then after powering the sensor will be moved fast into the start position. Therefore the sensor should be already inside the magnet near the start position
// It has proven useful to manually hold the sensor close to its starting position when powering the Arduino until the motors are powered (approx 10sec)   
int alpha_Base = 85;
int alpha_Shoulder = 45;
int alpha_Ellbow = 55;
int alpha_Wrist = 70;

// Calibration Angles, in case motors have some tolerances. 
// Here: Better use standard values (90°). Set the tolerances directly in your Control Software (ie Matlab/Python Script)
int alpha_Base_cal = 90;
int alpha_Shoulder_cal = 90;
int alpha_Ellbow_cal = 90;
int alpha_Wrist_cal = 90;

// Readout of Angle of Potentiometer (extra Base Angle Sensor)
int PotiPin = A2;
int UccPin = A1;
double Uref_ADC = 3.745;    //V, set to your Reference Voltage on your design
double lpoti = 56.5;        //mm, Properties of sensor system 
double Ucc = 5.0;           //V
double l = 77.698;          //mm
double x0 = 18.96;          //mm
double alphaBase_Lin_a1 = 7.633209009676445; //°/V calcualted with calibration/linearization + Smallangle Approx, alphaBase = a1*Volt+a2 in °
double alphaBase_Lin_a2 = -11.672975576939738 ; //° calc with calibration/Linearization


double k ; 
int uout_x_raw;
double uout_x;
double x;
double x_s; 
double alpha_meas_rad;
double alpha_meas; 
double alpha_Base_meas;
String alpha_Base_meas_Str;



// 4) SETUP THE ARDUINO
void setup() {

  analogReference(EXTERNAL);  // Set External Reference Voltage (Uref_ADC)

  // Using Pin A0 = D14 as Relais-Bridge to disconnect the power to the Power-Shield to prevent the robot from jumping around during StartUp
  pinMode(RELAIS_POWER,OUTPUT);
  // Using Pin D13 as a Status LED on the Power-shield
  pinMode(STS_LED,OUTPUT); 
 
  digitalWrite(RELAIS_POWER,LOW); // Relais is off -> No Power is sent to the Power-Shield
  digitalWrite(STS_LED,LOW);


  Serial.begin(115200);
  Serial.println("Initializing... Please Wait");  //Start of initialization

  //Update these lines with the calibration angles
  arm.setJointCenter(BASE_ROT, alpha_Base_cal);
  arm.setJointCenter(SHOULDER, alpha_Shoulder_cal);
  arm.setJointCenter(ELBOW, alpha_Ellbow_cal);
  arm.setJointCenter(WRIST, alpha_Wrist_cal);
  arm.setJointCenter(WRIST_ROT, WRIST_ROTATION);
  arm.setJointCenter(GRIPPER, GRIPPER_POS);

  arm.setDelta(BASE_ROT,1);
  arm.setDelta(SHOULDER,1);
  arm.setDelta(ELBOW,1);
  arm.setDelta(WRIST,1);
  arm.setDelta(WRIST_ROT,1);
  arm.setDelta(GRIPPER,1);

 //Start to custom position.
  arm.begin(false);
  arm.setAllNowSoftStartUp(alpha_Base, alpha_Shoulder, alpha_Ellbow, alpha_Wrist,WRIST_ROTATION, GRIPPER_POS);
  //This method allows a custom start position to be set, but the setAllNow method MUST be run
  //immediately after the begin method and before any other movement commands are issued.

  //NOTE: The begin method takes approximately 8 seconds to start, due to the time required
  //to initialize the power circuitry.
  
  Serial.println("Initialization Complete");

  digitalWrite(RELAIS_POWER,HIGH); // Relais is switched on -> Powr-Shield has power + arm moves to "custom" positon

}

// 5) START READOUT
void loop() {

// The robot waits on a serial command to get to new position
// TAKE CARE TO JUST SEND RIGHT COMMANDS! THERE IS NOW ADDITIONAL SAFETY/REASONABLENESS CHECK HERE!

  while (Serial.available()) {
    arm.safeDelay(10);            // Delay without interrupting PWM for Servo Drivers

    // Radout of serial line
    if (Serial.available() > 0) {
      char c = Serial.read();
      readString += c;
    }

    update = true;
  }

  /*
Transmitted Char-Array/String:
Angle 1: xxx Base | Angle 2: xxx Schulter | Angle 3: xxx Ellbogen | Angle 4: xxx Wrist

Type: Sign + 3 digits -> 4digits per angle
composition of one command:
x x x x  ,  x x x x  ,  x  x  x  x   ,   x  x  x  x
0 1 2 3  4  5 6 7 8  9  10 11 12 13  14  15 16 17 18
*/


  if (update == true) {
    // Readout individual angles + convert to integer
    digitalWrite(STS_LED,HIGH); // LED on = Moving started
    String Str_alpha_Base = readString.substring(0, 4);
    String Str_alpha_Shoulder = readString.substring(5, 9);
    String Str_alpha_Ellbow = readString.substring(10, 14);
    String Str_alpha_Wrist = readString.substring(15, 19);

    alpha_Base = Str_alpha_Base.toInt();
    alpha_Shoulder = Str_alpha_Shoulder.toInt();
    alpha_Ellbow = Str_alpha_Ellbow.toInt();
    alpha_Wrist = Str_alpha_Wrist.toInt();

    // Move Robot
    setAngles(alpha_Base, alpha_Shoulder, alpha_Ellbow, alpha_Wrist);

    //clear variables
    readString = "";
    update = false;
    digitalWrite(STS_LED,LOW);  // LED off = Moving over
    
    // Delay is okay, because the Matlab Script is waiting for 4sec anyway
    // The Steppers move sometimes arbitory after the set command -> Try to lowpass this moves
    delay(1000); 
    
    // Evaluating Base Angle using extra Sensor + send the measured value via Serial Command    
    alpha_Base_meas = evaluatePoti_Mean();
    alpha_Base_meas_Str = String(alpha_Base_meas,2);
    Serial.println("...Braccio Angle. Base = " + alpha_Base_meas_Str + "| Shoulder = " + Str_alpha_Shoulder + "| Ellbow = " + Str_alpha_Ellbow + "| Wrist = " + Str_alpha_Wrist);

  }
}


// 6) FUNCTIONS

void setAngles(int alpha1, int alpha2, int alpha3, int alpha4) {
  
  // Check, if the angles can be set due to limits of system
  bool update_angles = true;
  if (alpha1 < base_min) {
    update_angles = false;
  }
  if (alpha1 > base_max) {
    update_angles = false;
  }
  if (alpha2 < shoulder_min) {
    update_angles = false;
  }
  if (alpha2 > shoulder_max) {
    update_angles = false;
  }
  if (alpha3 < ellbow_min) {
    update_angles = false;
  }
  if (alpha3 > ellbow_max) {
    update_angles = false;
  }
  if (alpha4 < wrist_min) {
    update_angles = false;
  }
  if (alpha4 > wrist_max) {
    update_angles = false;
  }


  // Set Angles
  if (update_angles == false) {
    Serial.println("...Cancel due to limit");

  } else {

    arm.setAllNowSynchron(alpha1, alpha2, alpha3, alpha4, WRIST_ROTATION, GRIPPER_POS);
  }
}



double evaluatePoti() {
  uout_x_raw = analogRead(PotiPin);
  uout_x = (double)uout_x_raw * (Uref_ADC / 1024.0);

  /*
  // OPTION 1: USING CALIBRATED VALUES
  // is creating more precise results
  */

  alpha_meas = alphaBase_Lin_a1 * uout_x + alphaBase_Lin_a2;
  return alpha_meas;

  /*
  // OPTION 2: USING THE CORRELATION OF THE DESIGN
  // is less precise (because the effective electrical length of the potentiometer is somehow tricky to measure)
  */

  /*
  Ucc = (double)analogRead(UccPin) * (Uref_ADC / 1024.0) * 2;
  k = Ucc / lpoti;
  x = uout_x/k;
  x_s = x0-x;
  alpha_meas_rad = asin(x_s / l);
  alpha_meas = alpha_meas_rad * RAD_TO_DEG;
  return alpha_meas;

  //In case a check is necessairy:
  //Serial.println("uout_x = " + String(uout_x) + "V(Raw=" + String(uout_x_raw) + "), x = " + String(x) + "mm, x_s = " + String(x_s) + "mm, alpha_meas_rad = " + String(alpha_meas_rad) + "rad, alpha_meas = " + String(alpha_meas) + "°");

  */



  
}

double evaluatePoti_Mean(){
  // Readout the Potentiometer & arith average them (Low Pass Filter of Data)

  double alpha_meas_temp =0.0;
  double alpha_meas_sum =0.0;

  int num_avgs = 10; // 10 Averages are selected

  for(int i_avg =0;i_avg<num_avgs;i_avg++){
    alpha_meas_temp = evaluatePoti();
    alpha_meas_sum=alpha_meas_sum+alpha_meas_temp;
    delay(10);
  }

alpha_meas_sum = alpha_meas_sum/((double)num_avgs);

return alpha_meas_sum;
}










