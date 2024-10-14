/*
 BraccioV2slowed.cpp - version 0.1
 Written by Pavel Povolni
 Based upon the BraccioV2 library by  Lukas Severinghaus
 originally licensed under GNU GPL V1.2.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */



#include "BraccioV2slowed.h"
Braccio::Braccio() {
}

void Braccio::begin() {
  _initializeServos(true);


}
void Braccio::begin(bool defaultPos) {
  _initializeServos(defaultPos);
}
//Initializes all servos by attaching and optionally moving them all
void Braccio::_initializeServos(bool defaultPos) {
  pinMode(SOFT_START_PIN, OUTPUT);
  digitalWrite(SOFT_START_PIN, LOW);
  _base.attach(_BASE_ROT_PIN);
  _shoulder.attach(_SHOULDER_PIN);
  _elbow.attach(_ELBOW_PIN);
  _wrist_rot.attach(_WRIST_ROT_PIN);
  _wrist.attach(_WRIST_PIN);
  _gripper.attach(_GRIPPER_PIN);
  if (defaultPos) {
    setAllNow(_jointCenter[BASE_ROT], _jointCenter[SHOULDER], _jointCenter[ELBOW],
              _jointCenter[WRIST], _jointCenter[WRIST_ROT], _jointCenter[GRIPPER]);

  }
  _softStart();
}
//Startup helper function
void Braccio::_softwarePWM(int high_time, int low_time) {
  digitalWrite(SOFT_START_PIN, HIGH);
  delayMicroseconds(high_time);
  digitalWrite(SOFT_START_PIN, LOW);
  delayMicroseconds(low_time);
}

/*
  This function, used only with the Braccio Shield V4 and greater,
  turn ON the Braccio softly and save it from brokes.
  The SOFT_START_CONTROL_PIN is used as a software PWM
  @param soft_start_level: the minimum value is -70, default value is 0 (SOFT_START_DEFAULT)
*/
void Braccio::_softStart() {
  long int tmp = millis();
  while (millis() - tmp < 2000)
    _softwarePWM(80, 450);   //the sum should be 530usec

  while (millis() - tmp < 6000)
    _softwarePWM(75, 430); //the sum should be 505usec

  digitalWrite(SOFT_START_PIN, HIGH);
}

/*
   Sets one joint to absolute target position. Constrains value to min/max limits,
   returns true if value was not constrained, false if it was constrained to the limits.
*/
bool Braccio::setOneAbsolute(int joint, int value) {
  int out = constrain(value, _jointMin[joint], _jointMax[joint]);
  _targetJointPositions[joint] = out;
  return joint == out;
}

/*
   Sets one joint to a target position relative to current target position. Constrains
   value to min/max limits, returns true if value was not constrained, false if it was
   constrained to limits.
*/
bool Braccio::setOneRelative(int joint, int value) {
  int currentPos = _targetJointPositions[joint];
  int rawPos = currentPos + value;
  int actualPos = constrain(rawPos, _jointMin[joint], _jointMax[joint]);
  _targetJointPositions[joint] = actualPos;
  return rawPos == actualPos;
}

/*
   Sets all joints to absolute target positions. Returns true if none of the values
   were constrained, false if at least one of the values was constrained to limits.
*/
bool Braccio::setAllAbsolute(int b, int s, int e, int w, int w_r, int g) {
  boolean out = true;
  out = out & setOneAbsolute(BASE_ROT, b);
  out = out & setOneAbsolute(SHOULDER, s);
  out = out & setOneAbsolute(ELBOW, e);
  out = out & setOneAbsolute(WRIST, w);
  out = out & setOneAbsolute(WRIST_ROT, w_r);
  out = out & setOneAbsolute(GRIPPER, g);
  return out;
}

/*
   Sets all joints to relative target positions. Returns true if none of the values
   were constrained, false if at least one of the values was constrained to limits.
*/
bool Braccio::setAllRelative(int b, int s, int e, int w, int w_r, int g) {
  boolean out = true;
  out = out & setOneRelative(BASE_ROT, b);
  out = out & setOneRelative(SHOULDER, s);
  out = out & setOneRelative(ELBOW, e);
  out = out & setOneRelative(WRIST, w);
  out = out & setOneRelative(WRIST_ROT, w_r);
  out = out & setOneRelative(GRIPPER, g);
  return out;
}
//Sets maximum command value of given joint
void Braccio::setJointMax(int joint, int value) {
  _jointMax[joint] = constrain(value, GLOBAL_MIN, GLOBAL_MAX);
}
//Sets minimum command value of given joint
void Braccio::setJointMin(int joint, int value) {
  _jointMin[joint] = constrain(value, GLOBAL_MIN, GLOBAL_MAX);
}
//Set the center point of given joint
void Braccio::setJointCenter(int joint, int offset) {
  _jointCenter[joint] = constrain(offset, GLOBAL_MIN, GLOBAL_MAX);
}

/*
   StartUp with a softstart. Sets all joints to absolute position immediately. Does not constrain values.
*/
void Braccio::setAllNowSoftStartUp(int b, int s, int e, int w, int w_r, int g) {
  _setServo(BASE_ROT, b, true);
  _setServo(SHOULDER, s, true);
  _setServo(ELBOW, e, true);
  _setServo(WRIST, w, true);
  _setServo(WRIST_ROT, w_r, true);
  _setServo(GRIPPER, g, true);
  _softStart();
}


/*
   Sets all joints to absolute position immediately. Does not constrain values.
*/
void Braccio::setAllNow(int b, int s, int e, int w, int w_r, int g) {
  _setServo(BASE_ROT, b, true);
  _setServo(SHOULDER, s, true);
  _setServo(ELBOW, e, true);
  _setServo(WRIST, w, true);
  _setServo(WRIST_ROT, w_r, true);
  _setServo(GRIPPER, g, true);
}
//Sets the speed of a given joint, defaults to 1
void Braccio::setDelta(int joint, int value) {
  _jointDelta[joint] = value;
}

/*
   Sets all joints to absolute position in a slow way. 
   All motors are turned at the same time till their final position is reached. Does not constrain values.
*/
void Braccio::setAllNowSynchron(int b, int s, int e, int w, int w_r, int g) {

  int oldpos_b_sync = _currentJointPositions[BASE_ROT];
  int oldpos_s_sync = _currentJointPositions[SHOULDER];
  int oldpos_e_sync = _currentJointPositions[ELBOW];
  int oldpos_w_sync = _currentJointPositions[WRIST];

  bool b_inc = false;
  bool s_inc = false;
  bool e_inc = false;
  bool w_inc = false;

  if (b-oldpos_b_sync>0){ //increasing
    b_inc = true;
  }
  if (s-oldpos_s_sync>0){ //increasing
    s_inc = true;
  }
  if (e-oldpos_e_sync>0){ //increasing
    e_inc = true;
  }
  if (w-oldpos_w_sync>0){ //increasing
    w_inc = true;
  }


  int dif_b = abs(b-oldpos_b_sync);
  int dif_s = abs(s-oldpos_s_sync);
  int dif_e = abs(e-oldpos_e_sync);
  int dif_w = abs(w-oldpos_w_sync);

  // Find Maximum
  int max = dif_b;
  if (dif_s>max){
    max = dif_s;
  }
  if (dif_e>max){
    max = dif_e;
  }
  if (dif_w>max){
    max = dif_w;
  }

  // Make one step per Axis at the same time
  for(int i=0; i<max;i++)
  {
    // Move Basis
    if (dif_b > 0){

      if (b_inc==true){  // Increasing Angle
        oldpos_b_sync +=1;           
      }
      else{ // Decreasing Angle
        oldpos_b_sync -=1;
      }
      _setServo(BASE_ROT,oldpos_b_sync, true);    
      dif_b -=1;
    }
    // Move Shoulder
      if (dif_s > 0){

      if (s_inc==true){  // Increasing Angle
        oldpos_s_sync +=1;           
      }
      else{ // Decreasing Angle
        oldpos_s_sync -=1;
      }
      _setServo(SHOULDER,oldpos_s_sync, true);    
      dif_s -=1;
    }
    // Move Elbow
    if (dif_e > 0){

      if (e_inc==true){  // Increasing Angle
        oldpos_e_sync +=1;           
      }
      else{ // Decreasing Angle
        oldpos_e_sync -=1;
      }
      _setServo(ELBOW,oldpos_e_sync, true);    
      dif_e -=1;
    }
    // Move Wrist
    if (dif_w > 0){

      if (w_inc==true){  // Increasing Angle
        oldpos_w_sync +=1;           
      }
      else{ // Decreasing Angle
      oldpos_w_sync -=1;
      }
      _setServo(WRIST,oldpos_w_sync, true);    
      dif_w -=1;
    }
  }  

  _setServo(WRIST_ROT, w_r, true);
  _setServo(GRIPPER, g, true);

}



//Sets a given joint to a specific position
void Braccio::_setServo(int joint, int value, bool updateTarget) {

  int delay_time = 15; //Microseconds, how long to wait before next movement
  int oldpos = 0;
  
  switch (joint) {
    case BASE_ROT:
     
      oldpos = _currentJointPositions[BASE_ROT];
      if (value>oldpos){
        for (int angle = oldpos; angle<=value;angle+=1){
          _base.write(angle);
          delay(delay_time);
        }
      }
      else if (value<oldpos) {
        for (int angle = oldpos; angle>=value;angle -=1){
          _base.write(angle);
          delay(delay_time);
        }
      }
      else
      {
        _base.write(value);
      }      
      _currentJointPositions[BASE_ROT] = value;
      
      if (updateTarget) {
        _targetJointPositions[BASE_ROT] = value;
      }
      break;
    case SHOULDER:
      oldpos = _currentJointPositions[SHOULDER];
      if (value>oldpos){
        for (int angle = oldpos; angle<=value;angle+=1){
          _shoulder.write(angle);
          delay(delay_time);
        }
      }
      else if (value<oldpos) {
        for (int angle = oldpos; angle>=value;angle -=1){
          _shoulder.write(angle);
          delay(delay_time);
        }
      }
      else
      {
        _shoulder.write(value);
      }   

      _currentJointPositions[SHOULDER] = value;
      if (updateTarget) {
        _targetJointPositions[SHOULDER] = value;
      }
      break;
    case ELBOW:
      oldpos = _currentJointPositions[ELBOW];

      if (value>oldpos){
        for (int angle = oldpos; angle<=value;angle+=1){
          _elbow.write(angle);
          delay(delay_time);
        }
      }
      else if (value<oldpos) {
        for (int angle = oldpos; angle>=value;angle -=1){
          _elbow.write(angle);
          delay(delay_time);
        }
      }
      else
      {
        _elbow.write(value);
      }  

      _currentJointPositions[ELBOW] = value;
      if (updateTarget) {
        _targetJointPositions[ELBOW] = value;
      }
      break;
    case WRIST:
      oldpos = _currentJointPositions[WRIST];

      if (value>oldpos){
        for (int angle = oldpos; angle<=value;angle+=1){
          _wrist.write(angle);
          delay(delay_time);
        }
      }
      else if (value<oldpos) {
        for (int angle = oldpos; angle>=value;angle -=1){
          _wrist.write(angle);
          delay(delay_time);
        }
      }
      else
      {
        _wrist.write(value);
      }  
      _currentJointPositions[WRIST] = value;
      if (updateTarget) {
        _targetJointPositions[WRIST] = value;
      }
      break;
    case WRIST_ROT:    

      _wrist_rot.write(value);
      _currentJointPositions[WRIST_ROT] = value;
      if (updateTarget) {
        _targetJointPositions[WRIST_ROT] = value;
      }
      break;
    case GRIPPER:
      _gripper.write(value);
      _currentJointPositions[GRIPPER] = value;
      if (updateTarget) {
        _targetJointPositions[GRIPPER] = value;
      }
      break;

  }
}
//Processes servo movement request
void Braccio::_moveServo(int joint) {
  int currentPos = _currentJointPositions[joint];
  int targetPos = _targetJointPositions[joint];
  if (currentPos != targetPos) {
    int dir = (currentPos <= targetPos) ? 1 : -1;
    int delta = _jointDelta[joint];
    int dirDelta = dir * delta;
    int newPos = currentPos + dirDelta;
    _setServo(joint, newPos, false);
  }
}
//Delays for ms with movement updates every t ms
void Braccio::safeDelay(int ms, int t){
  long currentTime = millis();
  while(millis() < currentTime + ms){
    update();
    delay(t);
  }
}

//Delays with movement updates every 10ms
void Braccio::safeDelay(int ms){
  safeDelay(ms, 10);
}
//Returns the calibrated center point of the given joint
int Braccio::getCenter(int joint){
  return _jointCenter[joint];
}
//Processes movement of each joint to achieve target endpoint
void Braccio::update() {
  _moveServo(BASE_ROT);
  _moveServo(SHOULDER);
  _moveServo(ELBOW);
  _moveServo(WRIST);
  _moveServo(WRIST_ROT);
  _moveServo(GRIPPER);

}
