// File:          pedestrian_new.cpp
// Date:          20.05.2021
// Description:
// Author:        Wenzheng Cai, Technische Hochschule Ulm
// Modifications:

#include <iostream>
#include <math.h>

#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Supervisor.hpp>

using namespace webots;
using namespace std;

int main () {
// int main (int argc, char *argv[]) {

  Supervisor *supervisor = new Supervisor();

  // get the parameters of the current world.
  int timeStep = (int)supervisor->getBasicTimeStep()*2;
 
  const int bodyPartsNumber = 13;
  const int walkSequencesNumber = 8;
  const double rootHeight = 1.27;
  const double cycleToDistanceRatio = 0.22;
  double speed = 0.8;  // m/s
  double turnSpeed = 1.0;  // radians/s
  double currentHeightOffset = 0;
  const string jointNames[13] = {
                                   "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
                                   "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
                                   "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
                                   "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
                                   "headAngle"
                                 };
  // move body parts up/down each step
  const double heightOffsets[8] = {  // those coefficients are empirical coefficients which result in a realistic walking gait
                                    -0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03
                                   };
  const double angles[13][8] = {  // those coefficients are empirical coefficients which result in a realistic walking gait
                                {-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74},  // left arm
                                {0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21},  // left lower arm
                                {0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0},  // left hand
                                {0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7},  // right arm
                                {-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38},  // right lower arm
                                {0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2},  // right hand
                                {-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4},  // left leg
                                {1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26},  // left lower leg
                                {0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07},  // left foot
                                {-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7},  // right leg
                                {0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49},  // right lower leg
                                {0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36},  // right foot
                                {0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09}  // head
                               };
  
  Node *humanNode = supervisor->getSelf();
  Field *humanTranslationField = humanNode->getField("translation");
  Field *humanRotationField = humanNode->getField("rotation");

  Field * joints_position_field[bodyPartsNumber];
  for (int i = 0; i < bodyPartsNumber; i++)
    joints_position_field[i] = supervisor->getSelf()->getField(jointNames[i]);
  Keyboard *kb;
  kb = supervisor->getKeyboard();
  kb->enable(timeStep);
  
  double walkingDistance = 0.0;
  
  // Main loop:
  while (supervisor->step(timeStep) != -1) {
    
    const double *humanTranslationValues = humanTranslationField->getSFVec3f();
    const double *humanRotationValues = humanRotationField->getSFRotation();
    
    double px = humanTranslationValues[0]; // actual x coordinate in world
    double py = humanTranslationValues[2]; // actual y coordinate in world
    double heading = humanRotationValues[3];  // actual heading in world
     
    double forward = 0.0;
    double turn = 0.0;
    int key;
    while( (key = kb->getKey()) != -1) {

      if(key==kb->UP || key=='W')
        forward = 1.0;
      if(key==kb->DOWN || key=='S')
        forward = -1.0;
      if(key==kb->LEFT || key=='A')
        turn = 1.0;
      if(key==kb->RIGHT || key=='D')
        turn = -1.0;    
    }
      
    forward *= speed * (timeStep / 1000.0);
    walkingDistance += forward;
    px += sin(heading)*forward;
    py += cos(heading)*forward;
    turn *= turnSpeed * (timeStep / 1000.0);
    heading += turn;

    double ratio = walkingDistance / cycleToDistanceRatio;
    int currentSequence = ((int)(floor(ratio))) % walkSequencesNumber;
    if(currentSequence < 0)
      currentSequence += walkSequencesNumber;
    ratio -= floor(ratio);
//    cout << "ratio: " << ratio << " " << currentSequence << " " << walkingDistance / cycleToDistanceRatio << endl;
    for (int i = 0; i < bodyPartsNumber; i++) {
      float currentAngle = angles[i][currentSequence] * (1 - ratio) + angles[i][(currentSequence + 1) % walkSequencesNumber] * ratio;
      if(forward == 0.0)
         currentAngle = 0.0;
      joints_position_field[i] ->setSFFloat(currentAngle);
//      supervisor->getSelf()->getField(jointNames[i])->setSFFloat(currentAngle);
//      cout << supervisor->getSelf()->getField(jointNames[i])->getSFFloat() << endl;
    }
    
    currentHeightOffset = heightOffsets[currentSequence] * (1 - ratio) + heightOffsets[(currentSequence + 1) % walkSequencesNumber] * ratio;


    const double newTranslationValues[3] = {px, rootHeight+currentHeightOffset, py};          
    
    humanTranslationField->setSFVec3f(newTranslationValues);
    
    const double newRotationValues[4] = {0, 1, 0, heading};    
    humanRotationField->setSFRotation(newRotationValues);
//    cout<< "I am here: " << newTranslationValues[0] << " " << newTranslationValues[2] << " " << newRotationValues[3] << endl;    
  }     
   
  delete supervisor;
  return 0;
}
