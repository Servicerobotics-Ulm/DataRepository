// File:          Larry_new.cpp
// Date:          13.01.2021
// Description:   
// Author:        Wenzheng Cai

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Lidar.hpp>

#define Wheel_Radius 0.2
#define Wheel_Space 0.465

using namespace webots;
using namespace std;

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  Motor *motors[4];
  char motors_names[4][12] = {"LeftWheel", "RightWheel"}; 
  for (int i = 0; i < 2; i++) {
    motors[i] = robot->getMotor(motors_names[i]);
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0.0);
    motors[i]->setAcceleration(5);
  }
  
  /*Lidar *lidar_front;
  lidar_front = robot->getLidar("Sick LMS 100_Front");
  lidar_front ->enable(timeStep);
  lidar_front ->enablePointCloud(); */
  
  Keyboard *kb;
  kb = robot->getKeyboard();
  kb->enable(timeStep);

  cout << "Use keyboard to control the robot Larry:" << endl;
  cout << "↑ go forward, ↓ go back" << endl;
  cout << "← turn left, → turn right" << endl;
  
  float leftSpeed = 0.0;
  float rightSpeed = 0.0;

  // Main loop:
  while (robot->step(timeStep) != -1) {
  
    int key=kb->getKey();
    
    if(key==kb->UP) {
      leftSpeed = 1.0;
      rightSpeed = 1.0;
    }
    else if(key==kb->DOWN) {
      leftSpeed = -1.0;
      rightSpeed = -1.0;
    }
    else if(key==kb->LEFT) {
      leftSpeed = -1.0;
      rightSpeed = 1.0;
    }
    else if(key==kb->RIGHT) {
      leftSpeed = 1.0;
      rightSpeed = -1.0;
    }
    else {
      leftSpeed = 0.0;
      rightSpeed = 0.0;
    }
    motors[0]->setVelocity(leftSpeed*3);
    motors[1]->setVelocity(rightSpeed*3);
    
  };

  delete robot;
  return 0;
}
