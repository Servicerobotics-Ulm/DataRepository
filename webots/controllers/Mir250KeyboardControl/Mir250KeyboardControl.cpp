#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>

using namespace webots;

const double wheelRadius = 0.1;
const double distanceWheelToRobotCenter = 0.2;
const double max_acc_x = 0.5;
const double max_acc_rot = 0.5;
double last_v_x = 0.0;
double last_v_rot = 0.0;

Motor* motor[2];
int timeStep;

void limit_value(double &value, double limit) {
  if(value > limit)
    value = limit;
  if(value < -limit)
    value = -limit;
}

// v_x is forward speed in m/s
// v_rot is counterclockwise rotational speed in radians/s
void setSpeed(double v_x, double v_rot) {
  double t = timeStep/1000.0;
  double acc_x = (v_x - last_v_x) / t;
  double acc_rot = (v_rot - last_v_rot) / t;
  limit_value(acc_x, max_acc_x);
  limit_value(acc_rot, max_acc_rot);
  last_v_x += acc_x * t;
  last_v_rot += acc_rot * t;
  motor[0]->setVelocity(last_v_x/wheelRadius - last_v_rot*distanceWheelToRobotCenter/wheelRadius);
  motor[1]->setVelocity(last_v_x/wheelRadius + last_v_rot*distanceWheelToRobotCenter/wheelRadius);  
}

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  timeStep = (int)robot->getBasicTimeStep();
  Keyboard *keyboard = robot->getKeyboard();
  keyboard->enable(timeStep);
  for(int i=0; i<2; i++) {
    motor[i] = robot->getMotor( i ? "RightWheelRotationalMotor" : "LeftWheelRotationalMotor");
    motor[i]->setPosition(INFINITY);
    motor[i]->setAcceleration(-1);
  }
  while (robot->step(timeStep) != -1) {
    double v_x = 0.0, v_rot = 0.0;
    int key;
    while ((key = keyboard->getKey()) != -1) {
      // if both 'w' and 'up' is pressed at same time, speed is doubled
      if (key == 'W' || key == keyboard->UP)
          v_x += 1.0;
      if (key == 'S' || key == keyboard->DOWN)
          v_x -= 1.0;
      if (key == 'A' || key == keyboard->LEFT)
          v_rot += 1.0;
      if (key == 'D' || key == keyboard->RIGHT)
          v_rot -= 1.0;
    }
    setSpeed(v_x, v_rot);
  }
  delete robot;
  return 0;
}

