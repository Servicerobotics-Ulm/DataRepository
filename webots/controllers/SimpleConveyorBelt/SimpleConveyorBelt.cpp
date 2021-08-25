// reads motor names from controllerArgs
// reads position from customData, sets motors position

// another Supervisor controller can set this position:
// webots::Supervisor *robot = new webots::Supervisor();
// double position = ...;
// robot -> getFromDef("ConveyorBeltRobot") -> getField("customData") -> setSFString(to_string(position));

// write position - getValue() into DEF CommObject field posDif

#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <math.h>

using namespace webots;
int main(int argc, char **argv) {
  Supervisor *robot = new Supervisor();
  int timeStep = (int)robot->getBasicTimeStep();
  Motor *motor = robot->getMotor("belt_motor");
  DistanceSensor *sensor = robot->getDistanceSensor("isBoxPresent");
  DistanceSensor *sensorStart = robot->getDistanceSensor("isBoxPresentStart");
  if(!motor || !sensor) {
    std::cerr << "no motor " << motor << " or sensor " << sensor << " found" << std::endl;
    return -1;
  }
  Node* commObject = robot->getFromDef("CommObject");
  if(!commObject) {
    std::cerr << "no DEF CommObject CommObject found" << std::endl;
    return -1;
  }
  Field* field = commObject->getField("isBoxPresent");
  if(!field) {
    std::cerr << "no isBoxPresent field in CommObject found" << std::endl;
    return -1;
  }
  sensor->enable(timeStep);
  sensorStart->enable(timeStep);
  motor->setPosition(INFINITY);
  while (robot->step(timeStep) != -1) {
    double getVal = sensor->getValue();
    bool isBoxPresent = getVal<0.99;
    motor->setVelocity(isBoxPresent ? 0.0 : 0.1);
    field->setSFBool(isBoxPresent);
    commObject->getField("isBoxPresentStart")->setSFBool(sensorStart->getValue()<0.99);
  }
  delete robot;
  return 0;
}
