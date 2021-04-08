#include <webots/Robot.hpp>
#include <webots/Connector.hpp>
#include <webots/Keyboard.hpp>

using namespace webots;
int main(int argc, char **argv) {
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();
  Connector* c = robot->getConnector("SmallBox0");
  c -> enablePresence(timeStep);
  while (robot->step(timeStep) != -1) {
  }
  delete robot;
  return 0;
}
