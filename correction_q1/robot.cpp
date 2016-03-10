#include "robot.h"
#include <cmath>

// Initialize a robot with the state <x0,y0,v0,hdg0>
Robot::Robot(double x0, double y0, double v0, double hdg0) :
  x(x0),y(y0),v(v0),hdg(hdg0) {};

Robot::Robot() : x(0),y(0),v(0),hdg(0) {};

// Update the robot state with the state equation
void Robot::clock(double u1, double u2, double dt){
  x += dt*v*std::cos(hdg);
  y += dt*v*std::sin(hdg);
  hdg += dt*u1;
  v += dt*u2;
};

void Robot::draw(){
    vibes::drawAUV(x,y,hdg*180.0/M_PI, 1.5, "[r]");
};
