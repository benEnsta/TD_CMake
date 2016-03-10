#ifndef __ROBOT__
#define __ROBOT__

#include "vibes.h"

class Robot {
public:

  // Initialize a robot with the state <x0,y0,v0,hdg0>
  Robot(double x0, double y0, double v0, double hdg0);
  Robot();

  // Update the robot state with the state equation
  void clock(double u1, double u2, double dt);

  void draw();

  // x,y position
  // v,hdg : speed and heading of the robot
  double x, y, v, hdg;

};


#endif //__ROBOT__
