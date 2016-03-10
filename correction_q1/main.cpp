#include <iostream>
#include "vibes.h"
#include "interval.h"
#include  <cmath>

#include "unistd.h"

#include "robot.h"
#include "controller.h"

using namespace std;

double dt = 0.005;
//typedef std::array<double,4> State;




int main(){

  Robot R(10,0,1,1);
  Controller C;


  vibes::beginDrawing();
  vibes::newFigure("Simulation");
  vibes::setFigureProperties(vibesParams("x",0,"y",40,"width",500,"height",500));
  vibes::axisLimits(-12, 12, -12, 12);
  for (double t = 0; t < 10; t+= dt){
      R.clock(C.u1, C.u2, dt);
      C.clock(R, t);
      vibes::clearFigure();
      R.draw();
      vibes::drawCircle(10*std::cos(t),10*std::sin(3*t),0.1, "[cyan]");
      usleep(40000);
  }

  vibes::endDrawing();
}
