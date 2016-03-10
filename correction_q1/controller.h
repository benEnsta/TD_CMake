#ifndef __CONTROLLER__
#define __CONTROLLER_

#include "robot.h"

class Controller{
public:
  Controller();
  void clock(Robot& R, double t);
  //commande computed by the controler
  double u1, u2;
};


#endif //__CONTROLLER__
