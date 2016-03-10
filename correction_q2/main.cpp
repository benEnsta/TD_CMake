#include <iostream>
#include "vibes.h"
#include "interval.h"
#include  <cmath>

#include "unistd.h"


using namespace std;

double dt = 0.005;
//typedef std::array<double,4> State;

class Robot {
public:

  // Initialize a robot with the state <x0,y0,v0,hdg0>
  Robot(double x0, double y0, double v0, double hdg0) : 
    x(x0),y(y0),v(v0),hdg(hdg0) {};
  
  Robot() : x(0),y(0),v(0),hdg(0) {};

  // Update the robot state with the state equation
  void clock(double u1, double u2){
    x += dt*v*cos(hdg);
    y += dt*v*sin(hdg);
    hdg += dt*u1;
    v += dt*u2;
  };

  void draw(){
      vibes::drawAUV(x,y,hdg*180.0/M_PI, 1.5, "[r]");
  };

  // x,y position 
  // v,hdg : speed and heading of the robot
  double x, y, v, hdg;
  double x_hat, y_hat;

};

int sign(const double& x){
    return (x >0) ? 1 : -1;
}

class Controller{
public:
  Controller(): u1(0),u2(0){};

  void clock(Robot& R, double t){
    double wx = 10*cos(t);
    double wy = 10*sin(3*t);

    double dwx = -10*sin(t);
    double dwy = 30*cos(3*t);

    double dx = R.v*cos(R.hdg);
    double dy = R.v*sin(R.hdg);
    double ddyx = 100*sign(wx - R.x_hat + dwx - dx);
    double ddyy = 100*sign(wy - R.y_hat + dwy - dy);

    double a11 = -R.v*sin(R.hdg);
    double a12 = cos(R.hdg);
    double a21 = R.v*cos(R.hdg);
    double a22 = sin(R.hdg);

    u1 = ( a22 * ddyx  - a12*ddyy ) / (a11*a22-a12*a21);
    u2 = ( -a21* ddyx  +a11*ddyy) / (a11*a22-a12*a21);

  };

  //commande computed by the controler
  double u1, u2;
};

class Estimator{
public:
   void estimate(Robot& R, interval& x, interval& y){
       double m_x1 = 15;
       double m_y1 = 10;
       double m_x2 = -15;
       double m_y2 = 10;
       double m_x3 = -15;
       double m_y3 = -10;
       x = x + dt*(R.v*cos(R.hdg)) + interval(-0.2, 0.2);
       y = y + dt*(R.v*sin(R.hdg)) + interval(-0.2, 0.2);

       interval d1 =  std::sqrt(std::pow(m_x1 - R.x, 2) + std::pow(m_y1 - R.y, 2)) + interval(-0.1,0.1);
       interval d2 =  std::sqrt(std::pow(m_x2 - R.x, 2) + std::pow(m_y2 - R.y, 2)) + interval(-0.1,0.1);
       interval d3 =  std::sqrt(std::pow(m_x3 - R.x, 2) + std::pow(m_y3 - R.y, 2)) + interval(-0.1,0.1);



       for (int i = 0; i < 3; i++){
           CDist(x,y,m_x1, m_y1, d1);
           CDist(x,y,m_x2, m_y2, d2);
           CDist(x,y,m_x3, m_y3, d3);
       }
   };
};





int main(){
  Robot R(10,0,1,1);
  interval x = 10 + interval(-2, 2);
  interval y = 0 + interval(-2, 2);

  Controller C;
  Estimator E;

  vibes::beginDrawing();
  vibes::newFigure("Simulation");
  vibes::setFigureProperties(vibesParams("x",0,"y",40,"width",500,"height",500));
  vibes::axisLimits(-12, 12, -12, 12);
  for (double t = 0; t < 10; t+= dt){

      R.clock(C.u1, C.u2);
      C.clock(R, t);
      E.estimate(R, x, y);
      R.x_hat = Center(x);
      R.y_hat = Center(y);
//      cerr << x << " " << y << "\n";
      vibes::clearFigure();
      R.draw();
      vibes::drawBox(x.inf, x.sup, y.inf, y.sup, "g");
      vibes::drawCircle(10*cos(t),10*sin(3*t),0.1, "[cyan]");
      usleep(40000);
  }

  vibes::endDrawing();
}
