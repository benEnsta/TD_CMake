#include "controller.h"

#include <cmath>
using std::cos;
using std::sin;

int sign(const double& x){
    return (x >0) ? 1 : -1;
}

Controller::Controller(): u1(0),u2(0){}

void Controller::clock(Robot& R, double t){
    double wx = 10*cos(t);
    double wy = 10*sin(3*t);

    double dwx = -10*sin(t);
    double dwy = 30*cos(3*t);

    double dx = R.v*cos(R.hdg);
    double dy = R.v*sin(R.hdg);
    double ddyx = 100*sign(wx - R.x + dwx - dx);
    double ddyy = 100*sign(wy - R.y + dwy - dy);

    double a11 = -R.v*sin(R.hdg);
    double a12 = cos(R.hdg);
    double a21 = R.v*cos(R.hdg);
    double a22 = sin(R.hdg);

    u1 = ( a22 * ddyx  - a12*ddyy ) / (a11*a22-a12*a21);
    u2 = ( -a21* ddyx  +a11*ddyy) / (a11*a22-a12*a21);

}
