#include <math.h>
#include <iostream>
#include <iomanip>
#include "kinematics.h"
using namespace std;

const int NumJoints = 4;

int main(void) {
  cout << fixed << setprecision(4);
  vector<double> joints(NumJoints);
  while (true) {
    double x, y, z, pitch;
    cout << "x [mm]: ";
    cin >> x;
    cout << "y [mm]: ";
    cin >> y;
    cout << "z [mm]: ";
    cin >> z;
    cout << "pitch [deg]: ";
    double d;
    cin >> d;
    pitch = d*M_PI / 180;
    Matrix44 m = xyzpToMatrix44(x, y, z, pitch);
    cout << str(m) << endl;
    vector<double> solved = inverse_kinematics(m);
    for (int i = 0; i < NumJoints; i++) {
      cout << setw(10) << solved[i] * 180 / M_PI;
    }
    cout << endl;
    cout << "------------------------------------------------------------" << endl;
  }
  return 0;
}
