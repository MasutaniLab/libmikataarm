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
    for (int i = 0; i < NumJoints; i++) {
      cout << "joints[" << i << "] (deg): ";
      double d;
      cin >> d;
      joints[i] = d*M_PI / 180;
    }
    Matrix44 m = forward_kinematics(joints);
    vector<double> solved = inverse_kinematics(m);
    cout << "Joints:";
    for (int i = 0; i < NumJoints; i++) {
      cout << setw(10) << joints[i] * 180 / M_PI;
    }
    cout << endl;
    cout << "Solved:";
    for (int i = 0; i < NumJoints; i++) {
      cout << setw(10) << solved[i] * 180 / M_PI;
    }
    cout << endl;
    cout << "Delta: ";
    for (int i = 0; i < NumJoints; i++) {
      cout << setw(10) << (solved[i]-joints[i]) * 180 / M_PI;
    }
    cout << endl;
    cout << "------------------------------------------------------------" << endl;
  }
  return 0;
}
