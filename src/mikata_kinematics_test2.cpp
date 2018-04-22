
#include <iostream>
#include "mikata.h"
#include "kinematics.h"
using namespace std;
using namespace ssr::mikata;

int main(const int argc, const char* argv[]) {
  if (argc < 3) {
    std::cout << "Invalid Usage." << std::endl;
    std::cout << "USAGE: $./test filename baudrate" << std::endl;
    return -1;
  }

  try {
    MikataArm arm(argv[1], atoi(argv[2]));
    arm.servoOn(true);
    arm.goHome();
    while (true) {
      double x, y, z, pitch;
      cout << "x [mm]: ";
      cin >> x;
      cout << "y [mm]: ";
      cin >> y;
      cout << "z [mm]: ";
      cin >> z;
      cout << "pitch [deg]: ";
      double deg;
      cin >> deg;
      pitch = deg*M_PI / 180;
      Matrix44 m = xyzpToMatrix44(x, y, z, pitch);
      cout << str(m) << endl;
      vector<double> solved = inverse_kinematics(m);
      vector<JointCommand> jcs;
      for (int i = 0; i < numJoints; i++) {
        cout << "jcs[" << i << "]" << solved[i] << endl;
        JointCommand jc;
        jc.angle = solved[i];
        jcs.push_back(jc);
      }
      arm.move(jcs);
    }
  }
  catch (std::exception& ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
    return -1;
  }

  return 0;
}
