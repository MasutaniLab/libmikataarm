
#include <stdlib.h>
#include <iostream>
#include "mikata.h"
#include "Thread.h"
using namespace std;
using namespace ssr::mikata;

int main(const int argc, const char* argv[]) {
  if (argc != 3) {
    std::cout << "Invalid Usage." << std::endl;
    std::cout << "USAGE: $./test filename baudrate" << std::endl;
    return -1;
  }

  try {
    MikataArm m(argv[1], atoi(argv[2]));
    m.servoOn(true);
    m.goHome();

    while (true) {
      std::vector<JointCommand> jc;
      for (int i = 0; i < numJoints; i++) {
        cout << "jc[" << i << "]: ";
        double deg;
        cin >> deg;
        jc[i].angle = deg*M_PI / 180;
      }
      m.move(jc);
    }
  }
  catch (std::exception& ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
    return -1;
  }

  return 0;
}
