#include <math.h>
#include <iostream>
#include "kinematics.h"

const int NumJoints = 4;

int main(void) {
  std::cout << std::fixed;
  std::cout << "Kinematics Test Start" << std::endl;
  int test_count = 0;
  std::vector<double> joints(NumJoints);

  joints[0] = 0;
  joints[1] = 0;
  joints[2] = 0;
  joints[3] = 0;
  std::cout << "Joints:";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  Matrix44 m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ <<"]" << std::endl << str(m) << std::endl;
  std::vector<double> solved = inverse_kinematics(m);
  std::cout << "Solved:";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << solved[i] << " ";
  }
  std::cout << "\n";
  std::cout << "Delta :";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << fabs(joints[i] - solved[i]) << " ";
  }
  std::cout << "\n";

  joints[0] = M_PI / 2;
  joints[1] = 0;
  joints[2] = 0;
  joints[3] = 0;
  std::cout << "Joints:";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ <<"]" << std::endl << str(m) << std::endl;
  solved = inverse_kinematics(m);
  std::cout << "Solved:";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << solved[i] << " ";
  }
  std::cout << "\n";
  std::cout << "Delta :";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << fabs(joints[i] - solved[i]) << " ";
  }
  std::cout << "\n";


  joints[0] = 0;
  joints[1] = 0;
  joints[2] = M_PI / 2;
  joints[3] = 0;
  std::cout << "Joints:";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ <<"]" << std::endl << str(m) << std::endl;
  solved = inverse_kinematics(m);
  std::cout << "Solved:";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << solved[i] << " ";
  }
  std::cout << "\n";
  std::cout << "Delta :";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << fabs(joints[i] - solved[i]) << " ";
  }
  std::cout << "\n";


  joints[0] = 0;
  joints[1] = 0;
  joints[2] = M_PI / 2;
  joints[3] = M_PI / 2;
  std::cout << "Joints:";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ <<"]" << std::endl << str(m) << std::endl;
  solved = inverse_kinematics(m);
  std::cout << "Solved:";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << solved[i] << " ";
  }
  std::cout << "\n";
  std::cout << "Delta :";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << fabs(joints[i] - solved[i]) << " ";
  }
  std::cout << "\n";

  joints[0] = 0;
  joints[1] = 0;
  joints[2] = -M_PI / 2;
  joints[3] = M_PI / 2;
  std::cout << "Joints:";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ << "]" << std::endl << str(m) << std::endl;
  solved = inverse_kinematics(m);
  std::cout << "Solved:";
  for (int i = 0; i < NumJoints; i++) {
    std::cout << solved[i] << " ";
  }
  std::cout << "\n";
  std::cout << "Delta :";
  for (int i = 0; i < NumJoints; i++) {
    std::cout << fabs(joints[i] - solved[i]) << " ";
  }
  std::cout << "\n";


  joints[0] = 0;
  joints[1] = 0;
  joints[2] = M_PI / 2;
  joints[3] = M_PI / 4;
  std::cout << "Joints:";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ << "]" << std::endl << str(m) << std::endl;
  solved = inverse_kinematics(m);
  std::cout << "Solved:";
  for (int i = 0; i < NumJoints; i++) {
    std::cout << solved[i] << " ";
  }
  std::cout << "\n";
  std::cout << "Delta :";
  for (int i = 0; i < NumJoints; i++) {
    std::cout << fabs(joints[i] - solved[i]) << " ";
  }
  std::cout << "\n";


  joints[0] = 0;
  joints[1] = 0;
  joints[2] = M_PI / 4;
  joints[3] = M_PI / 4;
  std::cout << "Joints:";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ << "]" << std::endl << str(m) << std::endl;
  solved = inverse_kinematics(m);
  std::cout << "Solved:";
  for (int i = 0; i < NumJoints; i++) {
    std::cout << solved[i] << " ";
  }
  std::cout << "\n";
  std::cout << "Delta :";
  for (int i = 0; i < NumJoints; i++) {
    std::cout << fabs(joints[i] - solved[i]) << " ";
  }
  std::cout << "\n";

  joints[0] = M_PI / 6;
  joints[1] = 0;
  joints[2] = M_PI / 2;
  joints[3] = M_PI / 4;
  std::cout << "Joints:";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ << "]" << std::endl << str(m) << std::endl;
  solved = inverse_kinematics(m);
  std::cout << "Solved:";
  for (int i = 0; i < NumJoints; i++) {
    std::cout << solved[i] << " ";
  }
  std::cout << "\n";
  std::cout << "Delta :";
  for (int i = 0; i < NumJoints; i++) {
    std::cout << fabs(joints[i] - solved[i]) << " ";
  }
  std::cout << "\n";


  joints[0] = M_PI / 6;
  joints[1] = M_PI / 6;
  joints[2] = M_PI / 2;
  joints[3] = M_PI / 5;
  std::cout << "Joints:";
  for(int i = 0;i < NumJoints;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ << "]" << std::endl << str(m) << std::endl;
  solved = inverse_kinematics(m);
  std::cout << "Solved:";
  for (int i = 0; i < NumJoints; i++) {
    std::cout << solved[i] << " ";
  }
  std::cout << "\n";
  std::cout << "Delta :";
  for (int i = 0; i < NumJoints; i++) {
    std::cout << fabs(joints[i] - solved[i]) << " ";
  }
  std::cout << "\n";

  return 0;
}
