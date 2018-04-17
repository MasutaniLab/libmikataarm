#include "kinematics.h"
#include <vector>
#include <iostream>

const double L1 = 76.5; //アクチュエータ1の下端から第2軸まで．台座の下端からならば加算必要
const double L2z = 148;
const double L2x = 24;
const double L2 = sqrt(L2z*L2z + L2x*L2x); //第2軸と第3軸の軸間距離
const double L3 = 150;
const static double L4 = 42.5+50; //第4軸からハンド中心まで


#include <Eigen/Dense>
//#include <Eigen/SVD>
//#include <Eigen/LU>

inline double normalize_angle(double angle) {
  while(angle > M_PI) angle -= 2 * M_PI;
  while(angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

typedef Eigen::Matrix<double, 4, 4> MATRIX44;
typedef Eigen::Matrix<double, 6, 6> MATRIX66;
typedef Eigen::Vector4d VECTOR4;
typedef Eigen::Matrix<double, 6, 1> VECTOR6;


static MATRIX44 RotTransX(const double t, const double x, const double y, const double z) {
  MATRIX44 result = MATRIX44::Identity(4,4);
  double c = cos(t); double s = sin(t);
  //result(0, 0) = 1; result(0, 1) = 0; result(0, 2) = 0;
  /* result(1, 0) = 0;*/ result(1, 1) = c; result(1, 2) =-s;
  /* result(2, 0) = 0;*/ result(2, 1) = s; result(2, 2) = c;
  result(0, 3) = x; result(1, 3) = y; result(2, 3) = z;
  return result;
}


static MATRIX44 RotTransY(const double t, const double x, const double y, const double z) {
  MATRIX44 result = MATRIX44::Identity(4,4);
  double c = cos(t); double s = sin(t);
  result(0, 0) = c; /*result(0, 1) = 0;*/ result(0, 2) = s;
  //result(1, 0) = 0; result(1, 1) = 1; result(1, 2) = 0; 
  result(2, 0) =-s; /*result(2, 1) = 0;*/ result(2, 2) = c;
  result(0, 3) = x; result(1, 3) = y; result(2, 3) = z;
  return result;
}

static MATRIX44 RotTransZ(const double t, const double x, const double y, const double z) {
  MATRIX44 result = MATRIX44::Identity(4,4);
  double c = cos(t); double s = sin(t);
  result(0, 0) = c; result(0, 1) = -s; /*result(0, 2) = 0;*/
  result(1, 0) = s; result(1, 1) = c; /*result(1, 2) = 0;*/
  //result(2, 0) = 0; result(2, 1) = 0; result(2, 2) = 1; 
  result(0, 3) = x; result(1, 3) = y; result(2, 3) = z;

  return result;
}


Matrix44 mat_to_mat(const MATRIX44& mat) {
  Matrix44 m;
  for(int i = 0;i < 4;i++) {
    for(int j = 0;j < 4;j++) {
      m.v[i][j] = mat(i, j);
    }
  }
  
  return m;
}


MATRIX44 mat_to_mat(const Matrix44& mat) {
  MATRIX44 m;
  for(int i = 0;i < 4;i++) {
    for(int j = 0;j < 4;j++) {
      m(i, j) = mat.v[i][j];
    }
  }
  return m;
}

Matrix44 forward_kinematics(const std::vector<double> joints, int debug) {
  MATRIX44 jointMat[5];
  jointMat[0] = RotTransZ(joints[0], 0, 0, 0);
  jointMat[1] = RotTransY(joints[1], 0, 0, L1);
  jointMat[2] = RotTransY(joints[2], 0, 0, L2);
  jointMat[3] = RotTransY(joints[3], 0, 0, L3);
  jointMat[4] = RotTransZ(0, 0, 0, L4); //ハンドまでの平行移動のみ

  MATRIX44 jointMatAbs[5];
  jointMatAbs[0] = jointMat[0];
  for(int i = 0;i < 4;i++) {
    jointMatAbs[i+1] = jointMatAbs[i] * jointMat[i+1];
    if (debug) {
      std::cout << "J" << i+1 << std::endl << jointMatAbs[i+1] << std::endl;
    }
  }
  VECTOR4 vec(0, 0, -L4, 1); //手先座標系における第4関節の位置
  if (debug) {
    std::cout << "test:" << std::endl << jointMatAbs[4] * vec << std::endl;
    inverse_kinematics(mat_to_mat(jointMatAbs[4]));		       
  }
  return mat_to_mat(jointMatAbs[4]);
}

#define print(x) \
  std::cout << #x ":" << std::endl << x << std::endl;

std::vector<double> inverse_kinematics(const Matrix44& mat) {
  MATRIX44 m = mat_to_mat(mat);
  print(m);
  //与えられた回転行列を実現可能なものに変換する．
  Eigen::Vector3d myMod(-m(1, 3), m(0, 3), 0.0);
  double pitch = 0.0;
  if (myMod.norm() > 1e-10) {
    myMod.normalize();
    Eigen::Vector3d mx = m.block(0, 0, 3, 1);
    Eigen::Vector3d my = m.block(0, 1, 3, 1);
    Eigen::Vector3d mz = m.block(0, 2, 3, 1);
    Eigen::Vector3d p = m.block(0, 3, 3, 1);
    Eigen::Vector3d mzMod = mz - mz.dot(myMod)*myMod;
    mzMod.normalize();
    Eigen::Vector3d mxMod = myMod.cross(mzMod);
    m.block(0, 0, 3, 1) = mxMod;
    m.block(0, 1, 3, 1) = myMod;
    m.block(0, 2, 3, 1) = mzMod;
    if (mzMod.dot(p) > 0) {
      pitch = acos(mzMod(2));
    } else {
      pitch = -acos(mzMod(2));
    }
  }
  print(pitch);

  VECTOR4  v(0, 0, -L4, 1);
  VECTOR4  p4 = m * v; //第4関節の位置
  //print(p4);
  VECTOR4  p2(0, 0, L1, 0); //第2関節の位置
  VECTOR4  v24 = p4 - p2;
  double   d24 = v24.head(3).norm();
  
  double th1, th2, th3, th4;
  double cos3 = (d24*d24 - L2*L2 - L3*L3) / (2 * L2 * L3);
  if (abs(cos3) > 1) {
    //解がない場合，例外を投げるべきでは？
    th1 = th2 = th3 = th4 = 0;
    std::cout << "解なし！" << std::endl;
  } else {
    th3 = acos(cos3);
    double psi = acos((d24*d24 + L2*L2 - L3*L3) / (2 * L2 * d24));
    double phi = atan2(v24.head(2).norm(), v24(2));
    th2 = phi - psi;

    th4 = normalize_angle(pitch - th3 - th2);
    if (v24.head(2).norm() < 1e-10) {
      th1 = 0;
    } else {
      th1 = atan2(v24(1), v24(0));
    }
  }
  
  std::vector<double> j;
  j.push_back(th1);
  j.push_back(th2);
  j.push_back(th3);
  j.push_back(th4);
  return j;
}

Matrix44 xyzpToMatrix44(double x, double y, double z, double pitch)
{
  double yaw = atan2(y, x);
  MATRIX44 m = RotTransZ(yaw, x, y, z) * RotTransY(pitch, 0, 0, 0);
  return mat_to_mat(m);
}
