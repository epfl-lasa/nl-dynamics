
#ifndef GPMDS_H
#define GPMDS_H


#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "MultiGPR.h"
#include <functional>

//#ifdef USE_DOUBLE_PRECISION
//typedef double REALTYPE;
//#else
//typedef float REALTYPE;
//#endif
//typedef double REALTYPE;




typedef Eigen::Matrix<REALTYPE,Eigen::Dynamic,Eigen::Dynamic> MatrixXr;
typedef Eigen::Matrix<REALTYPE,Eigen::Dynamic,1> VectorXr;
typedef Eigen::Matrix<REALTYPE,3,1> Vector3r;
typedef Eigen::Matrix<REALTYPE,4,1> Vector4r;
typedef Eigen::Matrix<REALTYPE,3,3> Matrix3r;
typedef Eigen::AngleAxis<REALTYPE> AngleAxisr;

class GPMDS{
  MultiGPR * mGPR;
  //Vector3r (*originalDynamics)(Vector3r);
  std::function<Vector3r(Vector3r)> originalDynamics;
  REALTYPE speedErrorTol,angleErrorTol;
  bool bSparse;
 public:
  GPMDS(REALTYPE a);
  //GPMDS(double a);
  GPMDS(REALTYPE ell, REALTYPE sigmaF, REALTYPE sigmaN,REALTYPE speedErrorTol = 0.1, REALTYPE angleErrorTol = 0.1);
  void setSparse(bool);
  void setGPParameters(REALTYPE ell, REALTYPE sigmaF, REALTYPE sigmaN);
  void setOriginalDynamics(std::function<Vector3r(Vector3r)>);
  void addData(Vector3r position, Vector3r velocity);
  Vector3r reshapedDynamics(Vector3r position);
  Vector4r computeLMDSParameters(Vector3r position, Vector3r velocity);
  void prepareFastQuery();
  bool checkNewData(Vector3r position, Vector4r theta);
  static void dummyFunction(int);
};

#endif // GPMDS_H
