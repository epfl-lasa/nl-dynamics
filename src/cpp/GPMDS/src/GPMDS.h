#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "MultiGPR.h"

#ifdef USE_DOUBLE_PRECISION
typedef double REALTYPE;
#else
typedef float REALTYPE;
#endif

using namespace Eigen;
using namespace std;

typedef Matrix<REALTYPE,Dynamic,Dynamic> MatrixXr;
typedef Matrix<REALTYPE,Dynamic,1> VectorXr;
typedef Matrix<REALTYPE,3,1> Vector3r;
typedef Matrix<REALTYPE,4,1> Vector4r;
typedef Matrix<REALTYPE,3,3> Matrix3r;
typedef AngleAxis<REALTYPE> AngleAxisr;

class GPMDS{
  MultiGPR * mGPR;
  Vector3r (*originalDynamics)(Vector3r);
  REALTYPE speedErrorTol,angleErrorTol;
 public:
  GPMDS(REALTYPE ell, REALTYPE sigmaF, REALTYPE sigmaN,REALTYPE speedErrorTol = 0.1, REALTYPE angleErrorTol = 0.1);
  void setGPParameters(REALTYPE ell, REALTYPE sigmaF, REALTYPE sigmaN);
  void setOriginalDynamics(Vector3r (*fun)(Vector3r));
  void addData(Vector3r position, Vector3r velocity);
  Vector3r reshapedDynamics(Vector3r position);
  Vector4r computeLMDSParameters(Vector3r position, Vector3r velocity);
  void prepareFastQuery();
  bool checkNewData(Vector3r position, Vector4r theta);

};
