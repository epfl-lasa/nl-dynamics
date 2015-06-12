#ifndef MGPR_H
#define MGPR_H


# include <eigen3/Eigen/Dense>
# include <iostream>

//#ifdef USE_DOUBLE_PRECISION
//typedef double REALTYPE;
//#else
//typedef float REALTYPE;
//#endif

typedef double REALTYPE;

typedef Eigen::Matrix<REALTYPE,Eigen::Dynamic,Eigen::Dynamic> MatrixXr;
typedef Eigen::Matrix<REALTYPE,Eigen::Dynamic,1> VectorXr;

class MultiGPR{
  MatrixXr inputData;
  MatrixXr outputData;
  MatrixXr KXX;
  MatrixXr KXX_;
  MatrixXr KXx;
  MatrixXr KxX;

  int nData;
  bool bNeedPrepare;

  double l_scale;
  double sigma_f;
  double sigma_n;

  VectorXr dist;

  VectorXr regressors;


 public:
  MultiGPR(){}


  MultiGPR(int inputDim, int outputDim);

  void setHyperParams(double l, double f, double n){l_scale = l; sigma_f = f; sigma_n = n;};
  void addTrainingData(VectorXr newInput, VectorXr newOutputs);

  double SQEcovFuncD(VectorXr x1,VectorXr x2);
  void debug();


  MatrixXr SQEcovFunc(MatrixXr x1);
  VectorXr SQEcovFunc(MatrixXr x1, VectorXr x2);

  void prepareRegression(bool force_prepare = false);
  VectorXr doRegression(VectorXr inp,bool prepare = false);
  int getNData(){return nData;};
  void clearTrainingData();
};


#endif //MGPR_H
