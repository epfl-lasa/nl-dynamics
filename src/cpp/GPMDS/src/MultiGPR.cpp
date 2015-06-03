#include "GPMDS/MultiGPR.h"
#include "GPMDS/Timer.h"

using namespace Eigen;
using namespace std;

MultiGPR::MultiGPR(int inputDim,int outputDim)
{
  inputData.resize(inputDim,0);
  outputData.resize(outputDim,0);
  nData = 0;
}



void MultiGPR::addTrainingData(VectorXr newInput, VectorXr newOutputs)
{
  nData++;
  if(nData>=inputData.cols()){
    inputData.conservativeResize(inputData.rows(),nData);
    outputData.conservativeResize(outputData.rows(),nData);
  }

  //cout<<inputData<<endl<<newInput<<endl;
  inputData.col(nData-1) = newInput;
  outputData.col(nData-1) = newOutputs;
  bNeedPrepare = true;
}



double MultiGPR::SQEcovFuncD(VectorXr x1, VectorXr x2)
{
  dist = x1-x2;
  //cout<<dist<<endl;
  double d = dist.dot(dist);
  d = sigma_f*sigma_f*exp(-1/l_scale/l_scale/2*d);
  return d;
}

VectorXr MultiGPR::SQEcovFunc(MatrixXr x1, VectorXr x2){
  int nCol = x1.cols();
  VectorXr KXx(nCol);
  for(int i=0;i<nCol;i++){
    KXx(i)=SQEcovFuncD(x1.col(i),x2);
  }
  return KXx;
}


// This is a slow process that should be replaced by linear solve at some point
void MultiGPR::prepareRegression(bool force_prepare)
{
  if(!bNeedPrepare & !force_prepare)
    return;
  //    SQEcovFunc(inputData);

  KXX = SQEcovFunc(inputData);
  //    cout<<"dfdf"<<endl;
  KXX_ = KXX;

  for(int i=0;i<KXX.cols();i++)
    KXX_(i,i) += sigma_n*sigma_n;
  //    cout<<"dfdf"<<endl;
  // this is the time theif:
  KXX_ = KXX_.inverse();
  //    cout<<"dfdf"<<endl;
  bNeedPrepare = false;
}



VectorXr MultiGPR::doRegression(VectorXr inp,bool prepare){
  if(prepare){
    prepareRegression();
  }

  VectorXr outp(outputData.rows());
  KXx = SQEcovFunc(inputData,inp);
  KxX = SQEcovFunc(inputData,inp).transpose();
  VectorXr tmp(inputData.cols());
  // this line is the slow one, hard to speed up further?
  tmp = KXX_*KXx;
  // the rest is noise in comparison with the above line.
  for(int i=0;i<outputData.rows();i++){
    outp(i)=tmp.dot(outputData.row(i));
  }
  return outp;
}

MatrixXr MultiGPR::SQEcovFunc(MatrixXr x1){
  int nCol = x1.cols();
  MatrixXr retMat(nCol,nCol);
  for(int i=0;i<nCol;i++){
    for(int j=i;j<nCol;j++){
      retMat(i,j)=SQEcovFuncD(x1.col(i),x1.col(j));
      retMat(j,i)=retMat(i,j);
    }
  }
  return retMat;
}

void MultiGPR::debug()
{
  cout<<"input data \n"<<inputData<<endl;
  cout<<"output data \n"<<outputData<<endl;
}
