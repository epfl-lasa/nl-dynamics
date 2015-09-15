#include "GPMDS/GPMDS.h"
#include <math.h>
#include "GPMDS/Timer.h"

using namespace Eigen;
using namespace std;

GPMDS::GPMDS(REALTYPE a)
{

}

//GPMDS::GPMDS(double a)
//{

//}

GPMDS::GPMDS(REALTYPE ell, REALTYPE sigmaF, REALTYPE sigmaN,REALTYPE speedErrorTol, REALTYPE angleErrorTol)
{
  //Initialize Gaussian Process
  mGPR = new MultiGPR(3,4);
  setGPParameters(ell, sigmaF, sigmaN);
  this->speedErrorTol = speedErrorTol;
  this->angleErrorTol = angleErrorTol;
  bSparse = false;
}

void GPMDS::setSparse(bool s=true){
  bSparse = s;
}

void GPMDS::setGPParameters(REALTYPE ell, REALTYPE sigmaF, REALTYPE sigmaN)
{
  //Create the GPR (input 3, output 4)
  //mGPR = new MultiGPR(3,4);
  //Set parameters for the GPR;
  mGPR->setHyperParams(ell, sigmaF, sigmaN);
}

void GPMDS::setOriginalDynamics(std::function<Vector3r(Vector3r)> fun)
{
  //Set the original dynamics to the function passed in by the user
  originalDynamics = fun;
}

void GPMDS::prepareFastQuery()
{
  //Set the original dynamics to the function passed in by the user
  mGPR->prepareRegression(true);
}

Vector4r GPMDS::computeLMDSParameters(Vector3r position, Vector3r velocity){
  Vector3r originalVelocity, axis;
  Vector4r theta;
  REALTYPE kappa, angle;

  //Compute original dynamics at position
  originalVelocity = originalDynamics(position);

  // Find the scaling between vectors
  kappa = (velocity.norm()/originalVelocity.norm())-1.0;
  //cout<<"velocoity.norm() "<<velocity.norm()<<" org. vel nomr. "<<originalVelocity.norm()<<endl;
  //Compare resulting original velocity with velocity to compute speed scaling (kappa) and rotation axis angle
  angle = acos((originalVelocity.dot(velocity))/(originalVelocity.norm()*velocity.norm()));
  Vector3r colinerity_measure;
  colinerity_measure= originalVelocity/originalVelocity.norm() - velocity/velocity.norm();
//  if(colinearity_measure.norm()<0.00001){
//      axis.setRandom();
//      axis -= axis.dot(velocity)*velocity/velocity.norm();
//  }else{
  axis = originalVelocity.cross(velocity);

  axis /= axis.norm();

  //Create the datapoint, theta (the 4 number output of the GPR- axis angle, scaling)
  for (int i=0; i<3; i++)
    theta(i) = angle*axis(i);

//  cout<<"start here"<<endl;
//  cout<<originalVelocity<<endl;
//  cout<<velocity<<endl;
//  cout<<axis<<endl;
//  cout<<"ang "<<angle<<endl;
  theta(3) = kappa;
//  for (int i = 0; i < 4; ++i) {
//      if(std::isnan(theta(i))){
//          cout<<"start here"<<endl;
//          cout<<originalVelocity<<endl;
//          cout<<velocity<<endl;
//          cout<<axis<<endl;
//          cout<<"ang "<<angle<<endl;
//      }
//  }

  return theta;
}


void GPMDS::addData(Vector3r position, Vector3r velocity)
{
  Vector4r theta;
  // need to subtract target posioitn
  theta = computeLMDSParameters(position, velocity);
  //cout<<"theta before GPR: "<<theta<<endl;
  if(checkNewData(position,theta))
    mGPR->addTrainingData(position, theta);
}

bool GPMDS::checkNewData(Vector3r position, Vector4r theta){
  if(mGPR->getNData() <1 || bSparse==false)
    return true;

  Vector4r theta_p;
  theta_p = mGPR->doRegression(position,true);
  Vector3r angleaxis_p,angleaxis;
  for(int i=0;i<3;i++){
    angleaxis(i) = theta(i);
    angleaxis_p(i) = theta_p(i);
  }
  REALTYPE angle,angle_p;
  angle_p = angleaxis_p.norm();
  angle = angleaxis.norm();
  if(fabs(theta_p(3) - theta(3))>speedErrorTol)// || fabs(angle_p-angle)>angleErrorTol){
  {
    return true;
  }
  else {
    cout << "Number of data points: " << mGPR->getNData() << endl;
    return false;
  }
}

void GPMDS::clearData()
{
    mGPR->clearTrainingData();
}

Vector3r GPMDS::reshapedDynamics(Vector3r position)
{

  Vector3r originalVelocity, velocity, axis;
  Vector4r result;
  Matrix3r rot_mat;
  REALTYPE kappa, angle;


  // Get original dynamics at position
  originalVelocity = originalDynamics(position);
  //cout<<"dsfijsijfd"<<endl;
  //cout<<originalVelocity<<endl;
  // Perform GPR on the given position to compute reshaping parameters
  //(*mGPR).prepareRegression();
  //cout<<originalVelocity<<endl;
  result = (*mGPR).doRegression(position);

  // Calculate axis and angle from reshaping parameters
  for (int i=0; i<3; i++){axis(i)=result(i);}
  kappa  = result(3);
  angle = axis.norm();
  axis = axis/angle;
  if (angle < 0.001)
  {
    // If the angle is very small, make rotation matrix the identity
   rot_mat.setIdentity();
   //cout<<"yes cam here"<<rot_mat<<endl;
  }
  else
  {
    // Find the rotation matrix from the axis and angle
    AngleAxisr rot_aa(angle, axis);
    rot_mat = rot_aa.toRotationMatrix();
  }
  //cout<<rot_mat<<endl;
  //Rotate and scale the original velocity
  velocity = rot_mat*originalVelocity;
  //cout<<velocity<<endl;
  //kappa = 0.0;
  if(kappa < -0.7)
      kappa = -0.7;

  kappa = 0.0;
  velocity= velocity*(kappa+1);
   // cout<<velocity<<endl;
  //Return resulting velocity
  return velocity;
}

Vector3r GPMDS::evaluateOriginalDynamics(Vector3r position)
{
    return originalDynamics(position);
}


void GPMDS::dummyFunction(int a){

}
