#include "GPMDS/GPMDS.h"
#include "GPMDS/Timer.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <fstream>

using namespace std;
using namespace Eigen;

void load_training_data(const char *fname, vector<Vector3r> &pos, vector<Vector3r> &vel) {
  Vector3r tempPos;
  Vector3r tempVel;
  ifstream myfile;
  myfile.open(fname);
  while(myfile >> tempPos(0)){
    myfile>>tempPos(1)>>tempPos(2)>>tempVel(0)>>tempVel(1)>>tempVel(2);
    pos.push_back(tempPos);
    vel.push_back(tempVel);
  }
}

Vector3r linear_isotropic_dynamics(Vector3r pos) {
  Vector3r vel;
  vel = -3*pos;
  return vel;
}

int main(int argc, char *argv[]) {
  const double ell = 0.07;
  const double sigmaF = 1.0;
  const double sigmaN = 0.4;
  const double speedErrorTol = 0.01;
  const double angleErrorTol = 0.1;
  GPMDS mGPMDS(ell, sigmaF, sigmaN, speedErrorTol, angleErrorTol);
  mGPMDS.setOriginalDynamics(&linear_isotropic_dynamics);
  vector<Vector3r> training_pos;
  vector<Vector3r> training_vel;

  load_training_data("data.txt", training_pos, training_vel);
  cout<<"done reading "<<training_pos.size()<<" "<<training_vel.size()<<endl;
  Timer tmr;
  int N = training_pos.size();
  for (int i = 0; i < N; i++){
    //cout<<"in here now<<"<<endl;
    mGPMDS.addData(training_pos[i], training_vel[i]);
  }
  cout<<"done adding "<<tmr.elapsed()<<endl;

  tmr.reset();
  mGPMDS.prepareFastQuery();
  cout<<"preparing took.."<<tmr.elapsed()<<endl;
  Vector3r tvel;
  tmr.reset();
  tvel = mGPMDS.reshapedDynamics(training_pos[0]);
  cout<<"reshaping took.."<<tmr.elapsed()<<endl;

  return 0;
}
