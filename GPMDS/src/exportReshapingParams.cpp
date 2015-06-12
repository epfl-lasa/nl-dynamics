#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nl_msgs/AnchoredDemonstration.h>
#include <iostream>
#include <fstream>
#include "GPMDS/GPMDS.h"
#include "eigen3/Eigen/Dense"
#include "linear_velocity_fields.h"

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[])
{
    if(argc<2)
        exit(0);

    string file = argv[1];
    rosbag::Bag bag;
    bag.open(file, rosbag::bagmode::Read);
    vector<string> topics;
    topics.push_back("demonstration");
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    rosbag::MessageInstance m = *view.begin();
    nl_msgs::AnchoredDemonstration::ConstPtr msg = m.instantiate<nl_msgs::AnchoredDemonstration>();


    Eigen::Matrix<REALTYPE,3,3> A;
    float speedcap = 0.2;
    A.setZero();
    A(0,0) = -3;
    A(1,1) = -3;
    A(2,2) = -2;
    A*=1;
    Vector3d target;
    target.setZero();
    LinearVelocityField * reaching_motion = new LinearVelocityField(target,A,speedcap);
    auto dyn = std::bind(&LinearVelocityField::ComputeVelocity,reaching_motion,std::placeholders::_1);

    // gpmds dynamics
    GPMDS * gpmds = new GPMDS(0.2,0.4,0.02);
    gpmds->setOriginalDynamics(dyn);

    target<<-0.4,0.4,0.3;

    Vector3d pos,vel,angaxis;
    Vector4d theta;
    int n = msg->num_points;

    Matrix<double,Eigen::Dynamic,7> Data;
    Data.resize(n,7);

    for(int i = 0;i<n;i++){
        pos<<msg->demonstration[i].pose.position.x,msg->demonstration[i].pose.position.y,msg->demonstration[i].pose.position.z;
        vel<<msg->demonstration[i].twist.linear.x,msg->demonstration[i].twist.linear.y,msg->demonstration[i].twist.linear.z;

        theta = gpmds->computeLMDSParameters(pos-target,vel);
        for(int j=0;j<3;j++){
            Data(i,j)=pos(j);
        }
        for(int j=0;j<4;j++){
            Data(i,3+j)=theta(j);
        }
        for(int j=0;j<3;j++){
            angaxis(j)=theta(j);
        }
        for (int j = 0; j < 4; ++j) {
            if(std::isnan(theta(j))){
                cout<<"start here \n"<<endl;
                cout<<"org vel: "<<gpmds->evaluateOriginalDynamics(pos)<<endl;
                cout<<"r vel: "<<vel<<endl;
                cout<<"axis: "<<angaxis/(angaxis.norm()+0.00001) <<endl;
                cout<<"ang "<<angaxis.norm()<<endl;
            }
        }
    }
    string fname = file;
    fname+=".data";
    ofstream fichier(fname, ios::out | ios::trunc);
    fichier<<Data;
    fichier.close();

    return 0;
}

