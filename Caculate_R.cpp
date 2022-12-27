#include <iostream>
#include <sophus/se3.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

int main()
{
    Eigen::Vector3d x_local = {-2216.90345+2660.07638, 3159.71928-3777.20336, -1454.76367+1859.17948};
    Eigen::Vector3d y_local = {-3421.89154+2660.07638, 3116.34696-3777.20336, -2033.38741+1859.17948};
    Eigen::Vector3d z_local = {-2210.47642+2660.07638, 3500.26211-3777.20336, -2774.71575+1859.17948};

    Eigen::Matrix3d R_local;
    R_local.block(0,0,3,1) = x_local.normalized();
    R_local.block(0,1,3,1) = y_local.normalized();
    R_local.block(0,2,3,1) = z_local.normalized();
    cout<<x_local.dot(y_local)<<endl;
    cout<<z_local.dot(y_local)<<endl;
    cout<<x_local.dot(z_local)<<endl;
    cout<<"R_w_2_l = \n"<<R_local<<endl;
    cout<<"R_w_2_l = \n"<<R_local.transpose()<<endl;
    cout<<"R_w_2_l = \n"<<R_local.inverse()<<endl;
    


    return 0;
}