#include "ros/ros.h"
#include <iostream> 
#include "geometry_msgs/Pose.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tf2_eigen/tf2_eigen.h>
#include <gtest/gtest.h>
#include <tf2/convert.h>


using namespace Eigen;
using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tf2eigen_test");
    ros::NodeHandle nh;

    geometry_msgs::Pose pose1;
    pose1.position.x = 3;
    pose1.position.y = 0;
    pose1.position.z = 0;
    pose1.orientation.x = 0;
    pose1.orientation.y = -0.707;
    pose1.orientation.z = 0;
    pose1.orientation.w = 0.707;

    Eigen::Affine3d out_matrix;
    Eigen::Affine3d out_matrix1;
    //fromMsg(pose1,out_matrix);
    Eigen::MatrixX4f matrix;
    tf2::convert(pose1,out_matrix);
    tf2::fromMsg(pose1,out_matrix1);
    //ROS_INFO_STREAM(out_matrix(3,3));
    for(int i = 0 ; i < 4; i ++)
    {
        for(int j = 0 ; j < 4; j ++)
        {
            std::cout<<out_matrix(i,j)<<" ";
        }
        std::cout<<std::endl;
    }
    //ROS_INFO_STREAM(pose1);
    //ROS_INFO_STREAM(out_matrix);
    return 0;
}