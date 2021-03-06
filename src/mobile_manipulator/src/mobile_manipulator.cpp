#include <stdio.h>  
#include "math.h"
#include "sstream"
#include "iomanip"
#include <sys/types.h>  
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>  
#include "ros/ros.h"
#include "string"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <gtest/gtest.h>
#include "Eigen/Core"
#include "Eigen/Geometry"


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;


class move_robot
{//定义一个和运动相关的类，主要用来获取当前的位恣
public:
    move_robot():group("manipulator"),robot_model_loader("robot_description"),kinematic_model(robot_model_loader.getModel()),
                kinematic_state(*group.getCurrentState()),joint_model_group(kinematic_model->getJointModelGroup("manipulator"))
    {
        Client ac("follow_joint_trajectory", true);
        if (!ac.waitForServer(ros::Duration(2.0)))
        {
            ROS_ERROR("Could not connect to action server");
            exit(-1);
        }
        else
        {
            ROS_INFO("Connecte with UR10 Now!");
        }
        group.setMaxVelocityScalingFactor(0.3);
        group.setMaxAccelerationScalingFactor(0.2);
        group.setNamedTarget("null_space");
        group.move();
        ros::Duration(0.5).sleep();

    }
    moveit::planning_interface::MoveGroup group;
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    moveit::core::RobotState kinematic_state;
    const robot_state::JointModelGroup* joint_model_group;

};

class TCPIPAPI
{//定义一个tcpip类，主要是用来做socket的初始化，用来发送数据接受数据   
public:
    TCPIPAPI()
    {
        memset(&remote_addr,0,sizeof(remote_addr)); //数据初始化--清零  
        remote_addr.sin_family=AF_INET; //设置为IP通信  
        remote_addr.sin_addr.s_addr=inet_addr("192.168.1.101");//服务器IP地址  
        remote_addr.sin_port=htons(30003); //服务器端口号 
        if((client_sockfd=socket(AF_INET,SOCK_STREAM,0))<0)  
        {  
            perror("socket");  
            exit(-1);  
        } 
        if((client_sockfd=socket(AF_INET,SOCK_STREAM,0))<0)  
        {  
            perror("socket");  
            exit(-1);
        }
        if(connect(client_sockfd,(struct sockaddr *)&remote_addr,sizeof(struct sockaddr))<0)  
        {  
            perror("connect");  
            exit(-1);  
        }  
        ROS_INFO("CONNECTED TO SERVER");
    }
    void close_socket()
    {
        close(client_sockfd);
        ROS_INFO("SOCKET CLOSED!");
    } 
    int send_msg(std::string& msg)
    {
        strcpy(buf,msg.c_str());
        len = send(client_sockfd,buf,strlen(buf),0);
        return len;
    }
    char* recv_msg(void)
    {
        len = recv(client_sockfd,buf,BUFSIZ,0);
        return buf;
    }
    //全局变量
    int client_sockfd;  
    int len;  
    struct sockaddr_in remote_addr; //服务器端网络地址结构体  
    char buf[BUFSIZ];  //数据传送的缓冲区  
    

};


class MOBILE_MANIPULATOR
{
public:
    MOBILE_MANIPULATOR()
    {
        odom_data_flag = 0;
        length_a2b = -0.4;//这个数据还需要修改
        max_acc = 1.2;
        eef_vel.resize(6);
        pid.resize(3);
        pid = {5,5,5};
        max_vel = {0.8,0.8,0.8};

        //由于机械臂基座的坐标系和AGV基座的坐标系平行，只需要指定一个x方向的位移就行
        arm_base_to_agv_base_pose.position.x = length_a2b;
        arm_base_to_agv_base_pose.position.y = arm_base_to_agv_base_pose.position.z = 0;
        arm_base_to_agv_base_pose.orientation.x = arm_base_to_agv_base_pose.orientation.y = arm_base_to_agv_base_pose.orientation.z = 0;
        arm_base_to_agv_base_pose.orientation.w = 1;
        tf2::convert(arm_base_to_agv_base_pose,arm_base_to_agv_base_affine);
    }
void start();
void display(Eigen::Affine3d &matrix)
{
    std::cout<<"The Transpose Is"<<std::endl;
    for(int i = 0 ; i < 4; i ++)
    {
        for(int j = 0 ; j < 4 ; j ++)
        {
            std::cout<<std::setw(6)<<matrix(i,j);
        }
        std::cout<<std::endl;
    }
}

void subcallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(odom_data_flag == 0) 
    {
        //初始的数据值，当做世界坐标系来使用
        agv_base_pose_referenece = msg->pose.pose;
        tf2::convert(agv_base_pose_referenece,base_affine_init);
        base_affine_init_inverse = base_affine_init.inverse();
    }

    global_msgs = msg->pose.pose;
    tf2::convert(global_msgs,base_now_affine);
    //base_now_affine是当前小车中心相对出发的时候小车中心的齐次坐标变化
    base_now_affine = base_affine_init_inverse*base_now_affine;
    //这是实际的机械臂的基座相对出发点的坐标变换
    arm_base_affine = base_now_affine*arm_base_to_agv_base_affine;
    display(arm_base_affine);


    if(odom_data_flag == 0)
    {
        arm_pose_now = move.group.getCurrentPose(move.group.getEndEffectorLink()).pose;
        tf2::convert(arm_pose_now,eef_to_arm_base_affine);
        //在这次试验中，这个量需要保持不变，即机械臂末端在世界坐标系下面的位置不变
        eef_to_world_affine = arm_base_affine*eef_to_arm_base_affine;
        odom_data_flag = 1;
    }
}

private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher pose_now_pub;
    ros::Publisher pose_target_pub;

    TCPIPAPI tcpip;
    move_robot move;

    int odom_data_flag;
    std::vector<double> eef_vel;
    double max_acc;
    std::vector<double> max_vel;
    std::vector<double> pid;
    double length_a2b;

    geometry_msgs::Pose global_msgs,local_msgs;

    geometry_msgs::Pose arm_base_to_agv_base_pose;//AGV和机械臂之间的连接的关系，这个是一直不变的

    geometry_msgs::Pose agv_base_pose_now , agv_base_pose_referenece;//和AGV相关的位姿
    Eigen::Affine3d base_affine_init, base_affine_init_inverse, base_now_affine, arm_base_to_agv_base_affine, arm_base_affine;

    geometry_msgs::Pose arm_pose_now, arm_pose_target_true, arm_pose_target;//和机械臂相关的位姿
    Eigen::Affine3d eef_to_world_affine, eef_to_arm_base_affine;
    
    


    std::string combinemsg(std::vector<double> &velocity, double &acc)
    {
        double time2move = 0.1;//给机器人的运动时间多一点，但是实际上并不会运动那么多就会被下一条指令覆盖
        std::string move_msg;
        move_msg = "speedl([";
        move_msg = move_msg + double2string(velocity[0]) + ",";
        move_msg = move_msg + double2string(velocity[1]) + ",";
        move_msg = move_msg + double2string(velocity[2]) + ",";
        move_msg = move_msg + double2string(velocity[3]) + ",";
        move_msg = move_msg + double2string(velocity[4]) + ",";
        move_msg = move_msg + double2string(velocity[5]) + "]";
        move_msg = move_msg + ",";
        move_msg = move_msg + double2string(acc) + ",";
        move_msg = move_msg + double2string(time2move) + ")";
        move_msg = move_msg + "\n";
        return move_msg; 
    }
    std::string double2string(double &input)
    {
        std::string string_temp;
        std::stringstream stream;
        stream<<input;
        string_temp = stream.str();
        return string_temp;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mobile_manipulator");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    MOBILE_MANIPULATOR mobile_manipulator;
    mobile_manipulator.start(); 
    return 0;
}

void MOBILE_MANIPULATOR::start()
{

    pose_now_pub = nh.advertise<geometry_msgs::Pose>("pose_now",1000);
    pose_target_pub = nh.advertise<geometry_msgs::Pose>("pose_target",1000);
    //最重要的一个函数

    arm_pose_target = arm_pose_target_true = arm_pose_now;
    ROS_INFO_STREAM("Init Pose Is "<<arm_pose_now);
    std::string move_msg;//发送关于ur_script的消息

    for(int i = 0 ; i < 6 ; i ++)
    {
        eef_vel[i] = 0;
    }

    odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1000, &MOBILE_MANIPULATOR::subcallback,this);//1000是缓冲区的大小
    ros::Duration(1.0).sleep();

    local_msgs = global_msgs;

    ros::Time time_init = ros::Time::now();

    while(ros::ok())
    {
        if(odom_data_flag == 1)
        {
            while(ros::ok())
            {
                double error_x = global_msgs.position.x - local_msgs.position.x;
                double error_y = global_msgs.position.y - local_msgs.position.y;
                double error_z = global_msgs.orientation.w - local_msgs.orientation.w;
                if(fabs(error_x)>0.0000005||fabs(error_y)>0.0000005||fabs(error_z) > 0.0000005) 
                {
                    ROS_INFO("Reveiving New Data Now!");
                    //这个循环一般都不会死掉
                    break;
                }
            }
            local_msgs = global_msgs;
            eef_to_arm_base_affine = arm_base_affine.inverse()*eef_to_world_affine;
            tf2::convert(eef_to_arm_base_affine,arm_pose_target);
            arm_pose_now = move.group.getCurrentPose(move.group.getEndEffectorLink()).pose;

            eef_vel[0] = pid[0] * (arm_pose_target.position.x - arm_pose_now.position.x);
            eef_vel[1] = pid[1] * (arm_pose_target.position.y - arm_pose_now.position.y);
            eef_vel[2] = pid[2] * (arm_pose_target.position.z - arm_pose_now.position.z);
            for(int i = 0 ; i < 3; i ++)
            {
                if(eef_vel[i]>max_vel[i])
                    eef_vel[i] = max_vel[i];
                if(eef_vel[i]<(-max_vel[i]))
                    eef_vel[i] = -max_vel[i];
            }
            pose_now_pub.publish(arm_pose_now);
            pose_target_pub.publish(arm_pose_target);
            move_msg = combinemsg(eef_vel,max_acc);
            ROS_INFO_STREAM("Sending msg "<<move_msg);
            tcpip.send_msg(move_msg);
        }
        else
        {
            ROS_WARN("NO Odom DATA!!!");
            ros::Duration(0.1).sleep();
        }
    }
}
