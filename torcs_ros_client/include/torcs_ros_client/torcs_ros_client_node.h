#ifndef TORCS_ROS_CLIENT_NODE_H
#define TORCS_ROS_CLIENT_NODE_H

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <netdb.h>
#include <netinet/in.h>
#include <pthread.h>
#include <unistd.h>

/*** defines for UDP *****/
#define UDP_MSGLEN 1000
#define UDP_CLIENT_TIMEUOT 1000000
//#define __UDP_CLIENT_VERBOSE__
/************************/

#define PI 3.141592653589793

typedef int SOCKET;
typedef struct sockaddr_in tSockAddrIn;
#define CLOSE(x) close(x)
#define INVALID(x) x < 0

using namespace std;

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <torcs_interfaces/msg/torcs_ctrl.hpp>
#include <torcs_interfaces/msg/torcs_sensors.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_broadcaster.h>

#include <SimpleParser.h>

class torcs_ros_client_node : public rclcpp::Node 
{
private:
    struct config_struct
    {
        std::string host_name;
        int server_port;
        std::string id; 
        int max_episodes;
        int max_steps;
        std::string track_name;
        int stage;
        int num_opponents_ranges;
        int num_track_ranges;
        int num_focus_ranges;
        double loop_rate;
    };

    config_struct config_;

    sensor_msgs::msg::LaserScan track_;
    sensor_msgs::msg::LaserScan opponents_;
    sensor_msgs::msg::LaserScan focus_;
    geometry_msgs::msg::TwistStamped speed_;
    geometry_msgs::msg::TwistStamped globalSpeed_; //car speed in reference to world frame
    geometry_msgs::msg::PoseStamped globalPose_; //car pose in reference to world frame 
    geometry_msgs::msg::Vector3Stamped globalRPY_; //roll pitch yaw of car in reference to world frame
    std_msgs::msg::Bool restart_;

    float wheelSpinVel_[4];
    float* track_array_; 
    float* opponents_array_; 
    float* focus_array_;

    torcs_interfaces::msg::TORCSCtrl torcs_ctrl_;  
    torcs_interfaces::msg::TORCSSensors torcs_sensors_;

    SOCKET socketDescriptor_;
    int numRead_;

    char hostName_[1000];
    unsigned int serverPort_;
    char id_[1000];
    unsigned int maxEpisodes_;
    char trackName_[1000];

    tSockAddrIn serverAddress_;
    struct hostent *hostInfo_;
    struct timeval timeVal_;
    fd_set readSet_;
    char buf_[UDP_MSGLEN];

    bool shutdownClient_;
    unsigned long curEpisode_;
    unsigned long currentStep_; 

    std_msgs::msg::String debug_string_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<torcs_interfaces::msg::TORCSCtrl>::SharedPtr ctrl_sub_;
    rclcpp::Publisher<torcs_interfaces::msg::TORCSCtrl>::SharedPtr ctrl_pub_;
    rclcpp::Publisher<torcs_interfaces::msg::TORCSSensors>::SharedPtr torcs_sensors_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr track_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr opponents_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr focus_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr restart_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr globalSpeed_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr globalPose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr globalRPY_pub_;

    torcs_ros_client_node();
    ~torcs_ros_client_node();

    bool connect();

    void timer_callback();

    double getLoopRate();

    void getParams();

    void ctrlCallback(const torcs_interfaces::msg::TORCSCtrl::SharedPtr msg);

    bool getShutdownClientStatus();

    void laserMsgToFloatArray(sensor_msgs::msg::LaserScan scan, float* result);
    void laserMsgFromFloatArray(float* float_array, sensor_msgs::msg::LaserScan &scan_result);

    std::string ctrlMsgToString();

    sensor_msgs::msg::LaserScan initRangeFinder(std::string frame, double angle_min, double angle_max, double range_min, double range_max, int ranges_dim);

    std::string sensorsMsgToString();

    void sensorsMsgFromString(std::string torcs_string);

    virtual void init_angles(float *angles){
        for (int i = 0; i < 19; ++i)
        angles[i]=-90+i*10;
    };

};

#endif
