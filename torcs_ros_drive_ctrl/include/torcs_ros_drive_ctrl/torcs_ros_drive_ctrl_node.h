#ifndef TORCS_ROS_DRIVE_CTRL_NODE_H
#define TORCS_ROS_DRIVE_CTRL_NODE_H

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iostream>
#include <sstream>
#include <cstring>
#include <cassert>

#define PI 3.141592653589793
#define SIN5 0.08716
#define COS5 0.99619

using namespace std;

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <torcs_interfaces/msg/torcs_ctrl.hpp>
#include <torcs_interfaces/msg/torcs_sensors.hpp>

class torcs_ros_drive_ctrl : public rclcpp::Node 
{
private:
    struct config_struct{
        /* Gear Changing Constants*/
        // RPM values to change gear 
        std::vector<int64_t> gearUp;
        std::vector<int64_t> gearDown;
        /* Stuck constants*/
        
        // How many time steps the controller wait before recovering from a stuck position
        double stuckTime;
        // When car angle w.r.t. track axis is grather tan stuckAngle, the car is probably stuck
        double stuckAngle;

        /* Steering constants*/
        
        // Angle associated to a full steer command
        double steerLock; 
        // Min speed to reduce steering command 
        double steerSensitivityOffset;
        // Coefficient to reduce steering command at high speed (to avoid loosing the control)
        double wheelSensitivityCoeff;
        
        /* Accel and Brake Constants*/
        
        // max speed allowed
        double maxSpeed;
        // Min distance from track border to drive at  max speed
        double maxSpeedDist;
        
        /* ABS Filter Constants */
        
        // Radius of the 4 wheels of the car
        std::vector<double> wheelRadius;
        // min slip to prevent ABS
        double absSlip;           
        // range to normalize the ABS effect on the brake
        double absRange;
        // min speed to activate ABS
        double absMinSpeed;

        /* Clutch constants */
        double clutchMax;
        double clutchDelta;
        double clutchRange;
        double clutchDeltaTime;
        double clutchDeltaRaced;
        double clutchDec;
        double clutchMaxModifier;
        double clutchMaxTime;

        int stage;
    
        double loop_rate;
    
    };

    config_struct config_;

    sensor_msgs::msg::LaserScan track_;
    sensor_msgs::msg::LaserScan opponents_;
    sensor_msgs::msg::LaserScan focus_;
    geometry_msgs::msg::TwistStamped speed_;

    torcs_interfaces::msg::TORCSCtrl torcs_ctrl_out_;  
    torcs_interfaces::msg::TORCSCtrl torcs_ctrl_in_;  
    torcs_interfaces::msg::TORCSSensors torcs_sensors_;
  
public:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<torcs_interfaces::msg::TORCSCtrl>::SharedPtr ctrl_sub_;
    rclcpp::Publisher<torcs_interfaces::msg::TORCSCtrl>::SharedPtr ctrl_pub_;
    rclcpp::Subscription<torcs_interfaces::msg::TORCSSensors>::SharedPtr torcs_sensors_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr torcs_track_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr torcs_opponents_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr focus_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr speed_sub_;

    torcs_ros_drive_ctrl();
    ~torcs_ros_drive_ctrl();

    void getParams();

    // counter of stuck steps
    int stuck;
    
    // current clutch
    double clutch;

    // Solves the gear changing subproblems
    int getGear();

    // Solves the steering subproblems
    double getSteer();
    
    // Solves the acceleration subproblems
    double getAccel();
    
    // Apply an ABS filter to brake command
    double filterABS(double brake);

    // Solves the clucthing subproblems
    void clutching(double &clutch);

    void drive();

    void ctrlCallback(const torcs_interfaces::msg::TORCSCtrl::SharedPtr msg);
    void torcsSensorsCallback(const torcs_interfaces::msg::TORCSSensors::SharedPtr msg);
    void laserTrackCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void laserFocusCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void laserOpponentsCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void twistSpeedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    double getLoopRate();
};

#endif
