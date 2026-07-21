#ifndef TORCS_IMAGE_PUBLISHER_NODE_H
#define TORCS_IMAGE_PUBLISHER_NODE_H

#include <iostream>
#include <unistd.h>
#include <sys/shm.h>
#include <stdlib.h>  
#include <stdio.h>
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#define image_width 640
#define image_height 480

struct shared_use_st  
{  
  int written;
  unsigned char data[image_width*image_height*3];
  int pause;
  int zmq_flag; 
  int save_flag; 
};

class torcs_image_publisher_node : public rclcpp::Node 
{
private:
    struct config_struct{
        int resize_width, resize_height;
        double loop_rate;
        int paused;
    };
    void *shm = NULL;
    struct shared_use_st *shared_;
    int shmid;

    // Setup opencv
    cv::Mat screenRGB_;
    cv::Mat resizeRGB_;

    std_msgs::msg::Header header_;

    config_struct config_;

public:
  
    rclcpp::TimerBase::SharedPtr timer_;
    torcs_image_publisher_node();

    ~torcs_image_publisher_node();

    image_transport::ImageTransport it_;

    image_transport::Publisher image_publisher_;

    void timer_callback();

    double getLoopRate();

    void getParams();

};

#endif
