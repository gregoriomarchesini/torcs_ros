#include <torcs_img_publisher/torcs_img_publisher.h>

// constructor
torcs_image_publisher_node::torcs_image_publisher_node() : Node("torcs_image_publisher_node"), it_(rclcpp::Node::make_shared("image_transport_node")) {

    getParams();

    auto timer_countdown = std::chrono::milliseconds(static_cast<long int>(std::round(1000.0/config_.loop_rate)));

    timer_ = this->create_wall_timer(timer_countdown, std::bind(&torcs_image_publisher_node::timer_callback, this));

    RCLCPP_DEBUG(get_logger(), "init torcs image publisher node");

    RCLCPP_DEBUG(get_logger(), "Start Memory sharing");
    shmid = shmget((key_t)1234, sizeof(struct shared_use_st), 0666|IPC_CREAT);
    if(shmid == -1)  
    {  
        fprintf(stderr, "shmget failed\n");  
        exit(EXIT_FAILURE);  
    }
    shm = shmat(shmid, 0, 0);  
    if(shm == (void*)-1)  
    {  
        fprintf(stderr, "shmat failed\n");  
        exit(EXIT_FAILURE);  
    }  
    RCLCPP_DEBUG(get_logger(), "Memory sharing started");
    // printf("\n********** Memory sharing started, attached at blablabal %X **********\n", shm);

    shared_ = (struct shared_use_st*)shm; 
    shared_->written = 0;
    shared_->pause = config_.paused;
    shared_->zmq_flag = 0;  
    shared_->save_flag = 0;

    // Setup opencv
    cv::Mat screenRGB_(cvSize(image_width,image_height),IPL_DEPTH_8U,3);
    cv::Mat resizeRGB_(cvSize(config_.resize_width,config_.resize_height),IPL_DEPTH_8U,3);

    header_ = std_msgs::msg::Header();
    // publisher
    image_publisher_ = it_.advertise("pov_image", 1);

    // cv::namedWindow("TORCS Image");
}

torcs_image_publisher_node::~torcs_image_publisher_node(){
    // cv::destroyWindow("TORCS Image");
}

void torcs_image_publisher_node::getParams()
{
    declare_parameter("loop_rate", (double)10.0);
    config_.loop_rate = get_parameter("loop_rate").as_double(); 

    declare_parameter("resize_width", (int)640);
    config_.resize_width = get_parameter("resize_width").as_int(); 

    declare_parameter("resize_height", (int)480);
    config_.resize_height = get_parameter("resize_height").as_int(); 

    declare_parameter("paused", (int)1);
    config_.paused = get_parameter("paused").as_int(); 
}

void torcs_image_publisher_node::timer_callback()
{
  if (shared_->written == 1) {

    for (int h = 0; h < image_height; h++) {
      for (int w = 0; w < image_width; w++) {
       screenRGB_.data[(h*image_width+w)*3+2]=shared_->data[((image_height-h-1)*image_width+w)*3+0];
       screenRGB_.data[(h*image_width+w)*3+1]=shared_->data[((image_height-h-1)*image_width+w)*3+1];
       screenRGB_.data[(h*image_width+w)*3+0]=shared_->data[((image_height-h-1)*image_width+w)*3+2];
      }
    }
    
    resize(screenRGB_, resizeRGB_, cvSize(config_.resize_width,config_.resize_height));
    
    //Mat img = cvarrToMat(resizeRGB_, true);
    // Update GUI Window
    // cv::imshow("TORCS Image", img);
    // cv::waitKey(3);

    header_.stamp = this->get_clock()->now();
  
    //sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header_, "bgr8", img).toImageMsg();
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header_, "bgr8", resizeRGB_).toImageMsg();
    
    image_publisher_.publish(msg);

    shared_->written=0;
  }
}

double torcs_image_publisher_node::getLoopRate()
{
    return config_.loop_rate;
}

int main(int argc, char** argv)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"torcs_image_publisher_node: start torcs image publisher node");
    rclcpp::init(argc, argv);
    auto my_node = std::make_shared<torcs_image_publisher_node>();
    rclcpp::spin(my_node);
    rclcpp::shutdown();    
    return 0;
}
