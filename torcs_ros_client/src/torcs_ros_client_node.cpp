#include <functional>
#include <netinet/in.h>
#include <torcs_ros_client/torcs_ros_client_node.h>

torcs_ros_client_node::torcs_ros_client_node() : Node("torcs_ros_client_node") {
    getParams();

    auto timer_countdown = std::chrono::milliseconds(static_cast<long int>(std::round(1000.0/config_.loop_rate)));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(timer_countdown, std::bind(&torcs_ros_client_node::timer_callback, this));

    hostInfo_ = gethostbyname(config_.host_name.c_str());
        if (hostInfo_ == NULL)
      {
        RCLCPP_ERROR(get_logger(), "problem interpreting host: %s", config_.host_name.c_str());
        exit(1);
      }

    socketDescriptor_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (INVALID(socketDescriptor_))
    {
        RCLCPP_ERROR(get_logger(), "can not create socket");
        exit(1);
    }

    // Set some fields in the serverAddress structure.
    serverAddress_.sin_family = hostInfo_->h_addrtype;
    memcpy((char *) &serverAddress_.sin_addr.s_addr,
          hostInfo_->h_addr_list[0], hostInfo_->h_length);
    serverAddress_.sin_port = htons(config_.server_port);

    shutdownClient_ = false;
    curEpisode_ = 0;
    currentStep_ = 0;
    numRead_ = 0;

    torcs_ctrl_ = torcs_interfaces::msg::TORCSCtrl();
    torcs_sensors_ = torcs_interfaces::msg::TORCSSensors();
    speed_ = geometry_msgs::msg::TwistStamped();
    globalSpeed_ = geometry_msgs::msg::TwistStamped(); //car speed in reference two world frame
    globalPose_ = geometry_msgs::msg::PoseStamped();
    globalRPY_ = geometry_msgs::msg::Vector3Stamped();
    restart_ = std_msgs::msg::Bool(); //notify other nodes that game is being restarted

    torcs_sensors_.wheel_spin_vel.resize(4, 0);

    focus_array_ = new float[config_.num_focus_ranges];
    focus_ = initRangeFinder("base_link", 0-2*PI/360, 0+2*PI/360, 0, 200, 5);
    track_array_ = new float[config_.num_track_ranges];
    track_ = initRangeFinder("base_link", -PI/2, PI/2, 0, 200, 19);
    opponents_array_ = new float[config_.num_opponents_ranges];
    opponents_ = initRangeFinder("base_link", -PI, 0.99*PI, 0, 200, 36);

    debug_string_ = std_msgs::msg::String();

    ctrl_sub_ = create_subscription<torcs_interfaces::msg::TORCSCtrl>("torcs_ctrl_in", rclcpp::SystemDefaultsQoS(), std::bind(&torcs_ros_client_node::ctrlCallback, this, std::placeholders::_1));
    ctrl_pub_ = create_publisher<torcs_interfaces::msg::TORCSCtrl>("torcs_ctrl_out", 1);
    torcs_sensors_pub_ = create_publisher<torcs_interfaces::msg::TORCSSensors>("torcs_sensors_out", 1);
    globalSpeed_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("torcs_global_speed", 1); 
    globalPose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("torcs_global_pose", 1);
    globalRPY_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("torcs_global_rpy", 1);
    track_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("torcs_track", 1);
    opponents_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("torcs_opponents", 1);
    focus_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("torcs_focus", 1);
    speed_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("torcs_speed", 1);
    debug_pub_ = create_publisher<std_msgs::msg::String>("udp_string", 1);
    restart_pub_ = create_publisher<std_msgs::msg::Bool>("notifications/restart_process", 1);

    bool connected = false;
    while(connected == false)
    {
        RCLCPP_WARN(get_logger(), "Not connected to server yet!!");
        connected = connect();
    }
}

torcs_ros_client_node::~torcs_ros_client_node(){}

bool torcs_ros_client_node::connect(){
    /***********************************************************************************
    ************************* UDP client identification ********************************
    ***********************************************************************************/
    // Initialize the angles of rangefinders
    float angles[19];
    bool result = false;
    init_angles(angles);
    string initString = SimpleParser::stringify(string("init"),angles,19);
    RCLCPP_DEBUG(get_logger(), "Sending id to server: %s", config_.id.c_str());
    initString.insert(0,config_.id);
    RCLCPP_DEBUG(get_logger(), "Sending init string to the server: %s", initString.c_str());
    if (sendto(socketDescriptor_, initString.c_str(), initString.length(), 0,
              (struct sockaddr *) &serverAddress_,
              sizeof(serverAddress_)) < 0)
    {

      RCLCPP_ERROR(get_logger(), "cannot send data");
      CLOSE(socketDescriptor_);
      exit(1);
    }

    // wait until answer comes back, for up to UDP_CLIENT_TIMEUOT micro sec
    FD_ZERO(&readSet_);
    FD_SET(socketDescriptor_, &readSet_);
    timeVal_.tv_sec = 0;
    timeVal_.tv_usec = UDP_CLIENT_TIMEUOT;

    if (select(socketDescriptor_+1, &readSet_, NULL, NULL, &timeVal_))
    {
        // Read data sent by the solorace server
        memset(buf_, 0x0, UDP_MSGLEN);  // Zero out the buffer.
        numRead_ = recv(socketDescriptor_, buf_, UDP_MSGLEN, 0);
        if (numRead_ < 0)
        {
            RCLCPP_ERROR(get_logger(), "didn't get response from server...");
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "Received: %s", buf_);
            if (strcmp(buf_,"***identified***")==0)
            {
                RCLCPP_INFO(get_logger(), "Server %s", buf_);
                result = true;
            }
        }
    }

    return result;
}

double torcs_ros_client_node::getLoopRate()
{
    return config_.loop_rate;
}

bool torcs_ros_client_node::getShutdownClientStatus()
{
    return shutdownClient_;
}

void torcs_ros_client_node::ctrlCallback(const torcs_interfaces::msg::TORCSCtrl::SharedPtr msg)
{
    torcs_ctrl_ = *msg;
}

std::string torcs_ros_client_node::ctrlMsgToString(){
    std::string result;

    result  = SimpleParser::stringify("accel", (float) torcs_ctrl_.accel);
    result += SimpleParser::stringify("brake", (float) torcs_ctrl_.brake);
    result += SimpleParser::stringify("gear",  (int) torcs_ctrl_.gear);
    result += SimpleParser::stringify("steer", (float) torcs_ctrl_.steering);
    result += SimpleParser::stringify("clutch", (float) torcs_ctrl_.clutch);
    result += SimpleParser::stringify("focus",  (float) torcs_ctrl_.focus);
    result += SimpleParser::stringify("meta", (int) torcs_ctrl_.meta);

    return result;
}

std::string torcs_ros_client_node::sensorsMsgToString(){

    std::string result;

    laserMsgToFloatArray(focus_, focus_array_);
    laserMsgToFloatArray(opponents_, opponents_array_);
    laserMsgToFloatArray(track_, track_array_);
    for (int i=0; i<4; i++)
    {
        wheelSpinVel_[i] = torcs_sensors_.wheel_spin_vel[i];
    }

    result  = SimpleParser::stringify("angle", (float)torcs_sensors_.angle);
    result += SimpleParser::stringify("curLapTime", (float)torcs_sensors_.current_lap_time);
    result += SimpleParser::stringify("damage", (float)torcs_sensors_.damage);
    result += SimpleParser::stringify("distFromStart", (float)torcs_sensors_.dist_from_start);
    result += SimpleParser::stringify("distRaced", (float)torcs_sensors_.dist_raced);
    result += SimpleParser::stringify("focus", focus_array_, config_.num_focus_ranges);
    result += SimpleParser::stringify("fuel", (float)torcs_sensors_.fuel);
    result += SimpleParser::stringify("gear", (int)torcs_sensors_.gear);
    result += SimpleParser::stringify("lastLapTime", (float)torcs_sensors_.last_lap_time);
    result += SimpleParser::stringify("opponents", opponents_array_, config_.num_opponents_ranges);
    result += SimpleParser::stringify("racePos", (int)torcs_sensors_.race_pos);
    result += SimpleParser::stringify("rpm", (float)torcs_sensors_.rpm);
    result += SimpleParser::stringify("speedX", (float)speed_.twist.linear.x);
    result += SimpleParser::stringify("speedY", (float)speed_.twist.linear.y);
    result += SimpleParser::stringify("speedZ", (float)speed_.twist.linear.z);
    result += SimpleParser::stringify("track", track_array_, config_.num_track_ranges);
    result += SimpleParser::stringify("trackPos", (float) torcs_sensors_.track_pos);
    result += SimpleParser::stringify("wheelSpinVel", wheelSpinVel_, 4);
    result += SimpleParser::stringify("z", (float)globalPose_.pose.position.z);
    result += SimpleParser::stringify("x", (float)globalPose_.pose.position.x); 
    result += SimpleParser::stringify("y", (float)globalPose_.pose.position.y); 
    result += SimpleParser::stringify("roll", (float)globalRPY_.vector.x);
    result += SimpleParser::stringify("pitch", (float)globalRPY_.vector.y); 
    result += SimpleParser::stringify("yaw", (float)globalRPY_.vector.z); 
    result += SimpleParser::stringify("speedGlobalX", (float)globalSpeed_.twist.linear.x); 
    result += SimpleParser::stringify("speedGlobalY", (float)globalSpeed_.twist.linear.y); 

  return result;
}
void torcs_ros_client_node::sensorsMsgFromString(std::string torcs_string){
    torcs_sensors_.header.stamp = this->get_clock()->now();
    globalSpeed_.header.stamp = this->get_clock()->now();
    globalSpeed_.header.frame_id = "world";
    globalPose_.header.stamp = this->get_clock()->now();
    globalPose_.header.frame_id = "world";
    globalRPY_.header.stamp = this->get_clock()->now();
    globalRPY_.header.frame_id = "world";


    float angle;
    SimpleParser::parse(torcs_string, "angle", angle);
    torcs_sensors_.angle = angle;

    float curLapTime;
    SimpleParser::parse(torcs_string, "curLapTime", curLapTime);
    torcs_sensors_.current_lap_time = curLapTime;

    float damage;
    SimpleParser::parse(torcs_string, "damage", damage);
    torcs_sensors_.damage = damage;

    float distFromStart;
    SimpleParser::parse(torcs_string, "distFromStart", distFromStart);
    torcs_sensors_.dist_from_start = distFromStart;

    float distRaced;
    SimpleParser::parse(torcs_string, "distRaced", distRaced);
    torcs_sensors_.dist_raced = distRaced;

    float fuel;
    SimpleParser::parse(torcs_string, "fuel", fuel);
    torcs_sensors_.fuel = fuel;

    int gear;
    SimpleParser::parse(torcs_string, "gear", gear);
    torcs_sensors_.gear = gear;

    float lastLapTime;
    SimpleParser::parse(torcs_string, "lastLapTime", lastLapTime);
    torcs_sensors_.last_lap_time = lastLapTime;
    
    int racePos;
    SimpleParser::parse(torcs_string, "racePos", racePos);
    torcs_sensors_.race_pos = racePos;

    float rpm;
    SimpleParser::parse(torcs_string, "rpm", rpm);
    torcs_sensors_.rpm = rpm;

    float trackPos;
    SimpleParser::parse(torcs_string, "trackPos", trackPos);
    torcs_sensors_.track_pos = trackPos;

    SimpleParser::parse(torcs_string, "wheelSpinVel", wheelSpinVel_, 4);
    for (int i=0; i<4; i++)
    {
        torcs_sensors_.wheel_spin_vel[i] = wheelSpinVel_[i];
    }

    float z;
    SimpleParser::parse(torcs_string, "z", z);
    torcs_sensors_.z = z; //depreceated
    globalPose_.pose.position.z = z;
    float x;
    SimpleParser::parse(torcs_string, "x", x);
    globalPose_.pose.position.x = x;

    float y;
    SimpleParser::parse(torcs_string, "y", y);
    globalPose_.pose.position.y = y;

    float roll;
    SimpleParser::parse(torcs_string, "roll", roll);
    globalRPY_.vector.x = roll;

    float pitch;
    SimpleParser::parse(torcs_string, "pitch", pitch);
    globalRPY_.vector.y = pitch;

    float yaw;
    SimpleParser::parse(torcs_string, "yaw", yaw);
    globalRPY_.vector.z = yaw;

    //convert roll pitch yaw to quaternion for geometry message
    //geometry_msgs::msg::Quaternion quat_broadcast = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    tf2::Quaternion quat_broadcast;
    quat_broadcast.setEuler(pitch, roll, yaw);
    globalPose_.pose.orientation.x = quat_broadcast.x();
    globalPose_.pose.orientation.y = quat_broadcast.y();
    globalPose_.pose.orientation.z = quat_broadcast.z();
    globalPose_.pose.orientation.w = quat_broadcast.w();

    float speedGX;
    SimpleParser::parse(torcs_string, "speedGlobalX", speedGX);
    globalSpeed_.twist.linear.x = speedGX;

    float speedGY;
    SimpleParser::parse(torcs_string, "speedGlobalY", speedGY);
    globalSpeed_.twist.linear.y = speedGY;

    SimpleParser::parse(torcs_string, "focus", focus_array_, config_.num_focus_ranges);
    laserMsgFromFloatArray(focus_array_, focus_);

    SimpleParser::parse(torcs_string, "opponents", opponents_array_, config_.num_opponents_ranges);
    laserMsgFromFloatArray(opponents_array_, opponents_);

    SimpleParser::parse(torcs_string, "track", track_array_, config_.num_track_ranges);
    laserMsgFromFloatArray(track_array_, track_);

    speed_.header.stamp = this->get_clock()->now();
    float speedX, speedY, speedZ;
    SimpleParser::parse(torcs_string, "speedX", speedX);
    SimpleParser::parse(torcs_string, "speedY", speedY);
    SimpleParser::parse(torcs_string, "speedZ", speedZ);
    speed_.twist.linear.x = speedX;
    speed_.twist.linear.y = speedY;
    speed_.twist.linear.z = speedZ;

}

sensor_msgs::msg::LaserScan torcs_ros_client_node::initRangeFinder(std::string frame, double angle_min, double angle_max, double range_min, double range_max, int ranges_dim)
{
    sensor_msgs::msg::LaserScan result = sensor_msgs::msg::LaserScan();
    result.header = std_msgs::msg::Header();

    result.header.frame_id = frame;
    result.header.stamp = this->get_clock()->now();

    result.angle_min = angle_min;
    result.angle_max = angle_max;
    result.angle_increment = (angle_max - angle_min)/ranges_dim;
    result.range_min = range_min;
    result.range_max = range_max;
    result.ranges.resize(ranges_dim, 0);

    return result;
}
void torcs_ros_client_node::laserMsgToFloatArray(sensor_msgs::msg::LaserScan scan, float* result)
{
    int size = scan.ranges.size();
    for (int i=0; i<size; i++)
    {
        result[i] = scan.ranges[size-i];
    }
}

void torcs_ros_client_node::laserMsgFromFloatArray(float* float_array, sensor_msgs::msg::LaserScan &scan_result)
{
    int size = scan_result.ranges.size();
    scan_result.header.stamp = this->get_clock()->now();
    for (int i=0; i<size; i++)
    {
        scan_result.ranges[size-i] = float_array[i];
    }
}

void torcs_ros_client_node::timer_callback()
{
    RCLCPP_DEBUG(get_logger(), "Start timer callback");
    // wait until answer comes back, for up to UDP_CLIENT_TIMEUOT micro sec
    FD_ZERO(&readSet_);
    FD_SET(socketDescriptor_, &readSet_);
    timeVal_.tv_sec = 0;
    timeVal_.tv_usec = UDP_CLIENT_TIMEUOT;

    if (select(socketDescriptor_+1, &readSet_, NULL, NULL, &timeVal_))
    {
        // Read data sent by the solorace server
        // receive TORCS information from UDP socket
        memset(buf_, 0x0, UDP_MSGLEN);  // Zero out the buffer.
        numRead_ = recv(socketDescriptor_, buf_, UDP_MSGLEN, 0);
        if (numRead_ < 0)
        {
            RCLCPP_ERROR(get_logger(), "didn't get response from server...");
            CLOSE(socketDescriptor_);
            exit(1);
        }

        RCLCPP_DEBUG(get_logger(), "Received: %s", buf_);

        if (strcmp(buf_,"***shutdown***")==0)
        {
            // d.onShutdown();
            shutdownClient_ = true;
            RCLCPP_INFO(get_logger(), "Client Shutdown");
        }

        if (strcmp(buf_,"***restart***")==0)
        {
            // d.onRestart();
            RCLCPP_INFO(get_logger(), "Client Restart");
            restart_.data = true;
            restart_pub_->publish(restart_);
            restart_.data = false;
        }
        /**************************************************
        * Compute The Action to send to the solorace sever
        **************************************************/

        if ( (++currentStep_) != config_.max_steps)
        {
            // string action = d.drive(string(buf));
            // store sensor and ctrl data in ROS messages
            std::string udp_str(buf_);
            debug_string_.data = udp_str;
            debug_pub_->publish(debug_string_);
            sensorsMsgFromString((std::string) buf_);
            // now publish the created ROS messages
            ctrl_pub_->publish(torcs_ctrl_);
            torcs_sensors_pub_->publish(torcs_sensors_);
            globalSpeed_pub_->publish(globalSpeed_);
            globalPose_pub_->publish(globalPose_);
            globalRPY_pub_->publish(globalRPY_);
            track_pub_->publish(track_);
            opponents_pub_->publish(opponents_);
            focus_pub_->publish(focus_);
            speed_pub_->publish(speed_);

            if(torcs_ctrl_.meta == 1)
            {
                restart_.data = true;
            }
            restart_pub_->publish(restart_);
            restart_.data = false;
            //Broadcast tf:Broadcast
            geometry_msgs::msg::TransformStamped base_link = geometry_msgs::msg::TransformStamped(); //car frame in world coordinates

            base_link.header.stamp = this->get_clock()->now();
            base_link.header.frame_id = "world";
            base_link.child_frame_id = "base_link";

            base_link.transform.translation.x = globalPose_.pose.position.x;
            base_link.transform.translation.y = globalPose_.pose.position.y;
            base_link.transform.translation.z = globalPose_.pose.position.z;

            tf2::Quaternion quat_base; //quaternion object used in tf to calculate rotation matrix
            quat_base.setRPY(globalRPY_.vector.x, globalRPY_.vector.y, globalRPY_.vector.z); //set rotation by roll pitch yaw

            base_link.transform.rotation.x = quat_base.x();
            base_link.transform.rotation.y = quat_base.y();
            base_link.transform.rotation.z = quat_base.z();
            base_link.transform.rotation.w = quat_base.w();

            tf_broadcaster_->sendTransform(base_link);

            // create string from subscribed ctrl msg
            std::string action = ctrlMsgToString();
            memset(buf_, 0x0, UDP_MSGLEN);
            sprintf(buf_,"%s",action.c_str());
        }
      else
      {
          sprintf (buf_, "(meta 1)");
      }
        // send action string back to TORCS
        if (sendto(socketDescriptor_, buf_, strlen(buf_)+1, 0, (struct sockaddr *) &serverAddress_, sizeof(serverAddress_)) < 0)
        {
            RCLCPP_ERROR(get_logger(), "cannot send data");
            CLOSE(socketDescriptor_);
            exit(1);
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "Sending: %s", buf_);
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(), "** Server did not respond in 1 second");
    }
    RCLCPP_DEBUG(get_logger(), "End timer callback");

}

void torcs_ros_client_node::getParams()
{
    declare_parameter("host_name", (std::string)"localhost");
    config_.host_name = get_parameter("host_name").as_string(); 

    declare_parameter("server_port", (int)3001);
    config_.server_port = get_parameter("server_port").as_int(); 

    declare_parameter("id", (std::string)"SCR");
    config_.id = get_parameter("id").as_string(); 

    declare_parameter("max_episodes", (int)0);
    config_.max_episodes = get_parameter("max_episodes").as_int(); 

    declare_parameter("max_steps", (int)0);
    config_.max_steps = get_parameter("max_steps").as_int(); 

    declare_parameter("track_name", (std::string)"unknown");
    config_.track_name = get_parameter("track_name").as_string(); 

    declare_parameter("stage", (int)3);
    config_.stage = get_parameter("stage").as_int(); 

    declare_parameter("num_opponents_ranges", (int)36);
    config_.num_opponents_ranges = get_parameter("num_opponents_ranges").as_int(); 

    declare_parameter("num_track_ranges", (int)19);
    config_.num_track_ranges = get_parameter("num_track_ranges").as_int(); 

    declare_parameter("num_focus_ranges", (int)19);
    config_.num_focus_ranges = get_parameter("num_focus_ranges").as_int(); 

    declare_parameter("loop_rate", (double)100.0);
    config_.loop_rate = get_parameter("loop_rate").as_double(); 
}

int main(int argc, char** argv)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"torcs_ros_client_node: start torcs ros client node");
    rclcpp::init(argc, argv);
    auto my_node = std::make_shared<torcs_ros_client_node>();
    rclcpp::spin(my_node);
    rclcpp::shutdown();    
    return 0;
}
