#include "sensor_msgs/msg/laser_scan.hpp"
#include <torcs_ros_drive_ctrl/torcs_ros_drive_ctrl_node.h>

torcs_ros_drive_ctrl::torcs_ros_drive_ctrl() : Node("torcs_ros_drive_ctrl_node") {

    RCLCPP_DEBUG(get_logger(), "Constructor");

    getParams();

    auto timer_countdown = std::chrono::milliseconds(static_cast<long int>(std::round(1000.0/config_.loop_rate)));

    timer_ = this->create_wall_timer(timer_countdown, std::bind(&torcs_ros_drive_ctrl::drive, this));

    torcs_ctrl_in_ = torcs_interfaces::msg::TORCSCtrl();
    torcs_ctrl_out_ = torcs_interfaces::msg::TORCSCtrl();
    torcs_sensors_ = torcs_interfaces::msg::TORCSSensors();

    torcs_sensors_.wheel_spin_vel.resize(4, 0);

    focus_ = sensor_msgs::msg::LaserScan();
    track_ = sensor_msgs::msg::LaserScan();
    opponents_ = sensor_msgs::msg::LaserScan();
    speed_= geometry_msgs::msg::TwistStamped();

    ctrl_sub_ = create_subscription<torcs_interfaces::msg::TORCSCtrl>("torcs_ctrl_in", rclcpp::SystemDefaultsQoS(), std::bind(&torcs_ros_drive_ctrl::ctrlCallback, this, std::placeholders::_1));
    ctrl_pub_ = create_publisher<torcs_interfaces::msg::TORCSCtrl>("torcs_ctrl_out", 1);
    torcs_sensors_sub_ = create_subscription<torcs_interfaces::msg::TORCSSensors>("torcs_sensors_in", rclcpp::SystemDefaultsQoS(), std::bind(&torcs_ros_drive_ctrl::torcsSensorsCallback, this, std::placeholders::_1));

    torcs_track_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("torcs_track", rclcpp::SystemDefaultsQoS(), std::bind(&torcs_ros_drive_ctrl::laserTrackCallback, this, std::placeholders::_1));
    torcs_opponents_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("torcs_opponents", rclcpp::SystemDefaultsQoS(), std::bind(&torcs_ros_drive_ctrl::laserOpponentsCallback, this, std::placeholders::_1));
    focus_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("torcs_focus", rclcpp::SystemDefaultsQoS(), std::bind(&torcs_ros_drive_ctrl::laserFocusCallback, this, std::placeholders::_1));
    speed_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("torcs_speed", rclcpp::SystemDefaultsQoS(), std::bind(&torcs_ros_drive_ctrl::twistSpeedCallback, this, std::placeholders::_1));

    RCLCPP_DEBUG(get_logger(), "Constructor end");
}

torcs_ros_drive_ctrl::~torcs_ros_drive_ctrl(){}

void torcs_ros_drive_ctrl::drive()
{
    RCLCPP_DEBUG(get_logger(), "drive");
    // check if car is currently stuck
    if ( fabs(torcs_sensors_.angle) > config_.stuckAngle )
    {
        // update stuck counter
        stuck++;
    }
    else
    {
        // if not stuck reset stuck counter
        stuck = 0;
    }

    // after car is stuck for a while apply recovering policy
    if (stuck > config_.stuckTime)
    {
        RCLCPP_DEBUG(get_logger(), "car is stuck");
        /* set gear and sterring command assuming car is 
        * pointing in a direction out of track */
    
        // to bring car parallel to track axis
        double steer = - torcs_sensors_.angle / config_.steerLock; 
        int gear=-1; // gear R
        
        // if car is pointing in the correct direction revert gear and steer  
        if (torcs_sensors_.angle*torcs_sensors_.track_pos>0)
        {
            gear = 1;
            steer = -steer;
        }

        // Calculate clutching
        clutching(clutch);

        // build a torcs_ctrl message and publish it
        torcs_ctrl_out_.header.stamp = this->get_clock()->now();
        torcs_ctrl_out_.accel = 1.0;
        torcs_ctrl_out_.brake = 0.0;
        torcs_ctrl_out_.gear = gear;
        torcs_ctrl_out_.steering = steer;
        torcs_ctrl_out_.clutch = clutch;

        ctrl_pub_->publish(torcs_ctrl_out_);
    }
    else // car is not stuck
    {
        RCLCPP_DEBUG(get_logger(), "car is not stuck");
        // compute accel/brake command
        double accel_and_brake;
        if(!track_.ranges.empty())
        {
            accel_and_brake = getAccel();
        }
        else
        {
            accel_and_brake = 0; 
        }
        // compute gear 
        int gear = getGear();
        // compute steering
        double steer = getSteer();
        
        // normalize steering
        if (steer < -1)
            steer = -1;
        if (steer > 1)
            steer = 1;
    
        // set accel and brake from the joint accel/brake command 
        double accel,brake;
        if (accel_and_brake>0)
        {
            accel = accel_and_brake;
            brake = 0;
        }
        else
        {
            accel = 0;
            // apply ABS to brake
            brake = filterABS(-accel_and_brake);
        }

        // Calculate clutching
        clutching(clutch);

        // build a torcs_ctrl message and publish it
        torcs_ctrl_out_.header.stamp = this->get_clock()->now();
        torcs_ctrl_out_.accel = accel;
        torcs_ctrl_out_.brake = brake;
        torcs_ctrl_out_.gear = gear;
        torcs_ctrl_out_.steering = steer;
        torcs_ctrl_out_.clutch = clutch;

        ctrl_pub_->publish(torcs_ctrl_out_);
    }
}

// Solves the gear changing subproblems
int torcs_ros_drive_ctrl::getGear()
{
    int gear = torcs_ctrl_in_.gear;
    int rpm  = torcs_sensors_.rpm;

    // if gear is 0 (N) or -1 (R) just return 1 
    if (gear<1)
        return 1;
    // check if the RPM value of car is greater than the one suggested 
    // to shift up the gear from the current one     
    if (gear <6 && rpm >= config_.gearUp[gear-1])
        return gear + 1;
    else
    // check if the RPM value of car is lower than the one suggested 
    // to shift down the gear from the current one
        if (gear > 1 && rpm <= config_.gearDown[gear-1])
        return gear - 1;
        else // otherwhise keep current gear
        return gear;
}

// Solves the steering subproblems
double torcs_ros_drive_ctrl::getSteer()
{
    // steering angle is compute by correcting the actual car angle w.r.t. to track 
    // axis [cs.getAngle()] and to adjust car position w.r.t to middle of track [cs.getTrackPos()*0.5]
    double targetAngle=(torcs_sensors_.angle - torcs_sensors_.track_pos*0.5);
    // at high speed reduce the steering command to avoid loosing the control
    if (speed_.twist.linear.x > config_.steerSensitivityOffset)
        return targetAngle/(config_.steerLock*(speed_.twist.linear.x-config_.steerSensitivityOffset)*config_.wheelSensitivityCoeff);
    else
        return (targetAngle)/config_.steerLock;
}

// Solves the acceleration subproblems
double torcs_ros_drive_ctrl::getAccel()
{
    // checks if car is out of track
    if (torcs_sensors_.track_pos < 1 && torcs_sensors_.track_pos > -1)
    {
        // reading of sensor at +5 degree w.r.t. car axis
        double rxSensor=track_.ranges[9];
        // double rxSensor=cs.getTrack(10);
        // reading of sensor parallel to car axis
        // double cSensor=cs.getTrack(9);
        double cSensor=track_.ranges[10];
        // reading of sensor at -5 degree w.r.t. car axis
        // double sxSensor=cs.getTrack(8);
        double sxSensor=track_.ranges[11];

        double targetSpeed;

        // track is straight and enough far from a turn so goes to max speed
        if (cSensor>config_.maxSpeedDist || (cSensor>=rxSensor && cSensor >= sxSensor))
            targetSpeed = config_.maxSpeed;
        else
        {
            // approaching a turn on right
            if(rxSensor>sxSensor)
            {
                // computing approximately the "angle" of turn
                double h = cSensor*SIN5;
                double b = rxSensor - cSensor*COS5;
                double sinAngle = b*b/(h*h+b*b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = config_.maxSpeed*(cSensor*sinAngle/config_.maxSpeedDist);
            }
            // approaching a turn on left
            else
            {
                    // computing approximately the "angle" of turn
                double h = cSensor*SIN5;
                double b = sxSensor - cSensor*COS5;
                double sinAngle = b*b/(h*h+b*b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = config_.maxSpeed*(cSensor*sinAngle/config_.maxSpeedDist);
            }
        }
        // accel/brake command is expontially scaled w.r.t. the difference between target speed and current one
        return 2/(1+exp(speed_.twist.linear.x - targetSpeed)) - 1;
    }
    else
        return 0.3; // when out of track returns a moderate acceleration command
}

// Apply an ABS filter to brake command
double torcs_ros_drive_ctrl::filterABS(double brake)
{
    // convert speed to m/s
    double speed = speed_.twist.linear.x / 3.6;
    // when spedd lower than min speed for abs do nothing
    if (speed < config_.absMinSpeed)
        return brake;
    
    // compute the speed of wheels in m/s
    double slip = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        slip += torcs_sensors_.wheel_spin_vel[i] * config_.wheelRadius[i];
    }
    // slip is the difference between actual speed of car and average speed of wheels
    slip = speed - slip/4.0f;
    // when slip too high applu ABS
    if (slip > config_.absSlip)
    {
        brake = brake - (slip - config_.absSlip)/config_.absRange;
    }
    
    // check brake is not negative, otherwise set it to zero
    if (brake<0)
        return 0;
    else
        return brake;
}

// Solves the clucthing subproblems
void torcs_ros_drive_ctrl::clutching(double &clutch)
{
    double maxClutch = config_.clutchMax;

    // Check if the current situation is the race start
    if (torcs_sensors_.current_lap_time < config_.clutchDeltaTime  && config_.stage==2 && torcs_sensors_.dist_raced < config_.clutchDeltaRaced)
        clutch = maxClutch;

    // Adjust the current value of the clutch
    if(clutch > 0)
    {
        double delta = config_.clutchDelta;
        if (torcs_ctrl_in_.gear < 2)
        {
            // Apply a stronger clutch output when the gear is one and the race is just started
            delta /= 2;
            maxClutch *= config_.clutchMaxModifier;
            if (torcs_sensors_.current_lap_time < config_.clutchMaxTime)
                clutch = maxClutch;
        }

        // check clutch is not bigger than maximum values
        clutch = min(maxClutch,double(clutch));

        // if clutch is not at max value decrease it quite quickly
        if (clutch!=maxClutch)
        {
            clutch -= delta;
            clutch = max(0.0,double(clutch));
        }
        // if clutch is at max value decrease it very slowly
        else
            clutch -= config_.clutchDec;
    }
}

void torcs_ros_drive_ctrl::ctrlCallback(const torcs_interfaces::msg::TORCSCtrl::SharedPtr msg)
{
    RCLCPP_DEBUG(get_logger(), "subscribe ctrl");
    torcs_ctrl_in_ = *msg;
}

void torcs_ros_drive_ctrl::torcsSensorsCallback(const torcs_interfaces::msg::TORCSSensors::SharedPtr msg)
{
    RCLCPP_DEBUG(get_logger(), "subscribe sensors");
    torcs_sensors_ = *msg; 
}

void torcs_ros_drive_ctrl::laserTrackCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_DEBUG(get_logger(), "subscribe track");
    track_ = *msg;
}

void torcs_ros_drive_ctrl::laserFocusCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_DEBUG(get_logger(), "subscribe focus");
    focus_ = *msg;
}

void torcs_ros_drive_ctrl::laserOpponentsCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_DEBUG(get_logger(), "subscribe opponents");
    opponents_ = *msg;
}

void torcs_ros_drive_ctrl::twistSpeedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    RCLCPP_DEBUG(get_logger(), "subscribe speed");
    speed_ = *msg;
}

void torcs_ros_drive_ctrl::getParams()
{
    declare_parameter("gearUp", (std::vector<int64_t>)(4, 0));
    config_.gearUp = get_parameter("gearUp").as_integer_array(); 

    declare_parameter("gearDown", (std::vector<int64_t>)(4, 0));
    config_.gearDown = get_parameter("gearDown").as_integer_array(); 

    declare_parameter("stuckTime", (double)25.0);
    config_.stuckTime = get_parameter("stuckTime").as_double(); 

    declare_parameter("stuckAngle", (double)0.523598775);
    config_.stuckAngle = get_parameter("stuckAngle").as_double(); 

    /* Accel and Brake Constants*/
    declare_parameter("maxSpeedDist", (double)70.0);
    config_.maxSpeedDist = get_parameter("maxSpeedDist").as_double(); 

    declare_parameter("maxSpeed", (double)150.0);
    config_.maxSpeed = get_parameter("maxSpeed").as_double(); 

    /* Steering constants*/
    declare_parameter("steerLock", (double)0.366519);
    config_.steerLock = get_parameter("steerLock").as_double(); 

    declare_parameter("steerSensitivityOffset", (double)80.0);
    config_.steerSensitivityOffset = get_parameter("steerSensitivityOffset").as_double(); 

    declare_parameter("wheelSensitivityCoeff", (double)1.0);
    config_.wheelSensitivityCoeff = get_parameter("wheelSensitivityCoeff").as_double(); 

    /* ABS Filter Constants */
    declare_parameter("wheelRadius", (const std::vector<double>)(4, 0.0));
    config_.wheelRadius = get_parameter("wheelRadius").as_double_array(); 

    declare_parameter("absSlip", (double)2.0);
    config_.absSlip = get_parameter("absSlip").as_double(); 

    declare_parameter("absRange", (double)3.0);
    config_.absRange = get_parameter("absRange").as_double(); 

    declare_parameter("absMinSpeed", (double)3.0);
    config_.absMinSpeed = get_parameter("absMinSpeed").as_double(); 

    /* Clutch constants */
    declare_parameter("clutchMax", (double)0.5);
    config_.clutchMax = get_parameter("clutchMax").as_double(); 

    declare_parameter("clutchDelta", (double)0.05);
    config_.clutchDelta = get_parameter("clutchDelta").as_double(); 

    declare_parameter("clutchRange", (double)0.85);
    config_.clutchRange = get_parameter("clutchRange").as_double(); 

    declare_parameter("clutchDeltaTime", (double)0.02);
    config_.clutchDeltaTime = get_parameter("clutchDeltaTime").as_double(); 

    declare_parameter("clutchDeltaRaced", (double)10.0);
    config_.clutchDeltaRaced = get_parameter("clutchDeltaRaced").as_double(); 

    declare_parameter("clutchDec", (double)0.01);
    config_.clutchDec = get_parameter("clutchDec").as_double(); 

    declare_parameter("clutchMaxModifier", (double)1.3);
    config_.clutchMaxModifier = get_parameter("clutchMaxModifier").as_double(); 

    declare_parameter("clutchMaxTime", (double)1.5);
    config_.clutchMaxTime = get_parameter("clutchMaxTime").as_double(); 

    declare_parameter("stage", (int)3);
    config_.stage = get_parameter("stage").as_int(); 

    declare_parameter("loop_rate", (double)100.);
    config_.loop_rate = get_parameter("loop_rate").as_double(); 

}

double torcs_ros_drive_ctrl::getLoopRate()
{
  return config_.loop_rate;
}

int main(int argc, char** argv)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"torcs_ros_drive_ctrl_node: started driving");
    rclcpp::init(argc, argv);
    auto my_node = std::make_shared<torcs_ros_drive_ctrl>();
    rclcpp::spin(my_node);
    rclcpp::shutdown();    
    return 0;
}
