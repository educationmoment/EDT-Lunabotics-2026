#include    "rclcpp/rclcpp.hpp"
#include    "geometry_msgs/msg/accel_stamped.hpp"
#include    "sensor_msgs/msg/imu.hpp"

// Debug
#include    "geometry_msgs/msg/accel.hpp"
#include    "std_msgs/msg/string.hpp"
#include    "rclcpp/time.hpp"
#include    <iostream>
#include    <memory>
#include    <cmath>


// typedef unsigned long time_t;    // Size of time_t: 64 bis (8-Bytes)


class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode() : Node("localization_node") {
        RCLCPP_INFO( this->get_logger(), "Localization Node Startup" );

        RCLCPP_WARN( this->get_logger(), "Size of..." );
        RCLCPP_WARN( this->get_logger(), "\tINT: %lu", sizeof(int));
        RCLCPP_WARN( this->get_logger(), "\tlong INT: %lu", sizeof(long int));
        RCLCPP_WARN( this->get_logger(), "\tFLOAT: %lu", sizeof(float));
        RCLCPP_WARN( this->get_logger(), "\tDOUBLE: %lu", sizeof(double));


        // Debug
        this->debug_stream  = this->create_publisher<std_msgs::msg::String>(
            "debug_stream", 10
        );
        this->debug_state   = this->create_publisher<geometry_msgs::msg::Accel>(
            "debug_state", 1
        );

        // DEPRECATED
        // Create Subscriber to IMU Data
        //////////////////////////////////////
        // this->imu_subscription = this->create_subscription<geometry_msgs::msg::AccelStamped>(
            // "rs_node/imu",
            // 10,
            // std::bind(&LocalizationNode::imu_callback, this, std::placeholders::_1)
        // );
        // RCLCPP_INFO( this->get_logger(), "Initiallized Subscription to IMU");

        // Create Subscriber to IMU Data
        //////////////////////////////////////
        this->imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data",
            10,
            std::bind(&LocalizationNode::imu_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO( this->get_logger(), "Initiallized Subscription to IMU");

        this->timer = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&LocalizationNode::timer_callback, this)
        );
        //////////////////////////////////////
        return;
    }

private:
    // Variables
    ////////////////////
    // rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr imu_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;

    // Debug Stream
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_stream;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr debug_state;

    // Structure to hold acceleration and gyroscope data 
    ////////////////////
    struct {
        // Acceleration Data
        struct {
            double x, y, z;
        } linear_accel, angular_accel;

        // Velocity Data
        struct {
            double x, y, z;
        } linear_vel, angular_vel;

        // Displacement Data
        struct {
            double x, y, z;
        } linear_disp, angular_disp;
        time_t time_stamp;  // Possible Redundant (I think it is unused)
    } state, past_state;

    rclcpp::TimerBase::SharedPtr timer;

    // Static Variable to hold old timestamp
    ////////////////////
    static rclcpp::Time old_time;

    // Variable to hold initial acceleration (if stationary, assumed gravity)
    ////////////////////////////////////////
    struct {
        double magnitude;
        double x,y,z;
        bool defined;
    } gravity;


    // Callback function: Handles received IMU frames
    ////////////////////////////////////////
    void imu_callback( const sensor_msgs::msg::Imu::SharedPtr msg ) {
        rclcpp::Time current_time = msg->header.stamp; 

        double dt = (current_time - old_time).seconds();
        
        // If Old Time is not assigned, seed old time with 
        // current time and return.
        if( this->old_time.nanoseconds() == 0 ) {
            this->old_time = current_time;
            return;
        }

        // Calculate Linear Velocity 
        trapezoidal_rule( this->state.linear_vel.x, this->past_state.linear_accel.x, msg->linear_acceleration.x, dt );
        trapezoidal_rule( this->state.linear_vel.y, this->past_state.linear_accel.y, msg->linear_acceleration.y, dt );
        trapezoidal_rule( this->state.linear_vel.z, this->past_state.linear_accel.z, msg->linear_acceleration.z, dt );
        this->past_state.linear_accel.x = this->state.linear_accel.x; 
        this->past_state.linear_accel.y = this->state.linear_accel.y;
        this->past_state.linear_accel.z = this->state.linear_accel.z;

        //If abs(this - past) < 0.00001
        //Set all gains to 0 meow  
   
        // Calculate Angular Velocity
        trapezoidal_rule( this->state.angular_vel.x, this->past_state.angular_accel.x, msg->angular_velocity.x  , dt );
        trapezoidal_rule( this->state.angular_vel.y, this->past_state.angular_accel.y, msg->angular_velocity.y  , dt );
        trapezoidal_rule( this->state.angular_vel.z, this->past_state.angular_accel.z, msg->angular_velocity.z  , dt );
        this->past_state.angular_accel.x = this->state.angular_accel.x;
        this->past_state.angular_accel.y = this->state.angular_accel.y;
        this->past_state.angular_accel.z = this->state.angular_accel.z;

        // Store Current time in Past
        old_time = current_time;
        return;
    }
    
    // Trapezoidal Method
    ////////////////////
    void trapezoidal_rule( double& stream, double  val_a, double val_b, double dt) {
        static unsigned int debug_count = 0;
        if( dt <= 0.0 || dt >= 1.0 ) { return; }
        stream += 0.5 * (val_b + val_a) * dt;
        return;
    }

    // Timer Callback
    ////////////////////
    void timer_callback() {
        geometry_msgs::msg::Accel msg;

        // Collect and Publish Linear Velocity
        msg.linear.x = this->state.linear_vel.x;
        msg.linear.y = this->state.linear_vel.y;
        msg.linear.z = this->state.linear_vel.z;

        // Collect and Publish Angular Position
        msg.angular.x = this->state.angular_vel.x;
        msg.angular.y = this->state.angular_vel.y;
        msg.angular.z = this->state.angular_vel.z;

        /*RCLCPP_INFO( this->get_logger(), "Frame @ time %0.3lf", old_time.seconds() );
        RCLCPP_INFO( this->get_logger(), "---------" );
        RCLCPP_INFO( this->get_logger(), "\tX: %0.8lf", msg.angular.x );
        RCLCPP_INFO( this->get_logger(), "\tY: %0.8lf", msg.angular.y );
        RCLCPP_INFO( this->get_logger(), "\tZ: %0.8lf", msg.angular.z );
*/ ///Temporarily commented out while debugging depositing

        this->debug_state->publish(msg);
        return;
    }

};

rclcpp::Time LocalizationNode::old_time = rclcpp::Time(0, 0, RCL_ROS_TIME);


int main(int argc, char* argv[]) {
    std::cout << "Size of type_t: " << sizeof(time_t) << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
