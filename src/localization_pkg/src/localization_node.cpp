#include    "rclcpp/rclcpp.hpp"
#include    "geometry_msgs/msg/accel_stamped.hpp"

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
            "debug_state", 10
        );


        // Create Subscriber to IMU Data
        //////////////////////////////////////
        this->imu_subscription = this->create_subscription<geometry_msgs::msg::AccelStamped>(
            "rs_node/imu",
            10,
            std::bind(&LocalizationNode::imu_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO( this->get_logger(), "Initiallized Subscription to IMU");

        this->timer = this->create_wall_timer(
            std::chrono::milliseconds(250),
            std::bind(&LocalizationNode::timer_callback, this)
        );
        //////////////////////////////////////
        return;
    }

private:
    // Variables
    ////////////////////
    rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr imu_subscription;

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
    
    // Callback function: Handles received IMU frames
    ////////////////////////////////////////
    void imu_callback( const geometry_msgs::msg::AccelStamped::SharedPtr msg ) {
        rclcpp::Time current_time = msg->header.stamp;
        
        if ( ! old_time.nanoseconds() ) {
            old_time = current_time;
            return;
        } 
        double dt = (current_time - old_time).seconds();
        
        // Calculate Linear Velocity
        trapezoidal_rule( this->state.linear_vel.x, this->past_state.linear_accel.x, msg->accel.linear.x, dt );
        trapezoidal_rule( this->state.linear_vel.y, this->past_state.linear_accel.y, msg->accel.linear.y, dt );
        trapezoidal_rule( this->state.linear_vel.z, this->past_state.linear_accel.z, msg->accel.linear.z, dt );
        this->past_state.linear_accel.x = this->state.linear_accel.x; 
        this->past_state.linear_accel.y = this->state.linear_accel.y;
        this->past_state.linear_accel.z = this->state.linear_accel.z;

        // Calculate Angular Velocity
        trapezoidal_rule( this->state.angular_vel.x, this->past_state.angular_accel.x, msg->accel.angular.x, dt );
        trapezoidal_rule( this->state.angular_vel.y, this->past_state.angular_accel.y, msg->accel.angular.y, dt );
        trapezoidal_rule( this->state.angular_vel.z, this->past_state.angular_accel.z, msg->accel.angular.z, dt );
        this->past_state.angular_accel.x = this->state.angular_accel.x;
        this->past_state.angular_accel.y = this->state.angular_accel.y;
        this->past_state.angular_accel.z = this->state.angular_accel.z;

        // RCLCPP_INFO( this->get_logger(), "Received IMU Data at timestamp: %u", msg->header.stamp.nanosec);
        return;
    }
    
    // Trapezoidal Method
    ////////////////////
    void trapezoidal_rule( double& stream, double  val_a, double val_b, double dt) {
        static unsigned int debug_count = 0;

        RCLCPP_INFO(this->get_logger(), "Count: %u", debug_count++);
        if( dt <= 0.0 || dt >= 1.0 ) { return; }

        RCLCPP_WARN( this->get_logger(), "Integrating -- DT == %0.3lf", dt);
        stream += 0.5 * (val_b + val_a) * dt;
        return;
    }

    // Timer Callback
    ////////////////////
    void timer_callback() {
        // RCLCPP_INFO( this->get_logger(), "Timer Tick");
        // RCLCPP_INFO( this->get_logger(), "Velocity X\t|\tVelocity Y\t|\tVelocity Z");
        // RCLCPP_INFO( this->get_logger(), "X Accel: %0.3lf", state.linear_accel.x);
        // RCLCPP_INFO( this->get_logger(), "X Veloc: %0.3lf", state.linear_vel.x);
        // RCLCPP_INFO( this->get_logger(), "%0.3lf\t\t|\t%0.3lf\t\t|\t%0.3lf", state.linear_vel.x, state.linear_vel.y, state.linear_vel.z);
        // RCLCPP_INFO( this->get_logger(), "%0.3lf\t\t|\t%0.3lf\t\t|\t%0.3lf", state.angular_vel.x, state.angular_vel.y, state.angular_vel.z);
        // std_msgs::msg::String msg;
        geometry_msgs::msg::Accel msg;
        msg.linear.x = this->state.linear_vel.x;
        msg.linear.y = this->state.linear_vel.y;
        msg.linear.z = this->state.linear_vel.z;
        msg.angular.x = this->state.angular_vel.x;
        msg.angular.y = this->state.angular_vel.y;
        msg.angular.z = this->state.angular_vel.z;

        // msg.data = "Hello World!";

        this->debug_state->publish(msg);

        
        // RCLCPP_INFO( this->get_logger(), "Speed: %0.2f", std::sqrt( state.linear_vel.x * state.linear_vel.x + state.linear_vel.y * state.linear_vel.y + state.linear_vel.z * state.linear_vel.z ));
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
