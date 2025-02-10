#include    "rclcpp/rclcpp.hpp"
// #include    "std_msgs/msg/Float32MultiArray"
#include    "geometry_msgs/msg/accel_stamped.hpp"
#include    "rclcpp/time.hpp"
#include    <iostream>
#include    <memory>


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

        // Create Subscriber to IMU Data
        //////////////////////////////////////
        this->imu_subscription = this->create_subscription<geometry_msgs::msg::AccelStamped>(
            "rs_node/imu",
            10,
            std::bind(&LocalizationNode::imu_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO( this->get_logger(), "Initiallized Subscription to IMU");
        //////////////////////////////////////
        return;
    }

private:
    // Variables
    ////////////////////
    rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr imu_subscription;


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
        time_t time_stamp;
    } state, past_state;

    // Static Variable to hold old timestamp
    ////////////////////
    static rclcpp::Time old_time;
    
    // Callback function: Handles received IMU frames
    ////////////////////////////////////////
    void imu_callback( const geometry_msgs::msg::AccelStamped::SharedPtr msg ) {
        rclcpp::Time current_time = msg->header.stamp;
        double dt = (current_time - old_time).seconds();

        if ( ! old_time.nanoseconds() ) {
            old_time = current_time;
            return;
        } 
        
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
        if( dt <= 0.0 ) { return; }

        // RCLCPP_INFO( this->get_logger(), "\033[104mVal A: \t%0.3lf\033[0m", val_a );
        // RCLCPP_INFO( this->get_logger(), "\033[104mVal B: \t%0.3lf\033[0m", val_b );
        // RCLCPP_INFO( this->get_logger(), "\033[104mDel T: \t%u\033[0m", dt );
        // RCLCPP_INFO( this->get_logger(), "\033[106mVal A: \t%0.3lf\033[0m", 0.5 * ( val_a + val_b) * static_cast<double>(dt) / 1E9 );

        stream += 0.5 * (val_b + val_a) * dt;
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
