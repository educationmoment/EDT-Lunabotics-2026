#include    "rclcpp/rclcpp.hpp"
// #include    "std_msgs/msg/Float32MultiArray"
#include    "geometry_msgs/msg/accel_stamped.hpp"
#include    <iostream>
#include    <memory>


// typedef unsigned long time_t;    // Size of time_t: 64 bis (8-Bytes)

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode() : Node("localization_node") {
        RCLCPP_INFO( this->get_logger(), "Test from Localization Node" );

        RCLCPP_INFO( this->get_logger(), "Initiallizing stream");
        this->stream.accel.x = 0.0;
        this->stream.accel.y = 0.0;
        this->stream.accel.z = 0.0;
        this->stream.gyro.x = 0.0;
        this->stream.gyro.y = 0.0;
        this->stream.gyro.z = 0.0;

        RCLCPP_WARN( this->get_logger(), "Size of..." );
        RCLCPP_WARN( this->get_logger(), "\tINT: %lu", sizeof(int));
        RCLCPP_WARN( this->get_logger(), "\tlong INT: %lu", sizeof(long int));
        RCLCPP_WARN( this->get_logger(), "\tFLOAT: %lu", sizeof(float));
        RCLCPP_WARN( this->get_logger(), "\tDOUBLE: %lu", sizeof(double));





        RCLCPP_INFO( this->get_logger(), "Accel Stream");
        RCLCPP_INFO( this->get_logger(), "\tX: %0.2lf", this->stream.accel.x);
        RCLCPP_INFO( this->get_logger(), "\tY: %0.2lf", this->stream.accel.y);
        RCLCPP_INFO( this->get_logger(), "\tZ: %0.2lf", this->stream.accel.z);

        RCLCPP_INFO( this->get_logger(), "\tX: %0.2lf", this->stream.gyro.x);
        RCLCPP_INFO( this->get_logger(), "\tY: %0.2lf", this->stream.gyro.y);
        RCLCPP_INFO( this->get_logger(), "\tZ: %0.2lf", this->stream.gyro.z);
        

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
            double x;
            double y;
            double z;
        } accel;

        // Gyroscope Data
        struct {
            double x;
            double y;
            double z;
        } gyro;

        time_t time_stamp;
    } stream;
    
    void imu_callback( const geometry_msgs::msg::AccelStamped::SharedPtr msg ) {
        static double oldAccel_y = 0; 
        static unsigned int old_time = 0;
        
        if ( !old_time ) {
            old_time = msg->header.stamp.nanosec;
            return;
        } 
        
        if ( old_time == (unsigned long long)msg->header.stamp.nanosec ) {
            return;
        }
        
        trapezoidal_rule( this->stream.accel.y, oldAccel_y, msg->accel.linear.y, msg->header.stamp.nanosec - old_time);
        oldAccel_y = msg->accel.linear.y;
        
        
        RCLCPP_INFO( this->get_logger(), "Received IMU Data at timestamp: %u", msg->header.stamp.nanosec);
        RCLCPP_INFO( this->get_logger(), "Velocity Y: %0.2f", this->stream.accel.y);
        return;
    }
    

    // Trapezoidal Method
    ////////////////////
    void trapezoidal_rule( double& stream, double  val_a, double val_b, unsigned int dt) {
        if( !dt ) { return; }
        RCLCPP_INFO( this->get_logger(), "\033[104mVal A: \t%0.3lf\033[0m", val_a );
        RCLCPP_INFO( this->get_logger(), "\033[104mVal B: \t%0.3lf\033[0m", val_b );
        RCLCPP_INFO( this->get_logger(), "\033[104mDel T: \t%u\033[0m", dt );
        RCLCPP_INFO( this->get_logger(), "\033[106mVal A: \t%0.3lf\033[0m", 0.5 * ( val_a + val_b) * static_cast<double>(dt) / 1E9 );



        stream += 0.5 * (val_b + val_a) * static_cast<double>(dt) / 1E9;
        return;
    }

};


int main(int argc, char* argv[]) {
    std::cout << "Size of type_t: " << sizeof(time_t) << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
