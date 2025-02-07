#include    "rclcpp/rclcpp.hpp"
#include    <memory>


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


        RCLCPP_INFO( this->get_logger(), "Accel Stream");
        RCLCPP_INFO( this->get_logger(), "\tX: %0.2lf", this->stream.accel.x);
        RCLCPP_INFO( this->get_logger(), "\tY: %0.2lf", this->stream.accel.y);
        RCLCPP_INFO( this->get_logger(), "\tZ: %0.2lf", this->stream.accel.z);

        RCLCPP_INFO( this->get_logger(), "\tX: %0.2lf", this->stream.gyro.x);
        RCLCPP_INFO( this->get_logger(), "\tY: %0.2lf", this->stream.gyro.y);
        RCLCPP_INFO( this->get_logger(), "\tZ: %0.2lf", this->stream.gyro.z);


        return;
    }

private:
    struct {
        struct {
            double x;
            double y;
            double z;
        } accel;

        struct {
            double x;
            double y;
            double z;
        } gyro;
    } stream;
    


    // Trapezoidal Method
    ////////////////////
    void trapezoidal_rule( double stream, double  val_a, double val_b, double dt) {
        stream += 0.5 * (val_b + val_a) * dt;
        return;
    }

};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
