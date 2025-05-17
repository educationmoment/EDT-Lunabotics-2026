#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>  // Using Float32 for raw float output
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <iostream>

class SerialReaderNode : public rclcpp::Node {
public:
    SerialReaderNode() : Node("serial_reader_node") {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("arduino_data", 10);

        // Open serial port
        serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            return;
        }

        // Configure serial port
        struct termios tty;
        tcgetattr(serial_fd_, &tty);
        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag = IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 1;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tcsetattr(serial_fd_, TCSANOW, &tty);

        // Timer to poll serial data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SerialReaderNode::readSerial, this)
        );
    }

    ~SerialReaderNode() {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    void readSerial() {
        char buf[256];
        int n = read(serial_fd_, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = 0;
            std::string data(buf);

            // Parse the average light level as a float
            float avgLight = 0.0;
            if (parseLightLevel(data, avgLight) && (avgLight >= 10.0)) {
                std_msgs::msg::Float32 msg;
                msg.data = avgLight;  // Directly assign the float value
                if (msg.data > 96){
                  //RCLCPP_INFO(this->get_logger(), "Bucket full!");
                }
                publisher_->publish(msg);
            }
        }
    }

    bool parseLightLevel(const std::string &data, float &value) {
        // Try to parse the string as a float value
        std::stringstream ss(data);
        if (ss >> value) {
            return true;
        }
        return false;
    }

    int serial_fd_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;  // Using Float32 for raw float output
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialReaderNode>());
    rclcpp::shutdown();
    return 0;
}
