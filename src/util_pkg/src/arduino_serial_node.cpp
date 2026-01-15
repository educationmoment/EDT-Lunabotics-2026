#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cctype>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>  // sometimes helps with headers depending on version
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class ArduinoSerialNode : public rclcpp::Node
{
public:
  ArduinoSerialNode() : Node("arduino_serial_node")
  {
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud", 115200);

    port_ = this->get_parameter("port").as_string();
    const int baud = this->get_parameter("baud").as_int();

    pub_ = this->create_publisher<std_msgs::msg::Int32>("arduino/raw", 10);

    try {
      serial_.Open(port_);
      serial_.SetBaudRate(baudRateFromInt(baud));  // <-- SAFE mapping (no casting)
      serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      serial_.SetParity(LibSerial::Parity::PARITY_NONE);
      serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

      // Many Arduinos reset when the port opens
      std::this_thread::sleep_for(std::chrono::seconds(2));

      RCLCPP_INFO(get_logger(), "Opened %s @ %d", port_.c_str(), baud);
    }
    catch (const std::exception &e) {
      RCLCPP_FATAL(get_logger(), "Failed to open %s: %s", port_.c_str(), e.what());
      throw;
    }

    timer_ = this->create_wall_timer(10ms, std::bind(&ArduinoSerialNode::tick, this));
  }

private:
  static LibSerial::BaudRate baudRateFromInt(int baud)
  {
    switch (baud) {
      case 9600: return LibSerial::BaudRate::BAUD_9600;
      case 19200: return LibSerial::BaudRate::BAUD_19200;
      case 38400: return LibSerial::BaudRate::BAUD_38400;
      case 57600: return LibSerial::BaudRate::BAUD_57600;
      case 115200: return LibSerial::BaudRate::BAUD_115200;
      default: return LibSerial::BaudRate::BAUD_115200;
    }
  }

void tick()
{
  try {
    std::string line;
    serial_.ReadLine(line, '\n', 200);
    if (line.empty()) return;

    std::string digits;
    for (char c : line) {
      if (std::isdigit(static_cast<unsigned char>(c))) {
        digits.push_back(c);
      }
    }

    // If no digits found, ignore
    if (digits.empty()) return;

    std_msgs::msg::String msg;
    msg.data = digits;   // only numbers, no CR/LF
    pub_->publish(msg);
  }
  catch (...) {
    // ignore serial glitches
  }
}

  std::string port_;
  LibSerial::SerialPort serial_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoSerialNode>());
  rclcpp::shutdown();
  return 0;
}
