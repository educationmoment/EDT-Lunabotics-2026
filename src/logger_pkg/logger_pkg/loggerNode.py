from rclpy.node import Node
import csv
from interfaces_pkg.msg import MotorHealth
import rclpy


class LoggerNode( Node ):
    def __init__(self):
        super().__init__("logger_node")
        self.get_logger().info("CSV Logger Initiallized")
        
        # Create Subscription to /motor_health
        self.subscriber_ = self.create_subscription(
            msg_type=MotorHealth, 
            topic="/health_topic", 
            callback=self.subscription_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.initial = True

    # Function: Subscription Callback
    # @brief Acts as the callback function of the /health_topic. Reads subscription and prints
    #        in comma-separated variables format to a log file.
    # @param msg Data to be received
    def subscription_callback(self, msg) -> None:
        if self.initial:
            with open('/ssd/home/edt/Logger/robot_hardware.log', 'w', newline='') as csvfile:
                csvWriter = csv.writer( csvfile, 
                    delimiter='\t|\t',
                    quotechar='|',
                    quoting=csv.QUOTE_MINIMAL  
                )
                csvWriter.writerow(["left_motor_velocity", "left_motor_current", "left_motor_voltage", "left_motor_position", "right_motor_velocity", "right_motor_current", "right_motor_voltage", "right_motor_temperature", "right_motor_position", "left_lift_position", "left_lift_current", "left_lift_voltage", "right_lift_position", "right_lift_current", "right_lift_voltage", "tilt_position", "tilt_current", "tilt_voltage", "vibrator_current", "vibrator_voltage"])
                csvWriter.writerow([msg.left_motor_velocity, msg.left_motor_current, msg.left_motor_voltage, msg.left_motor_temperature, msg.left_motor_position, msg.right_motor_velocity, msg.right_motor_current, msg.right_motor_voltage, msg.right_motor_temperature, msg.right_motor_position, msg.left_lift_position, msg.left_lift_current, msg.left_lift_voltage, msg.right_lift_position, msg.right_lift_current, msg.right_lift_voltage, msg.tilt_position, msg.tilt_current, msg.tilt_voltage, msg.vibrator_current, msg.vibrator_voltage])
            self.initial = False
            pass
        else:
            with open('/ssd/home/edt/Logger/robot_hardware.log', 'a', newline='') as csvfile:
                csvWriter = csv.writer( csvfile, 
                    delimiter='\t|\t',
                    quotechar='|',
                    quoting=csv.QUOTE_MINIMAL  
                )
                # csvWriter.writerow(["left_motor_velocity", "left_motor_current", "left_motor_voltage", "left_motor_position", "right_motor_velocity", "right_motor_current", "right_motor_voltage", "right_motor_temperature", "right_motor_position", "left_lift_position", "left_lift_current", "left_lift_voltage", "right_lift_position", "right_lift_current", "right_lift_voltage", "tilt_position", "tilt_current", "tilt_voltage", "vibrator_current", "vibrator_voltage"])
                csvWriter.writerow([msg.left_motor_velocity, msg.left_motor_current, msg.left_motor_voltage, msg.left_motor_temperature, msg.left_motor_position, msg.right_motor_velocity, msg.right_motor_current, msg.right_motor_voltage, msg.right_motor_temperature, msg.right_motor_position, msg.left_lift_position, msg.left_lift_current, msg.left_lift_voltage, msg.right_lift_position, msg.right_lift_current, msg.right_lift_voltage, msg.tilt_position, msg.tilt_current, msg.tilt_voltage, msg.vibrator_current, msg.vibrator_voltage])
            pass
        return None
        
def main() -> int:
    try:
        rclpy.init()
        node = LoggerNode()
        rclpy.spin(node=node)
        rclpy.shutdown()
    except FileNotFoundError:
        print("File Not Found")
    except TypeError:
        print("Type Error")
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    return 0


if __name__ == "__main__":
    exit( main() )