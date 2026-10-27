import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class STM32USBNode(Node):
    def __init__(self):
        super().__init__('STM32_Received_Node')
        self.ser = serial.Serial(
            '/dev/ttyACM0',
            baudrate=115200,
            timeout=0.1
        )

        self.sub = self.create_subscription(
            String,
            '/stm32_cmd',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        command = msg.data.strip()

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()