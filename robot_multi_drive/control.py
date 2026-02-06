import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


import time
import serial
from crccheck.crc import Crc16
import time


class ControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.recording_processes = {}
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        
        self.ser = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=1)

        self.packet_header = [0xDE, 0xAD]
        self.packet_footer = [0xBE, 0xEF]

        self.lx = 0
        self.ly = 1

        self.THRESHOLD = 0.2
        self.get_logger().info('Motor Control Node has been started.')
    

    def value_change(self, value):
        if value < 0:
            return 2
        else:
            return value

    def apply_threshold(self, value, threshold=0.2):
        val = 0
        if abs(value) < threshold:
            val = 0
        elif value > 0:
            val =  1
        else:
            val = 2
        return val

    def reverse_value(self, value):
        if value == 0:
            return 0
        elif value == 1:
            return 2
        else:
            return 1

    def joy_callback(self, msg):
        
        joy_T = self.apply_threshold(msg.axes[6])
        joy_X = self.reverse_value(self.apply_threshold(msg.axes[2]))
        joy_Y = self.apply_threshold(msg.axes[3])
        joy_g = self.value_change(msg.buttons[3] - msg.buttons[1])  # Y - A
        joy_R = self.value_change(msg.buttons[0] - msg.buttons[4])  # X - B
        joy_H = msg.buttons[16]  # HOME button

        joy_lx = msg.axes[self.lx]
        joy_ly = msg.axes[self.ly]

        data_list = [joy_T, joy_X, joy_Y, joy_g, joy_R, joy_H]
        command = 0
        if joy_ly < -self.THRESHOLD:
            command += 6
        elif joy_ly > self.THRESHOLD:
            command += 0
        else:
            command += 3

        if joy_lx < -self.THRESHOLD:
            command += 2
        elif joy_lx > self.THRESHOLD:
            command += 0
        else:
            command += 1
        data_list.append(command)
        self.write_to_serial(data_list)
        # print(f"[SERIAL][SENT][{data_list}]")
        time.sleep(0.01)
        pass

    def calc_crc16(self, data):
        return Crc16.calc(data)
    
    def write_to_serial(self, command, port="/dev/ttyUSB0"):
        data = []
        data += (self.packet_header)
        data += command
        crc_value = self.calc_crc16(bytes(data))

        high_byte = (crc_value >> 8) & 0xFF
        low_byte = crc_value & 0xFF
        data.append(high_byte)
        data.append(low_byte)

        data += (self.packet_footer)
        # self.get_logger().info(f"Writing data: {data}")
        self.ser.write(data)

def main():
    rclpy.init()
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup any running recorders on exit
        for p in node.recording_processes.values():
            if p: p.kill()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
