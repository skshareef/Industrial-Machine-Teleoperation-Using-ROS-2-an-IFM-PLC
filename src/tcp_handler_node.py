#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TcpHandlerNode(Node):
    """
    Example node using while loop + spin_once.
    Connects to a TCP server (the IFM controller), toggles brake via button 0 in /joy,
    listens to /cmd_vel for speed/steering, and sends commands over TCP.
    """

    def __init__(self):
        super().__init__('tcp_handler_node')

        # TCP server details (IFM controller)
        self.controller_ip = '192.168.82.247'
        self.controller_port = 17123

        # Internal states
        self.brake_on = False
        self.prev_button0 = 0
        self.prev_speed = 0.0
        self.current_steer_output = 'none'
        self.steering_deadzone = 0.2  # tune as needed

        # Create TCP socket & connect
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.get_logger().info(f"Connecting to TCP server {self.controller_ip}:{self.controller_port} ...")
        try:
            self.sock.connect((self.controller_ip, self.controller_port))
            self.get_logger().info(f"Connected to {self.controller_ip}:{self.controller_port}")
        except socket.error as e:
            self.get_logger().error(f"Failed to connect: {e}")

        # Subscriptions
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info("TcpHandlerNode initialized with manual while-loop spinning.")

    def send_commands(self, commands):
        """Send each string in commands to the TCP server."""
        for cmd in commands:
            msg = (cmd + "\n").encode('utf-8')
            try:
                self.sock.sendall(msg)
                self.get_logger().info(f"Sent to TCP: {cmd}")
            except socket.error as e:
                self.get_logger().error(f"Error sending '{cmd}': {e}")

    def joy_callback(self, joy_msg: Joy):
        """Brake toggling on button0 as an example."""
        if len(joy_msg.buttons) < 1:
            return

        button0 = joy_msg.buttons[0]
        if self.prev_button0 == 0 and button0 == 1:
            self.brake_on = not self.brake_on
            if self.brake_on:
                self.handle_brake_on()
            else:
                self.handle_brake_off()
        self.prev_button0 = button0

    def handle_brake_on(self):
        self.get_logger().info("Brake ON => OUT0002 ON, speed=0, steering off.")
        cmds = [
            "OUT0002 ON",
            "SPEED=0",
            "REV_SPEED=0"
        ]
        # Turn off steering if any
        if self.current_steer_output == 'left':
            cmds.append("OUT0102 OFF")
        elif self.current_steer_output == 'right':
            cmds.append("OUT0101 OFF")

        self.send_commands(cmds)
        self.prev_speed = 0.0
        self.current_steer_output = 'none'

    def handle_brake_off(self):
        self.get_logger().info("Brake OFF => OUT0002 OFF.")
        self.send_commands(["OUT0002 OFF"])
        # Keep speed/steering states so we don't resend unless changed

    def cmd_vel_callback(self, msg: Twist):
        """Reads linear.x, angular.z, sends speed/steering if brake is off."""
        if self.brake_on:
            return  # ignore speed/steering if brake is on

        cmds = []

        # Speed logic
        linear_x = msg.linear.x
        if abs(linear_x - self.prev_speed) > 1e-6:
            if abs(linear_x) < 1e-6:
                cmds.append("SPEED=0")
            elif linear_x > 0:
                cmds.append(f"SPEED={linear_x}")
            else:
                cmds.append(f"REV_SPEED={abs(linear_x)}")
            self.prev_speed = linear_x

        # Steering logic with a deadzone
        angular_z = msg.angular.z
        if abs(angular_z) < self.steering_deadzone:
            # Turn off whichever was on
            if self.current_steer_output == 'left':
                cmds.append("OUT0102 OFF")
                self.current_steer_output = 'none'
            elif self.current_steer_output == 'right':
                cmds.append("OUT0101 OFF")
                self.current_steer_output = 'none'
        elif angular_z > 0:
            if self.current_steer_output != 'left':
                cmds.append("OUT0102 ON")
                self.current_steer_output = 'left'
        else:  # angular_z < 0
            if self.current_steer_output != 'right':
                cmds.append("OUT0101 ON")
                self.current_steer_output = 'right'

        if cmds:
            self.send_commands(cmds)

def main(args=None):
    rclpy.init(args=args)
    node = TcpHandlerNode()

    try:
        # Instead of rclpy.spin(node), do a while-loop:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            # If needed, do other operations in each loop iteration here.
            # time.sleep(0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt => shutting down.")
    finally:
        if node.sock:
            node.sock.close()
            node.get_logger().info("Closed TCP socket.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
