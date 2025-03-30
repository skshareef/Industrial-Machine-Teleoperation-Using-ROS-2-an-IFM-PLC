#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        
        # Publisher for velocity commands on /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to the /joy topic
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Initialize gear: 1 => 400mA, 2 => 800mA, 3 => 1100mA
        self.gear = 1
        self.gear_to_mA = {
            1: 400.0,
            2: 800.0,
            3: 1100.0
        }

        # Track previous button states to detect "press" events (0->1)
        self.prev_button4 = 0
        self.prev_button5 = 0

        self.get_logger().info(
            "Joystick Teleop Node started.\n"
            " - Axis 7: forward/back\n"
            " - Axis 3: left/right steering\n"
            " - Button 4: decrement gear\n"
            " - Button 5: increment gear\n"
            "Gears: 1=400mA, 2=800mA, 3=1100mA"
        )

    def joy_callback(self, joy_msg: Joy):
        """
        - Axis 7 -> forward/back. (index 7 must exist)
          * axis_7 < -0.1 => forward (+mA based on gear)
          * axis_7 >  0.1 => backward (-mA based on gear)
          * else          => 0 mA
        - Axis 3 -> left/right steering angle (±90°). (index 3 must exist)
        - Button 4 -> decrement gear (down to min=1)
        - Button 5 -> increment gear (up to max=3)
        """
        # Safety checks
        if len(joy_msg.axes) < 8:
            self.get_logger().warn("Joystick has fewer than 8 axes. Can't read axis 7.")
            return
        if len(joy_msg.axes) < 4:
            self.get_logger().warn("Joystick has fewer than 4 axes. Can't read axis 3.")
            return
        if len(joy_msg.buttons) < 6:
            self.get_logger().warn("Joystick has fewer than 6 buttons. Can't read button 4 or 5.")
            return

        # --- 1) Handle Gear Changes via Buttons 4 & 5 ---
        curr_button4 = joy_msg.buttons[4]  # Decrement gear
        curr_button5 = joy_msg.buttons[5]  # Increment gear

        # Detect a "press" (0 -> 1)
        if self.prev_button4 == 0 and curr_button4 == 1:
            self.gear -= 1
            if self.gear < 1:
                self.gear = 1

        if self.prev_button5 == 0 and curr_button5 == 1:
            self.gear += 1
            if self.gear > 3:
                self.gear = 3

        # Update prev button states
        self.prev_button4 = curr_button4
        self.prev_button5 = curr_button5

        # --- 2) Determine forward/back from Axis 7 ---
        axis_7 = joy_msg.axes[7]
        gear_mA = self.gear_to_mA[self.gear]
        if axis_7 < -0.1:
            # Forward => +gear_mA
            speed = gear_mA
        elif axis_7 > 0.1:
            # Backward => -gear_mA
            speed = -gear_mA
        else:
            # No movement
            speed = 0.0

        # --- 3) Compute Steering Angle from Axis 3 ---
        axis_3 = joy_msg.axes[3]
        steering_angle_deg = 90.0 * axis_3  # range -90..+90

        # --- 4) Populate and Publish Twist ---
        twist = Twist()
        twist.linear.x = speed                # mA (forward/back)
        twist.angular.z = steering_angle_deg  # degrees (steering)
        self.cmd_pub.publish(twist)

        # --- Log for clarity ---
        self.get_logger().info(
            f"Gear={self.gear}, Axis7={axis_7:.2f} => Speed={speed:.1f} mA, "
            f"Axis3={axis_3:.2f} => Angle={steering_angle_deg:.1f} deg"
        )

def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
