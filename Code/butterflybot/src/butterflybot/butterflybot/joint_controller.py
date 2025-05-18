import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import threading


class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Create publishers for joint positions
        self.publisher_joint_front_left = self.create_publisher(Float64, 'joint_front_left_topic', 10)
        self.publisher_joint_front_right = self.create_publisher(Float64, 'joint_front_right_topic', 10)
        self.publisher_joint_rear_left = self.create_publisher(Float64, 'joint_rear_left_topic', 10)
        self.publisher_joint_rear_right = self.create_publisher(Float64, 'joint_rear_right_topic', 10)

        # Create publishers for propeller thrust
        self.publisher_pp_joint_front_left = self.create_publisher(Float64, '/model/robot1/joint/pp_joint_front_left/cmd_thrust', 10)
        self.publisher_pp_joint_front_right = self.create_publisher(Float64, '/model/robot1/joint/pp_joint_front_right/cmd_thrust', 10)
        self.publisher_pp_joint_rear_left = self.create_publisher(Float64, '/model/robot1/joint/pp_joint_rear_left/cmd_thrust', 10)
        self.publisher_pp_joint_rear_right = self.create_publisher(Float64, '/model/robot1/joint/pp_joint_rear_right/cmd_thrust', 10)

        # Call timer_callback every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.active = False  # Motion state toggle
        self.position = 0.51  # Initial joint position
        self.thrust_value = 0.1  # Initial thrust value

    def timer_callback(self):
        if self.active:
            # Toggle joint position
            self.position = -self.position

            # Create and publish joint position messages
            msg_front_left = Float64(data=self.position)
            msg_front_right = Float64(data=-self.position)
            msg_rear_left = Float64(data=self.position)
            msg_rear_right = Float64(data=-self.position)

            self.publisher_joint_front_left.publish(msg_front_left)
            self.publisher_joint_front_right.publish(msg_front_right)
            self.publisher_joint_rear_left.publish(msg_rear_left)
            self.publisher_joint_rear_right.publish(msg_rear_right)

            # Create and publish thrust messages
            msg_thrust = Float64(data=self.thrust_value)
            self.publisher_pp_joint_front_left.publish(msg_thrust)
            self.publisher_pp_joint_front_right.publish(msg_thrust)
            self.publisher_pp_joint_rear_left.publish(msg_thrust)
            self.publisher_pp_joint_rear_right.publish(msg_thrust)

            # Log for debugging
            self.get_logger().info(
                f'Publishing joint positions: FL: {msg_front_left.data}, FR: {msg_front_right.data}, RL: {msg_rear_left.data}, RR: {msg_rear_right.data}')
            self.get_logger().info(
                f'Publishing thrust: {self.thrust_value} to all joints')

    def toggle_active(self):
        self.active = not self.active
        if self.active:
            self.get_logger().info('Motion started')
        else:
            self.get_logger().info('Motion stopped')
            msg_thrust_zero = Float64(data=0.0)
            self.publisher_pp_joint_front_left.publish(msg_thrust_zero)
            self.publisher_pp_joint_front_right.publish(msg_thrust_zero)
            self.publisher_pp_joint_rear_left.publish(msg_thrust_zero)
            self.publisher_pp_joint_rear_right.publish(msg_thrust_zero)


def keyboard_listener(node):
    while rclpy.ok():
        cmd = input("Enter 's' to toggle motion state, 'q' to quit: ")
        if cmd == 's':
            node.toggle_active()
        elif cmd == 'q':
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = JointController()

    thread = threading.Thread(target=keyboard_listener, args=(node,))
    thread.start()

    rclpy.spin(node)

    thread.join()


if __name__ == '__main__':
    main()
