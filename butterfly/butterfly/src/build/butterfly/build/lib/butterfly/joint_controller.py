import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import threading


class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Create publishers for joint2_move, joint3_move, joint4_move, joint5_move topics to publish target positions
        self.publisher_Right_joint = self.create_publisher(Float64, 'Right_joint_topic', 10)
        self.publisher_Left_joint = self.create_publisher(Float64, 'Left_joint_topic', 10)
        # self.publisher_bash_footprint = self.create_publisher(Float64, 'joint4_move', 10)
        # self.publisher_joint5 = self.create_publisher(Float64, 'joint5_move', 10)

        # Create publishers for propeller thrust
        self.publisher_pp_Right_joint = self.create_publisher(Float64, '/model/robot1/joint/pp_Right_joint/cmd_thrust', 10)
        self.publisher_pp_Left_joint = self.create_publisher(Float64, '/model/robot1/joint/pp_Left_joint/cmd_thrust', 10)
        # self.publisher_pp_bash_footprint_joint = self.create_publisher(Float64, '/model/robot1/joint/pp_bash_footprint_joint/cmd_thrust', 10)
        # self.publisher_pp5 = self.create_publisher(Float64, '/model/robot1/joint/pp5_joint/cmd_thrust', 10)

        # Call timer_callback every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.active = False  # Motion state toggle
        self.position = 0.51  # Initial joint position
        self.thrust_value = 0.1  # Initial thrust value

    def timer_callback(self):
        if self.active:
            # Toggle joint position
            self.position = -self.position  # Switch between -0.5 and 0.5
            
            # Create joint messages
            msg_Right_joint = Float64()
            msg_Left_joint = Float64()
            #msg_bash_footprint_joint = Float64()
            # msg_joint5 = Float64()

            # Set target positions for joints
            msg_Right_joint.data = self.position
            msg_Left_joint.data = -self.position
            #msg_bash_footprint_joint.data = -self.position
            # msg_joint5.data = self.position

            # Publish target positions
            self.publisher_Right_joint.publish(msg_Right_joint)
            self.publisher_Left_joint.publish(msg_Left_joint)
            #self.publisher_bash_footprint_joint.publish(msg_bash_footprint_joint)
            # self.publisher_joint5.publish(msg_joint5)

            # Create thrust messages
            msg_pp_Right_joint = Float64(data=self.thrust_value)
            msg_pp_Left_joint = Float64(data=self.thrust_value)
            # msg_pp_bash_footprint_joint = Float64(data=self.thrust_value)
            # msg_pp5 = Float64(data=self.thrust_value)

            # Publish thrust
            self.publisher_pp_Right_joint.publish(msg_pp_Right_joint)
            self.publisher_pp_Left_joint.publish(msg_pp_Left_joint)
            # self.publisher_pp_bash_footprint_joint.publish(msg_pp_bash_footprint_joint)
            # self.publisher_pp5.publish(msg_pp5)

            # Log target positions and thrust for debugging
            self.get_logger().info(
                f'Publishing joint positions: Right_joint: {msg_Right_joint.data}, Left_joint: {msg_Left_joint.data}')
            self.get_logger().info(
                f'Publishing thrust: pp_Right_joint: {msg_pp_Right_joint.data}, pp_Left_joint: {msg_pp_Left_joint.data}')

    def toggle_active(self):
        # Toggle motion state
        self.active = not self.active
        if self.active:
            self.get_logger().info('Motion started')
        else:
            self.get_logger().info('Motion stopped')
            # Stop propeller thrust
            msg_pp_Right_joint = Float64(data=0.0)
            msg_pp_Left_joint = Float64(data=0.0)
            #sg_pp_bash_footprint_joint = Float64(data=0.0)
            # msg_pp5 = Float64(data=0.0)

            self.publisher_pp_Right_joint.publish(msg_pp_Right_joint)
            self.publisher_pp_Left_joint.publish(msg_pp_Left_joint)
            #self.publisher_pp_bash_footprint_joint.publish(msg_pp_bash_footprint_joint)
            # self.publisher_pp5.publish(msg_pp5)


def keyboard_listener(node):
    """
    Independent thread to listen for keyboard inputs to control motion state
    """
    while rclpy.ok():
        cmd = input("Enter 's' to toggle motion state, 'q' to quit: ")
        if cmd == 's':
            node.toggle_active()  # Toggle motion state
        elif cmd == 'q':
            rclpy.shutdown()  # Shutdown ROS2 node


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    node = JointController()

    # Start keyboard listener thread
    thread = threading.Thread(target=keyboard_listener, args=(node,))
    thread.start()

    # Start ROS2 event loop
    rclpy.spin(node)

    # Wait for the thread to finish
    thread.join()


if __name__ == '__main__':
    main()
