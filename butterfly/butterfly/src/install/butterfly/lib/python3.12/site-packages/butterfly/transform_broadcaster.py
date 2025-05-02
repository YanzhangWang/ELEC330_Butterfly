from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node

class TransformBroadcasterNode(Node):
    def __init__(self):
        super().__init__('transform_broadcaster_node')
        self.broadcaster = TransformBroadcaster(self)

        # Timer to broadcast transforms periodically
        self.timer = self.create_timer(0.1, self.broadcast_transform)

    def broadcast_transform(self):
        transform = TransformStamped()

        # Set the timestamp
        transform.header.stamp = self.get_clock().now().to_msg()

        # Set the frame IDs
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'camera_link'

        # Set the translation (for example, [1.0, 0.0, 0.0])
        transform.transform.translation.x = 1.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        # Set the rotation (as quaternion)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        # Broadcast the transform
        self.broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = TransformBroadcasterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
