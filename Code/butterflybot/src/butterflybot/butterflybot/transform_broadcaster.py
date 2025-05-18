from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node

class TransformBroadcasterNode(Node):
    def __init__(self):
        super().__init__('transform_broadcaster_node')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transforms)

    def broadcast_transforms(self):
        now = self.get_clock().now().to_msg()

        # Transform 1: base_link -> camera_link
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'base_link'
        t1.child_frame_id = 'camera_link'
        t1.transform.translation.x = 1.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0

        # Transform 2: robot1/lidar_link -> robot1/gpu_lidar
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'robot1/lidar_link/gpu_lidar'
        t2.transform.translation.x = 0.1
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.05
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        # Transform 3: robot1/imu_link -> robot1/imu_sensor
        t3 = TransformStamped()
        t3.header.stamp = now
        t3.header.frame_id = 'base_link'
        t3.child_frame_id = 'robot1/imu_link/imu_sensor'
        t3.transform.translation.x = 0.0
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 0.1
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0

        # Broadcast all transforms
        self.broadcaster.sendTransform(t1)
        self.broadcaster.sendTransform(t2)
        self.broadcaster.sendTransform(t3)

def main(args=None):
    rclpy.init(args=args)
    node = TransformBroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
