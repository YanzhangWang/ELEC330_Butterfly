import rclpy
from rclpy.node import Node
from cartographer_ros_msgs.srv import WriteState
from python_qt_binding.QtWidgets import QPushButton, QVBoxLayout, QWidget
from rviz2_panel import Panel


class MapSavePanel(Panel):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('MapSavePanel')

        self.node = rclpy.create_node('map_save_panel')
        self.client = self.node.create_client(WriteState, '/write_state')

        self.button = QPushButton('Save Map')
        self.button.clicked.connect(self.save_map)

        layout = QVBoxLayout()
        layout.addWidget(self.button)

        widget = QWidget()
        widget.setLayout(layout)
        self.setWidget(widget)

    def save_map(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error('Service /write_state not available')
            return

        request = WriteState.Request()
        request.filename = '/home/YOUR_USER/butterflybot_state.pbstream'

        future = self.client.call_async(request)

        def done_cb(fut):
            self.node.get_logger().info('Map saved to pbstream')

        future.add_done_callback(done_cb)
