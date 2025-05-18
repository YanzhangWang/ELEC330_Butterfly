import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
import cv2
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/boxes_image',  # Make sure this matches your camera topic
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            Detection2DArray,
            '/object_detections',
            10)
        
        # Add a publisher for the annotated image
        self.image_publisher = self.create_publisher(
            Image,
            '/annotated_image',
            10)
            
        self.bridge = CvBridge()
        self.get_logger().info('Object Detection Node has been started')
        
        # Define object classes with geometric shapes instead
        self.class_names = {
            0: "Sphere",
            1: "Triangle",
            2: "Cube"
        }
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info('Received an image')
            
            # Create a copy of the image for annotation
            annotated_image = cv_image.copy()
            
            # In a real implementation, run your detection model here
            # For now, we'll use OpenCV's simple blob detector to find the green objects
            
            # Convert to HSV for easier color detection
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define color ranges (adjust these values as needed)
            # Green color range
            lower_green = np.array([40, 50, 50])
            upper_green = np.array([80, 255, 255])
            # Cyan color range
            lower_cyan = np.array([85, 50, 50])
            upper_cyan = np.array([95, 255, 255])
            
            # Create masks
            green_mask = cv2.inRange(hsv, lower_green, upper_green)
            cyan_mask = cv2.inRange(hsv, lower_cyan, upper_cyan)
            
            # Find contours
            green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cyan_contours, _ = cv2.findContours(cyan_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Create detection message
            detection_msg = Detection2DArray()
            detection_msg.header = msg.header
            
            # Process green objects (spheres)
            for i, contour in enumerate(green_contours):
                if cv2.contourArea(contour) > 100:  # Filter small contours
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Draw bounding box with label
                    cv2.rectangle(annotated_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    
                    # Add class label and confidence
                    class_id = 0  # Sphere
                    confidence = 0.95
                    label = f"{self.class_names[class_id]}: {confidence:.2f}"
                    cv2.putText(annotated_image, label, (x, y-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Add to detection message
                    det = Detection2D()
                    det.header = msg.header
                    
                    # Set bounding box
                    bbox = BoundingBox2D()
                    # Instead of using bbox.center.x and bbox.center.y directly
                    # Use the correct structure for Pose2D
                    bbox.center.position.x = x + w/2
                    bbox.center.position.y = y + h/2
                    bbox.size_x = float(w)
                    bbox.size_y = float(h)
                    det.bbox = bbox
                    
                    # Set class ID and confidence
                    # If `ObjectHypothesisWithPose` directly has an attribute for class information
                    hypothesis = ObjectHypothesisWithPose()
                    # Option 1: If the message has a nested structure
                    hypothesis.hypothesis.class_id = str(class_id)

                    self.get_logger().info(f"Fields available: {dir(hypothesis)}")

                    det.results.append(hypothesis)
                    
                    detection_msg.detections.append(det)
            
            # Process cyan objects (triangles)
            for i, contour in enumerate(cyan_contours):
                if cv2.contourArea(contour) > 100:  # Filter small contours
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Draw bounding box with label
                    cv2.rectangle(annotated_image, (x, y), (x+w, y+h), (0, 255, 255), 2)
                    
                    # Add class label and confidence
                    class_id = 1  # Triangle
                    confidence = 0.92
                    label = f"{self.class_names[class_id]}: {confidence:.2f}"
                    cv2.putText(annotated_image, label, (x, y-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    
                    # Add to detection message
                    det = Detection2D()
                    det.header = msg.header
                    
                    # Set bounding box
                    bbox = BoundingBox2D()
                    bbox.center.x = x + w/2
                    bbox.center.y = y + h/2
                    bbox.size_x = float(w)
                    bbox.size_y = float(h)
                    det.bbox = bbox
                    
                    # Set class ID and confidence
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.id = str(class_id)
                    hypothesis.score = confidence
                    det.results.append(hypothesis)
                    
                    detection_msg.detections.append(det)
            
            # Publish detection message
            self.publisher.publish(detection_msg)
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.image_publisher.publish(annotated_msg)
            
            self.get_logger().info(f'Published {len(detection_msg.detections)} detections')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()