import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo  # Import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray
import image_geometry
from tf2_geometry_msgs import Vector3Stamped, PointStamped
import tf2_ros
from geometry_msgs.msg import Pose2D
import numpy as np
import os
import cv2
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import Float64


class ObjectFollowingNode(Node):
    def __init__(self):
        super().__init__('object_following_node')
        self.subscription = self.create_subscription(Image, '/image', self.image_callback, 10)  # Use CompressedImage
        model_path = os.path.join('/home/mateus/ros2_ws/src/gpg_test/gpg_test', 'runs', 'detect', 'train22', 'weights', 'best.pt')
        self.model = YOLO(model_path)  # load a custom model
        self.cv_bridge = CvBridge()
        self.threshold = 0.23

        self.distance_publish = self.create_publisher(
            Float64, 
            '/distance', 
            10)
        
        self.angle_publish = self.create_publisher(
            Float64, 
            '/angle', 
            10)


    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Perform object detection
        #results = self.model(cv_image)[0]
        results = self.model.track(cv_image)
        
        #Plot results
        result1 = results[0].plot()

        centroids=[]
        i=0

        height, width = cv_image.shape[:2]

        area = None
        distance = None
        x_center = None
        y_center = None
        areas = []
        smallest_area = 3999.0 
        smallest_box = None

        #Visualize
        #for detection in results[0].boxes.xyxy:
        for detection in results[0].boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = detection[:6]  # Extract relevant information

            scores = 1.0

            x_center = (x1 + x2) / 2.0 - width / 2.0
            y_center = (y1 + y2) / 2.0 - height / 2.0
            area = (x2 - x1) * (y2 - y1)

            if area < smallest_area and score > self.threshold:
                smallest_area = area
                smallest_box = (x1, y1, x2, y2, x_center, y_center)  # Save box details

            if smallest_box is not None:
                x1, y1, x2, y2, x_center, y_center = smallest_box

            x_center= (x1 + x2) / 2.0 - width / 2.0
            y_center = (y1 + y2) / 2.0 - height / 2.0
            centroids.append([x_center, y_center])

            #Calculating Bounding Box Size

            area = smallest_area       

            # Print the centroid position for the current object
            self.get_logger().info(f'Object {i + 1}: X = {x_center}, Y = {y_center}, Area = {area}, Distance = {distance} meters')

            if score > self.threshold:
                result = results[0].plot()
                cv_image = result
                # Visualize the result
        cv2.imshow('Robot Camera Feed', cv_image)
        cv2.waitKey(1)

        if isinstance(area, float) and isinstance(x_center, float):
            distance_msg = Float64()
            distance_msg.data = area
            self.distance_publish.publish(distance_msg)

            angle_msg = Float64()
            angle_msg.data = x_center
            self.angle_publish.publish(angle_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
