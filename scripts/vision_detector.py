#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped

class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_detector')
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.fx = np.float64(692.8)
        self.fy = np.float64(692.8)
        self.cx = np.float64(400.0)
        self.cy = np.float64(400.0)
        
        self.baseline = np.float64(0.07)
        
        self.left_image = None
        self.left_header = None

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/custom_stereo/stereo/right/camera_info',
            self.info_callback,
            qos_profile_sensor_data)

        self.sub_left = self.create_subscription(
            Image,
            '/custom_stereo/stereo/left/image_raw',
            self.left_callback,
            qos_profile_sensor_data)

        self.sub_right = self.create_subscription(
            Image,
            '/custom_stereo/stereo/right/image_raw',
            self.right_callback,
            qos_profile_sensor_data)

        self.get_logger().info("Stereo Vision activated. Identifying round shapes.")

    def info_callback(self, msg):
        self.fx = np.float64(msg.k[0])
        self.cx = np.float64(msg.k[2])
        self.fy = np.float64(msg.k[4])
        self.cy = np.float64(msg.k[5])
        if msg.p[3] != 0:
            self.baseline = np.float64(abs(msg.p[3] / self.fx))

    def get_apple_center(self, img, color):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        if color == 'green':
            mask = cv2.inRange(hsv, np.array([35, 50, 50]), np.array([85, 255, 255]))
        else:
            mask = cv2.inRange(hsv, np.array([0, 50, 50]), np.array([10, 255, 255])) + \
                   cv2.inRange(hsv, np.array([170, 50, 50]), np.array([180, 255, 255]))
                   
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = np.float64(cv2.contourArea(cnt))
            if area > 100.0:
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0: continue
                
                # Math check to confirm the shape is round
                circularity = 4 * math.pi * (area / (perimeter * perimeter))
                if circularity > 0.7:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = np.float64(M["m10"] / M["m00"])
                        cy = np.float64(M["m01"] / M["m00"])
                        
                        (x, y), radius = cv2.minEnclosingCircle(cnt)
                        center = (int(x), int(y))
                        radius = int(radius)
                        
                        height, width = img.shape[:2]
                        if 10 < cx < width - 10 and 10 < cy < height - 10:
                            return cx, cy, radius, center
        return None, None, None, None

    def left_callback(self, msg):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.left_header = msg.header
        except:
            pass

    def right_callback(self, msg):
        if self.left_image is None: return
        
        try:
            right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            left_img_copy = self.left_image.copy()
            header = self.left_header
        except:
            return

        for color, bgr in [('red', (0,0,255)), ('green', (0,255,0))]:
            xl, yl, radius_l, center_l = self.get_apple_center(left_img_copy, color)
            xr, yr, radius_r, center_r = self.get_apple_center(right_image, color)

            if xl is not None and xr is not None:
                pixel_shift = np.float64(xl - xr)
                if pixel_shift > 0.1:
                    
                    depth_z = np.float64((self.fx * self.baseline) / pixel_shift)
                    
                    if 0.2 < depth_z < 1.2:
                        cv_x = np.float64((xl - self.cx) * depth_z / self.fx)
                        cv_y = np.float64((yl - self.cy) * depth_z / self.fy)

                        t = TransformStamped()
                        t.header.stamp = header.stamp
                        t.header.frame_id = header.frame_id
                        t.child_frame_id = f'apple_{color}'

                        t.transform.translation.x = float(depth_z)
                        t.transform.translation.y = float(-cv_x)
                        t.transform.translation.z = float(-cv_y)
                        t.transform.rotation.w = 1.0
                        self.tf_broadcaster.sendTransform(t)

                        cv2.circle(left_img_copy, center_l, radius_l, bgr, 2)
                        cv2.putText(left_img_copy, f'{color.capitalize()}: {depth_z:.3f}m', (center_l[0] - radius_l, center_l[1] - radius_l - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2)

        cv2.imshow("Robot Stereo Vision", left_img_copy)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()