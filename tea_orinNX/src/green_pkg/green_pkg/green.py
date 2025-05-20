#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2
import pyzed.sl as sl

class PaddyNode(Node):
    def __init__(self):
        super().__init__('greenoneobj')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'information', 5)
        self.timer = self.create_timer(1.25, self.process_frame)  # 0.8 Hz

        self.get_logger().info('Paddy node started')
        self.zed = sl.Camera()
        init_params = sl.InitParameters(
            camera_resolution=sl.RESOLUTION.HD2K,
            camera_fps=60,
            depth_mode=sl.DEPTH_MODE.PERFORMANCE,
            coordinate_units=sl.UNIT.METER,
            depth_maximum_distance=10,
            enable_image_enhancement=True,
            coordinate_system=sl.COORDINATE_SYSTEM.IMAGE
        )

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Camera initialization failed: {repr(status)}")
            rclpy.shutdown()
            return

        self.image_zed = sl.Mat()
        self.depth_zed = sl.Mat()
        self.point_cloud_zed = sl.Mat()
        self.runtime_params = sl.RuntimeParameters()
        self.kernel = np.ones((5, 5), "uint8")

    def process_frame(self):
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.depth_zed, sl.MEASURE.DEPTH)
            self.zed.retrieve_measure(self.point_cloud_zed, sl.MEASURE.XYZ)
            image_frame = self.image_zed.get_data()

            hsv_frame = cv2.cvtColor(image_frame, cv2.COLOR_BGRA2BGR)
            hsv_frame = cv2.cvtColor(hsv_frame, cv2.COLOR_BGR2HSV)

            green_lower = np.array([35, 100, 50], np.uint8)
            green_upper = np.array([85, 255, 255], np.uint8)
            green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)
            green_mask = cv2.dilate(green_mask, self.kernel)

            contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 300:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(image_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(image_frame, "Green Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

                    center_x = x + w // 2
                    center_y = y + h // 2
                    depth_value = self.depth_zed.get_value(center_x, center_y)[1]
                    print('Depth Value is' ,depth_value) 

                    if np.isfinite(depth_value):
                        msg = Float32MultiArray()
                        msg.data = [(depth_value - 0.8) * 100]
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Published depth: {msg.data[0]:.2f} cm")
                    else:
                        self.get_logger().warn("Depth not valid.")

            cv2.imshow("ZED | Green Color Detection", image_frame)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PaddyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.zed.close()
        cv2.destroyAllWindows()
        rclpy.shutdown()
