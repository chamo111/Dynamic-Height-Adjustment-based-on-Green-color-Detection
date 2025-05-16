#!/usr/bin/env python3

import numpy as np
import cv2
import pyzed.sl as sl
import rospy
from std_msgs.msg import Float32MultiArray

nodeName = 'greenoneobj.py'
topicName = 'information'

rospy.init_node(nodeName, anonymous=True)  # Initialize the publisher node

publisher1 = rospy.Publisher(topicName, Float32MultiArray, queue_size=5)  # Publishing to the topic name
ratePublisher = rospy.Rate(0.8)  # Publishing 1 message per second

intMessage=0

def main():
    # Initialize ZED Camera
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD2K  # Use HD2K resolution
    init_params.camera_fps = 60  # Set fps to 60
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Set depth mode to performance
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units for depth measurements
    init_params.depth_maximum_distance = 10  # Set maximum depth distance to 10 meters
    init_params.enable_image_enhancement = True  # Enable image enhancement
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE  # Use image coordinate system

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Camera initialization failed: {repr(status)}")
        exit()

    # Create Mat objects for capturing ZED images and depth
    image_zed = sl.Mat()
    depth_zed = sl.Mat()
    point_cloud_zed = sl.Mat()
    runtime_params = sl.RuntimeParameters()

    kernel = np.ones((5, 5), "uint8")  # Kernel for morphological transformations

    print("Press 'q' to quit.")
    depth = Float32MultiArray()

    while True:
        # Grab a new frame from the ZED
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            # Retrieve the left image, depth map, and point cloud from the ZED camera
            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud_zed, sl.MEASURE.XYZ)

            image_frame = image_zed.get_data()

            # Convert ZED image (BGRA) to HSV color space
            hsv_frame = cv2.cvtColor(image_frame, cv2.COLOR_BGRA2BGR)
            hsv_frame = cv2.cvtColor(hsv_frame, cv2.COLOR_BGR2HSV)

            # Define color ranges and create masks for green color
            green_lower = np.array([35, 100, 50], np.uint8)  # Adjust these for your lighting
            green_upper = np.array([85, 255, 255], np.uint8)  # Adjust these for your lighting
            green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)

            # Morphological Transform, Dilation for the green mask
            green_mask = cv2.dilate(green_mask, kernel)

            # Find contours for the green mask
            contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 300:  # Filter small contours
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(image_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(image_frame, "Green Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

                    # Get depth and 3D coordinates at the center of the bounding box
                    center_x = x + w // 2
                    center_y = y + h // 2
                    depth_value = depth_zed.get_value(center_x, center_y)[1]  # Depth value
                    point3D = point_cloud_zed.get_value(center_x, center_y)[1]  # 3D coordinates (x, y, z)

                    intMessage1 = (depth_value - 0.8) * 1000  # Example transformation
                    #intMessage2 = (depth_value - 0.4) * 1000
                    
                    
                    depth.data = [intMessage1]
                    rospy.loginfo(f"Publishing array: {depth.data}")
                    publisher1.publish(depth)  # Publish the message

                    # Check if depth value is valid
                    if np.isfinite(depth_value):
                        print(f"Green detected at depth: {depth_value:.2f} meters")
                        print(f"published depth: {intMessage:.2f} meters")
                        #print(f"3D Coordinates: X: {point3D[0]:.2f} m, Y: {point3D[1]:.2f} m, Z: {point3D[2]:.2f} m")
                    else:
                        print("Green detected but depth data not available.")

            # Display the processed frame
            cv2.imshow("ZED | Green Color Detection", image_frame)

            # Exit on 'q' key press
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break    
            
            ratePublisher.sleep()
                       
        else:
            print("Failed to grab frame from ZED.")
            break

    # Release resources
    cv2.destroyAllWindows()
    zed.close()


if __name__ == "__main__":
    main()
