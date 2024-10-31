#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

# Initialize the ROS node
rospy.init_node('robot_controller_with_camera')

# rospy.init_node('topic_publisher')

#create our topic publisher
rospy.loginfo("Node initialized")
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.loginfo("Publisher created")

# Create a CvBridge object for converting ROS Image messages to OpenCV format
bridge = CvBridge()

rate = rospy.Rate(2)
move = Twist()
# move.linear.x = 0.5
# move.angular.z = 0.5

# PID parameters
Kp = 0.005   # Proportional gain
Ki = 0.0    # Integral gain
Kd = 0.01  # Derivative gain

# Initialize PID variables
integral = 0
derivative = 0
control_signal = 0
previous_error = 0
cx = 0
cy = 0


# Callback function to process the image data from the camera
def image_callback(msg):
    global integral, previous_error, derivative, control_signal, cx, cy
    try:
        # Convert the ROS Image message to a format OpenCV understands
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Get the resolution of the image
        height, width, _ = cv_image.shape

        # Now you can use `width` and `height` to set the screen center
        screen_center_x = width // 2
        screen_center_y = height // 2

        # Convert frame to grayscale
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

        # Apply BW filter with threshold - contains NOT road
        # Things brighter than the certain gray value 100 turned to white
        _, road_mask = cv.threshold(gray, 100, 255, cv.THRESH_BINARY)  # Adjust threshold value as needed

        # Invert the mask (contains ROAD)
        road_mask = cv.bitwise_not(road_mask)

        # Find contours in the road mask
        contours, _ = cv.findContours(road_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour (assuming the road is the largest detected area)
            largest_contour = max(contours, key=cv.contourArea)

            # Calculate moments of the largest contour
            moments = cv.moments(largest_contour)

            if moments["m00"] != 0:
                # Calculate center of mass of the grey road mask
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])

                # Draw a red circle at the centroid
                radius = 10  # Adjust radius as needed
                cv.circle(cv_image, (cx, cy), radius, (0, 0, 255), -1)  #(0,0,255) = RED
            
            error_x = screen_center_x - cx

            # PID calculations
            integral += error_x
            derivative = error_x - previous_error
            control_signal = Kp * error_x + Ki * integral + Kd * derivative
            previous_error = error_x

            # Adjust angular velocity based on control signal
            move.angular.z = control_signal  # Negative to correct steering direction

            # Optionally adjust linear speed for forward motion
            move.linear.x = 0.1
        
        # Display the image in a window
        cv.imshow("Camera View", cv_image)
        cv.waitKey(1)  # Allows OpenCV to refresh the window
    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

# Subscribe to the camera topic
rospy.Subscriber('/rrbot/camera1/image_raw', Image, image_callback)

while not rospy.is_shutdown():
    pub.publish(move)
    rate.sleep()
