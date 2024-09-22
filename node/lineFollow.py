#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

pub = None
flag = 1

section1End = 200
section2End = 600
section3End = 800

count1 = 0
count2 = 0
count3 = 0


# Callback function to process the image data
def image_callback(ros_image):
    bridge = CvBridge()
    global pub
    global flag
    global section1End,section3End,section2End,count1,count2,count3
    

    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")

        #CAN I PUT MOVEMENT ADJUSTMENTS HERE?
        move = Twist()  # Create a new Twist message to adjust movement

        # Example movement logic (replace with your actual logic):
        # If some condition based on the image is met (e.g., detecting a line), adjust movement


        height, width, _ = cv_image.shape

        # Prepare text strings
        #width_text = f"Width = {width}"
        #height_text = f"Height = {height}"

        # Draw the width and height on the image
        # cv.putText(cv_image, width_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
        # cv.putText(cv_image, height_text, (10, 70), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)


        blurred = cv.GaussianBlur(cv_image, (5, 5), 0)

        
        # Reset counts for each image
        count1 = 0
        count2 = 0
        count3 = 0


        # Define the target color in BGR (R162, G124, B70)
        target_color_bgr = np.array([70, 124, 162])  # BGR format

        # Define a color range around the target color
        lower_bound = target_color_bgr - np.array([10, 10, 10])  # Lower range
        upper_bound = target_color_bgr + np.array([10, 10, 10])  # Upper range

        # Create a mask based on the defined color range
        mask = cv.inRange(blurred, lower_bound, upper_bound)

        # Specify the height (row) you want to read
        row_index = 650  # For example, the 100th row (adjust based on your image)

          # Get the pixel values for that specific row
        row_pixels = mask[row_index, :]  # ':' means all columns
        #print(f"Row Pixels: {row_pixels}")

        for index, pixel_value in enumerate(row_pixels):
            if (pixel_value == 255):
                if (index < section1End) and (index > 0):  # Check if pixel value is 0
                    count1 +=1

                if (index < section2End) and (index > section1End):  # Check if pixel value is 0
                    count2 +=1

                if (index < section3End) and (index > section2End):  # Check if pixel value is 0
                    count3 +=1

        count1_text = f"Count 1 = {count1}"
        count2_text = f"Count 2 = {count2}"
        count3_text = f"Count 3 = {count3}"

        cv.putText(mask, count1_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
        cv.putText(mask, count2_text, (10, 70), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
        cv.putText(mask, count3_text, (10, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)



        if (count2>140):
            move.linear.x = (1)
            move.angular.z = 0
            pub.publish(move)

        elif (count1 > 10):
            move.angular.z = 2 # Slight turn to adjust # 0.5
            move.linear.x=(0.3)
            pub.publish(move)
            flag = 1 #remembers which side it went off at

        elif (count3 > 10):
            move.angular.z = -2  # Slight turn to adjust 0.5
            move.linear.x=(0.3)
            pub.publish(move)
            flag = 3 #remembers which side it went off at

        else: #if way off path, turn back on
            move.linear.x = 0.3
            if (flag == 3):
                move.angular.z = -2  # Slight turn to adjust
            elif (flag == 1):
                move.angular.z = 2  # Slight turn to adjust
            pub.publish(move)
        
        
        # while (line perfectly not in middle)
        #     if (more stuff on left)
        #         adjust right
        #      if (more stuff on right)
        #         adjust leftc
        
        # Display the image using OpenCV
        cv.imshow("Camera Image", mask)
        cv.waitKey(1)
        
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    global pub
    # Initialize the ROS node
    rospy.init_node('camera_subscriber', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Subscribe to the camera's image topic
    image_topic = "/robot/camera1/image_raw"  # Make sure this matches the topic from your camera plugin
    rospy.Subscriber(image_topic, Image, image_callback)

    # Keep the node alive until it's stopped
    rospy.spin()

    # Close any OpenCV windows when the node is stopped
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
