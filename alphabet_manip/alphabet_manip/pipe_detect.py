#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2 
import numpy as np
import statistics
import time



class PipeDetect(Node):
    def __init__(self) -> None:
        super().__init__('pipe_detect_node')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/Spot/gripper_camera/image_color', self.__image_callback, 1)
        self.Mx=[]
        self.My=[]

    def __image_callback(self, msg):
###############################################################
### Color segmentation

        #cv_image = self.bridge.imgmsg_to_cv2(msg)
        #original = cv_image.copy()
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #h, w, _ = original.shape
        #r_low = np.array([0, 0, 20], dtype="uint8")
        #r_up = np.array([100, 20, 150], dtype="uint8")
        #mask2 = cv2.inRange(cv_image, r_low, r_up)
        #cnts2 = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #cnts2 = cnts2[0] if len(cnts2) == 2 else cnts2[1]
#
        #for c in cnts2:
        #    x,y,w,h = cv2.boundingRect(c)
        #    cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 2)
        #    M = cv2.moments(c)
        #    if M['m00'] > 0:
        #        cx = int(M['m10']/M['m00'])
        #        cy = int(M['m01']/M['m00'])
        #        self.Mx.append(cx)
        #        self.My.append(cy)
        #cv2.imshow("Resulting_image", original)
        #cv2.waitKey(1)
###############################################################
### Edge Detection: Canny-Edge + tresholding


    #    color_im = self.bridge.imgmsg_to_cv2(msg)
    #    gray_im = cv2.cvtColor(color_im, cv2.COLOR_BGR2GRAY)
    #    edges = cv2.Canny(gray_im, 150, 200)
    #    # Apply adaptive thresholding to separate gray pixels
    #    _, imagethreshold = cv2.threshold(gray_im, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    #    # Combine the edges and threshold images
    #    combined = cv2.bitwise_and(edges, imagethreshold)
    #    imagecontours, _ = cv2.findContours(combined, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #    #for each of the contours detected, the shape of the contours is approximated using approxPolyDP() function and the contours are drawn in the image using drawContours() function
    #    for count in imagecontours:
    #        epsilon = 0.01 * cv2.arcLength(count, True)
    #        approximations = cv2.approxPolyDP(count, epsilon, True)
    #        cv2.drawContours(color_im, [approximations], 0, (0), 3)
    #    #the name of the detected shapes are written on the image
    #        i, j = approximations[0][0]
    #        if len(approximations) == 3:
    #            pass
    #        elif len(approximations) == 4:
    #            pass
    #        elif len(approximations) == 5:
    #            pass
    #        elif 6 < len(approximations) < 15:
    #            pass
    #            #cv2.putText(color_im, "Ellipse", (i, j), cv2.FONT_HERSHEY_COMPLEX, 1, 0, 2)
    #        else:
    #            cv2.putText(color_im, "Circle", (i, j), cv2.FONT_HERSHEY_COMPLEX, 1, 0, 2)
    #    cv2.imshow("Resulting_image", color_im)
    #    cv2.waitKey(1)
    ##############################################################
    ## Edge Detection: Hough Circles

    
        cimg = self.bridge.imgmsg_to_cv2(msg)
        img = cv2.cvtColor(cimg,cv2.COLOR_BGR2GRAY)
        img = cv2.medianBlur(img,5)
        circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,50,
                            param1=50,param2=60,minRadius=30,maxRadius=60)
        
        circles = np.uint16(np.around(circles))
        print('#############################')
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

        cv2.imshow('detected circles',cimg)
        cv2.waitKey(1)
        


def main(args=None):
    rclpy.init(args=args)
    node = PipeDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
