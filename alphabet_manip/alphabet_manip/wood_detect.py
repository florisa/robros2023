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



class wood_detect(Node):
    def __init__(self):
        super().__init__('wood_detector')
        self.create_subscription(Image, '/Spot/gripper_camera', self.__image_callback, 1)
        self.__walkpub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.twist = Twist()
        self.Mx = []
        self.My = []
        self.br = CvBridge()


    def __image_callback(self, message):
        
        cv_image = self.br.imgmsg_to_cv2(message)
        original = cv_image.copy()
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, _ = original.shape
        r_low = np.array([10, 100, 20], dtype="uint8")
        r_up = np.array([20, 225, 200], dtype="uint8")
        mask2 = cv2.inRange(cv_image, r_low, r_up)
        cnts2 = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts2 = cnts2[0] if len(cnts2) == 2 else cnts2[1]

        for c in cnts2:
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 2)
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                self.Mx.append(cx)
                self.My.append(cy)

        '''
        if not self.Mx:
            self.twist.angular.z = 0.5
            self.__walkpub.publish(self.twist)
            time.sleep(5)
            print('hi')
        else:
            cx_mean = statistics.mean(self.Mx)
            cy_mean = statistics.mean(self.My)
            cv2.circle(original, (int(cx_mean), int(cy_mean)), 5, (255,0,0), -1)
            M = cv2.moments(mask2)
            if M['m00'] > 0:
                self.cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(original, (self.cx, cy), 5, (255,0,0), -1)
                self.err = self.cx - cx_mean
                self.twist.linear.x = 1.0
                self.twist.angular.z = -float(self.err) / 10
                self.__walkpub.publish(self.twist)
            cv2.imshow('original', original)
            cv2.waitKey(5)
            if not cnts2:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.__walkpub.publish(self.twist)
                self.create_timer(3, self.spot_callback)
        '''
            
        cv2.imshow('original', original)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    detector = wood_detect()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()