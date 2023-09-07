import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2 
import numpy as np
from ultralytics import YOLO
import os

from ament_index_python.packages import get_package_share_directory



class number_detect(Node):
    def __init__(self):
        super().__init__('number_detector')

        model_path = os.path.join(get_package_share_directory('alphabet_manip'), 'data/mnist.pt')
        self.data_path = os.path.join(get_package_share_directory('alphabet_manip'), 'data//MNIST.v8i.yolov5pytorch/data.yaml')
        self.model = YOLO(model_path)
        self.br = CvBridge()
        self.create_subscription(Image, '/Spot/gripper_camera/image_color', self.__image_callback, 1)
        self.Mx = []
        self.My = []
        
    def __image_callback(self, msg):
        image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(image, stream=True)
        for _, a in enumerate(results):
            data = a.boxes.data
            if data.numpy().size == 0:
                print('Nothing detected')
            else:
                x = int(data[0,0].numpy())
                y = int(data[0,1].numpy())
                w = int(data[0,2].numpy())
                h = int(data[0,3].numpy())
                conf = data[0,4].numpy()
                det_num = int(data[0,5].numpy())
                print(det_num)
                startP = (x, y)
                endP = (w, h)
                lw = max(round(sum(image.shape) / 2 * 0.003), 2)
                thick = max(lw - 1, 1)
                cv2.rectangle(image, startP, endP, color=(0,255,0), thickness=2)
                cv2.putText(image, "Detected number: " + str(det_num) + "   Confidence: " + str(conf), (x, y),0,lw / 3,(0, 100, 0),thickness=thick,lineType=cv2.LINE_AA)
            
        cv2.imshow('image', image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    detector = number_detect()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()