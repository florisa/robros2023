from .pipe_detect import PipeDetect
from .number_detect import number_detect
from .wood_detect import wood_detect
from .ObstacleAvoider import ObstacleAvoider
from .moveit_ik import MoveitIKClientAsync as IK
from .moveit_action import MoveGroupActionClient as Moveit


from rclpy.executors import Executor
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster

import rclpy
import time
from rclpy.executors import SingleThreadedExecutor

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


angles = None
flag = False
tf = []

class Test(Node):
    def __init__(self):
        super().__init__('test')
        
        global tf
        tf = TransformStamped()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tfb_ = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.time_cb)
        self.tfs = []
    
            

    def time_cb(self):
        global flag
        try:

            #self.tf_buffer.wait_for_transform_async('gripper_centre', 'odom', rclpy.time.Time())
            ## Determine current gripper pose
            t = self.tf_buffer.lookup_transform('base_link', 'gripper_centre', rclpy.time.Time(), rclpy.duration.Duration(seconds=1))
            
            
            tf.header.frame_id = "base_link"
            tf.child_frame_id = "moved_gripper"
            self.stamp = self.get_clock().now().to_msg()
            ## Modify current gripper pose to reach target pose 
            tf.transform.translation.x = t.transform.translation.x 
            tf.transform.translation.y = t.transform.translation.y+0.5
            tf.transform.translation.z = t.transform.translation.z
            tf.transform.rotation = t.transform.rotation
            self.tfs = tf
            #tf.transform.translation.x = t.transform.translation.x-0.5
            #tf._child_frame_id = "moved_gripper2"
            self.tfb_.sendTransform(self.tfs)
            flag = True
        except(TransformException):
            self.get_logger().warn('not working')
            return
        
            



def main(args=None):
    global angles
    rclpy.init(args=args)
    test = Test()
    while not flag:
        rclpy.spin_once(test)
    print('bla')
    ik_solver = IK(tf)
    while angles is None:
        angles = ik_solver.send_request('moved_gripper')
    #pipeDetector = PipeDetect()
    #numberDetector = number_detect()
    #woodDetector = wood_detect()
    ######obstacleAvoider = ObstacleAvoider()
    ######rclpy.spin(obstacleAvoider)
    moveit = Moveit()
    target_angles = angles
    print(angles)
    moveit.send_goal(target_angles)
    while not moveit.get_moveit_status():
        rclpy.spin_once(moveit)



if __name__ == "__main__":
    main()
