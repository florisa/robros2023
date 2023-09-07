import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.create_subscription(LaserScan, 'scan', self.__scan_callback, qos_profile=qos_policy)
        self.movepub = self.create_publisher(Twist, 'cmd_vel', 10)

    def __scan_callback(self, message: LaserScan):
        walk_message = Twist()
        self.maxang = message.angle_max
        self.minang = message.angle_min
        self.ranges=message.ranges
        index = int(len(self.ranges)*20/360)
        mid_index = int((len(self.ranges)/2)+0.5)
        self.ranges = self.ranges[mid_index-index:mid_index+index]
        for i in self.ranges:
            if i <= 1.5 or len(set(self.ranges)) == 1:
                walk_message.linear.x = 0.0
                self.movepub.publish(walk_message)
            else:
                walk_message.linear.x = 0.8
                self.movepub.publish(walk_message)
        
        #print(self.ranges)
def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
