import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavigateToPoseActionClient(Node):

    def __init__(self):
        super().__init__('nav_to_pose_execute_python')

        self.status = False

        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self, position, orientation):
        goal_msg = NavigateToPose.Goal()
        self.status = False

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]

        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]

        goal_msg.pose = pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info(str(future))
        self.status = True
        # rclpy.shutdown()

    def get_nav2_status(self):
        return self.status

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(str(feedback_msg))


# def main(args=None):
#     rclpy.init(args=args)
#     action_client = NavigateToPoseActionClient()
#     rclpy.spin(action_client)
#
#
# if __name__ == '__main__':
#     main()
