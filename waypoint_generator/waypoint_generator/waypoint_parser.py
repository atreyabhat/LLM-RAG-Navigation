import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from rclpy.action import ActionClient
import json

import numpy as np

class Nav2WaypointsSender(Node):
    def __init__(self):
        super().__init__('nav2_waypoints_sender')

        # Action client for NavigateToPose
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoint_subscription = self.create_subscription(String, 'llm_waypoint', self.waypoint_callback, 10)
        self.waypoints = []
        self.current_index = 0

    def waypoint_callback(self, msg):
        self.get_logger().info(f'Waypoint received: {msg.data}')
        data = msg.data
        # convert string to dictionary
        data = json.loads(data)
        print(type(data))
        # convert dictionary to PoseStamped
        waypoints = create_pose_from_dict(data)
        self.set_waypoints(waypoints)
        self.send_next_goal()

    def set_waypoints(self, waypoints): 
        self.waypoints = waypoints

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!')
            # rclpy.shutdown()
            self.current_index = 0
            return

        if not self.nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose server not available!')
            rclpy.shutdown()
            return

        # Create and send the next goal
        pose = self.waypoints[self.current_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f'Sending goal {self.current_index + 1} to: '
            f'({pose.pose.position.x}, {pose.pose.position.y})'
        )
        self.nav_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(f'Feedback: {feedback.feedback.current_pose}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal {self.current_index + 1} was rejected!')
            self.current_index += 1
            self.send_next_goal()
            return

        self.get_logger().info(f'Goal {self.current_index + 1} accepted! Waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info(f'Goal {self.current_index + 1} reached successfully!')
        # result = future.result()
        # print(result)
        # if result.status == GoalStatus.STATUS_SUCCEEDED:
        #     self.get_logger().info(f'Goal {self.current_index + 1} reached successfully!')
        # else:
        #     self.get_logger().error(
        #         f'Goal {self.current_index + 1} failed with status: {result.status}'
        #     )

        # Move to the next waypoint
        self.current_index += 1
        self.send_next_goal()


def create_pose(x, y, theta):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rclpy.time.Time().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    pose.pose.orientation.z = np.sin(theta / 2.0)  # Quaternion calculation
    pose.pose.orientation.w = np.cos(theta / 2.0)

    return pose


def create_pose_from_dict(position_dict):
    poses = []
    for name, (x, y, theta) in position_dict.items():
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rclpy.time.Time().to_msg()

        # Set position
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Set orientation (convert theta to quaternion)
        pose.pose.orientation.z = np.sin(theta / 2.0)
        pose.pose.orientation.w = np.cos(theta / 2.0)

        poses.append(pose)
    return poses


def main(args=None):
    rclpy.init(args=args)

    # Create the Nav2WaypointsSender node
    waypoints_sender = Nav2WaypointsSender()

    # # Define waypoints dictionary
    # waypoint_dict = {
    #     'move1': (9.0, -3.5, 0.0),  # Moving to Kitchen
    #     'move2': (2.0, -1.5, 0.0),  # Moving to Hall
    #     'move3': (4.0, 0.5, 1.57),  # Moving to Bedroom
    # }

    # # Convert dictionary to list of PoseStamped
    # waypoint_poses = create_pose_from_dict(waypoint_dict)

    # # Set waypoints and start navigation
    # waypoints_sender.set_waypoints(waypoint_poses)
    # waypoints_sender.send_next_goal()

    rclpy.spin(waypoints_sender)


if __name__ == '__main__':
    main()

