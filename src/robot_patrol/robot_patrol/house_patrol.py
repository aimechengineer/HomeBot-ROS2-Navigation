#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import numpy as np

class HousePatrolNode(Node):  
    def __init__(self):
        super().__init__('house_patrol') 
        self.navigator = BasicNavigator()

        # Set initial pose
        initial_pose = self.create_pose_stamped(0.0, 0.0, 0.0)
        self.navigator.setInitialPose(initial_pose)

        # Wait for Nav2 to be active
        self.navigator.waitUntilNav2Active()

        # Set waypoints and start navigation
        waypoints = [
            self.create_pose_stamped(4.0, 2.5, np.radians(0)),
            self.create_pose_stamped(2.5, 2.5, np.radians(90)),
            self.create_pose_stamped(2.5, 7.0, np.radians(-130.60)),
            self.create_pose_stamped(-3.5, 0.0, np.radians(11.31)),
            self.create_pose_stamped(9.0, 2.5, np.radians(155.56)),
            self.create_pose_stamped(3.5, 5.0, np.radians(-125.00)),
            self.create_pose_stamped(0.0, 0.0, 0.0)  # Assuming the robot stops facing the original direction
        ]
        self.follow_waypoints(waypoints)

    def create_pose_stamped(self, position_x, position_y, orientation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = position_x
        pose.pose.position.y = position_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose

    def follow_waypoints(self, waypoints):
        self.navigator.followWaypoints(waypoints)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info('Navigation Feedback: %s' % feedback)
        result = self.navigator.getResult()
        self.get_logger().info('Navigation Result: %s' % result)

def main(args=None):
    rclpy.init(args=args)
    node = HousePatrolNode()  
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
