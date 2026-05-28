import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose

def generate_test_description():
    nav2_traffic_node = launch_ros.actions.Node(
        package='next_gen_nav2_traffic',
        executable='nav2_traffic',
        name='nav2_traffic_node',
        output='screen'
    )

    return launch.LaunchDescription([
        nav2_traffic_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'nav2_traffic_node': nav2_traffic_node}

class TestPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_pipeline_node')
        self.received_inner_goal = False
        
        self.inner_action_server = ActionServer(
            self.node,
            NavigateToPose,
            '/robot0/inner/navigate_to_pose',
            self.inner_goal_callback
        )

    def tearDown(self):
        self.node.destroy_node()

    def inner_goal_callback(self, goal_handle):
        self.node.get_logger().info('Mock Inner Action Server received a goal!')
        self.received_inner_goal = True
        goal_handle.succeed()
        return NavigateToPose.Result()

    def test_pipeline_flow(self):
        client = ActionClient(self.node, NavigateToPose, '/robot0/navigate_to_pose')
        
        self.node.get_logger().info('Waiting for /robot0/navigate_to_pose action server...')
        client.wait_for_server(timeout_sec=10.0)
        self.assertTrue(client.server_is_ready(), "Action server not ready")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = 10.0
        goal_msg.pose.pose.position.y = 20.0
        
        self.node.get_logger().info('Sending goal to trigger pipeline...')
        send_goal_future = client.send_goal_async(goal_msg)
        
        while rclpy.ok() and not send_goal_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        self.assertTrue(send_goal_future.done(), "Goal send timed out")
        
        timeout = time.time() + 15.0
        self.node.get_logger().info('Waiting for pipeline to reach inner action server...')
        
        while time.time() < timeout and not self.received_inner_goal:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        self.assertTrue(
            self.received_inner_goal, 
            "Pipeline failed: Did not receive inner navigation goal on /robot0/inner/navigate_to_pose"
        )

if __name__ == '__main__':
    import pytest
    pytest.main([__file__])
