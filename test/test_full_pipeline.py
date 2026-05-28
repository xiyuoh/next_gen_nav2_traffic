import os
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_test_description():
    # Declare the use_zenoh launch argument
    declare_use_zenoh = DeclareLaunchArgument(
        'use_zenoh',
        default_value='false',
        description='Whether to launch zenohd node'
    )
    
    use_zenoh = LaunchConfiguration('use_zenoh')

    # 1. Simulation Launch
    simulation_launch = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'sp_demo_nav2_bringup', 'cloned_multi_tb3_simulation_launch.py',
            'robots:=robot0={x: 0.0, y: 5.0, yaw: 0.0}; robot1={x: 3.0, y: 5.0, yaw: 0.0};'
        ],
        output='screen'
    )

    # 2. nav2_traffic Node
    nav2_traffic_node = launch_ros.actions.Node(
        package='next_gen_nav2_traffic',
        executable='nav2_traffic',
        name='nav2_traffic_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 3. mapf_post cargo run
    mapf_post_dir = 'src/arjo/mapf_post'
    mapf_post_process = launch.actions.ExecuteProcess(
        cmd=['cargo', 'run', '--example', 'rest_api', '--', '-p', 'example_trajectories/2follower.csv'],
        cwd=mapf_post_dir,
        output='screen'
    )

    # 4. zenohd Node (Conditional)
    zenohd_node = launch_ros.actions.Node(
        package='rmw_zenoh_cpp',
        executable='rmw_zenohd',
        name='zenohd_node',
        output='screen',
        condition=IfCondition(use_zenoh)
    )

    return launch.LaunchDescription([
        declare_use_zenoh,
        simulation_launch,
        nav2_traffic_node,
        mapf_post_process,
        zenohd_node,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'simulation_launch': simulation_launch,
        'nav2_traffic_node': nav2_traffic_node,
        'mapf_post_process': mapf_post_process,
        'zenohd_node': zenohd_node
    }

class TestFullPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_full_pipeline_node')

    def tearDown(self):
        self.node.destroy_node()

    # launch_testing requires test methods to accept proc_info (and optionally proc_output)
    # as arguments to inject process information fixtures.
    def test_send_goals(self, proc_info):
        # Create clients for both robots
        client0 = ActionClient(self.node, NavigateToPose, 'robot0/navigate_to_pose')
        client1 = ActionClient(self.node, NavigateToPose, 'robot1/navigate_to_pose')

        self.node.get_logger().info('Waiting for action servers...')
        
        # Spin while waiting for servers to avoid blocking graph discovery
        timeout = time.time() + 90.0
        while rclpy.ok() and time.time() < timeout and not (client0.server_is_ready() and client1.server_is_ready()):
            rclpy.spin_once(self.node, timeout_sec=0.5)
            
        self.assertTrue(client0.server_is_ready(), "robot0 action server not ready")
        self.assertTrue(client1.server_is_ready(), "robot1 action server not ready")

        # Goal for robot0
        goal0 = NavigateToPose.Goal()
        goal0.pose.header.frame_id = 'map'
        goal0.pose.pose.position.x = 5.0
        goal0.pose.pose.position.y = 5.0
        goal0.pose.pose.orientation.w = 1.0

        # Goal for robot1
        goal1 = NavigateToPose.Goal()
        goal1.pose.header.frame_id = 'map'
        goal1.pose.pose.position.x = 8.0
        goal1.pose.pose.position.y = 5.0
        goal1.pose.pose.orientation.w = 1.0

        self.node.get_logger().info('Sending goal for robot0...')
        future0 = client0.send_goal_async(goal0)
        
        self.node.get_logger().info('Sending goal for robot1...')
        future1 = client1.send_goal_async(goal1)

        # Spin until both goals are accepted
        while rclpy.ok() and not (future0.done() and future1.done()):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(future0.done(), "Goal 0 send timed out")
        self.assertTrue(future1.done(), "Goal 1 send timed out")
        
        handle0 = future0.result()
        handle1 = future1.result()
        
        self.assertTrue(handle0.accepted, "Goal 0 was rejected")
        self.assertTrue(handle1.accepted, "Goal 1 was rejected")
        
        self.node.get_logger().info('Goals accepted, waiting for results...')
        
        result_future0 = handle0.get_result_async()
        result_future1 = handle1.get_result_async()
        
        # Wait for results with a timeout (e.g. 2 minutes for simulation to complete)
        timeout = time.time() + 120.0
        while rclpy.ok() and time.time() < timeout and not (result_future0.done() and result_future1.done()):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        self.assertTrue(result_future0.done(), "Goal 0 result timed out")
        self.assertTrue(result_future1.done(), "Goal 1 result timed out")
        
        result0 = result_future0.result()
        result1 = result_future1.result()
        
        self.assertEqual(result0.status, GoalStatus.STATUS_SUCCEEDED, "Goal 0 did not succeed")
        self.assertEqual(result1.status, GoalStatus.STATUS_SUCCEEDED, "Goal 1 did not succeed")
        
        self.node.get_logger().info('Both goals succeeded!')
