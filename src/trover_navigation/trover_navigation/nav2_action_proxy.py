#!/usr/bin/env python3
"""
Action proxy that intercepts NavigateToPose goals from rviz2,
fixes the timestamp issue, and forwards to the real bt_navigator.
This allows the Nav2 panel in rviz2 to work with simulation time.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from builtin_interfaces.msg import Time
import threading


class Nav2ActionProxy(Node):
    def __init__(self):
        super().__init__('nav2_action_proxy')

        self.callback_group = ReentrantCallbackGroup()

        # Client to forward goals to real bt_navigator
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'bt_navigator/navigate_to_pose',
            callback_group=self.callback_group
        )

        # Server to receive goals from rviz2
        self.nav_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Nav2 Action Proxy started')
        self.get_logger().info('  Listening on: navigate_to_pose')
        self.get_logger().info('  Forwarding to: bt_navigator/navigate_to_pose')

    def execute_callback(self, goal_handle):
        self.get_logger().info(
            f'Received goal: ({goal_handle.request.pose.pose.position.x:.2f}, '
            f'{goal_handle.request.pose.pose.position.y:.2f})'
        )

        # Wait for the real action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('bt_navigator action server not available')
            goal_handle.abort()
            return NavigateToPose.Result()

        # Fix the timestamp - set to 0 to use latest transform
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_handle.request.pose
        goal_msg.pose.header.stamp = Time(sec=0, nanosec=0)
        goal_msg.behavior_tree = goal_handle.request.behavior_tree

        self.get_logger().info('Forwarding goal with fixed timestamp...')

        # Send goal to real bt_navigator
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: self.feedback_callback(goal_handle, fb)
        )

        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, send_goal_future)
        client_goal_handle = send_goal_future.result()

        if not client_goal_handle.accepted:
            self.get_logger().error('Goal rejected by bt_navigator')
            goal_handle.abort()
            return NavigateToPose.Result()

        self.get_logger().info('Goal accepted by bt_navigator')

        # Wait for result
        result_future = client_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded')
            goal_handle.succeed()
        else:
            self.get_logger().warn(f'Navigation failed with status: {result.status}')
            goal_handle.abort()

        return result.result

    def feedback_callback(self, goal_handle, feedback_msg):
        # Forward feedback to rviz2
        goal_handle.publish_feedback(feedback_msg.feedback)


def main():
    rclpy.init()
    node = Nav2ActionProxy()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
