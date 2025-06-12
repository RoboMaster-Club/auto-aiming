#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int16, String
import time
import math

NAV_STATUS = {
    'NOT_STARTED': 'NOT_STARTED',
    'NAVIGATING': 'NAVIGATING',
    'IDLING': 'IDLING'
}

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.nav_status_pub = self.create_publisher(String, 'nav_status', 10)
        self.feedback = None
        self.goal_handle = None
        self.result_future = None
        self.initial_pose_received = False
        self.goal_sent_time = None

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl_pose_cb, qos)

    def _amcl_pose_cb(self, msg):
        self.initial_pose_received = True

    def set_initial_pose(self, pose: PoseWithCovarianceStamped):
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info('Setting initial pose')
        self.initial_pose_pub.publish(pose)

    def wait_for_initial_pose(self):
        while not self.initial_pose_received:
            self.get_logger().info('Waiting for amcl_pose...')
            rclpy.spin_once(self, timeout_sec=0.5)

    def send_goal(self, pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.get_logger().info(f'Sending goal to: {pose.pose.position}')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return False

        self.result_future = self.goal_handle.get_result_async()
        self.goal_sent_time = time.time()
        return True

    def is_navigating(self):
        if not self.result_future:
            return False
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.1)
        return not self.result_future.done()

    def cancel_goal(self):
        if self.goal_handle:
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            self.get_logger().info("Canceled current goal")
            self.goal_handle = None
            self.result_future = None
            self.goal_sent_time = None

    def time_since_goal_sent(self):
        if self.goal_sent_time:
            return time.time() - self.goal_sent_time
        return None

    def publish_nav_status(self, status):
        self.nav_status_pub.publish(String(data=status))


class PoseScheduler(Node):
    def __init__(self, navigator: Navigator):
        super().__init__('pose_scheduler')
        self.navigator = navigator
        self.match_started = False
        self.start_time = None
        self.low_health = False
        self.goal_timeout = 20  # seconds

        self.named_poses = {
            "CENTER": self._make_pose([2.9, 1.0]),
            "FAR_WALL": self._make_pose([-0.583, 1.25]),
            "HOME": self._make_pose([-0.2, 0.0, 90]),
            "HALF_METER_FORWARD": self._make_pose([0.5, 0.0]),
            "HEAL": self._make_pose([0.7, -2.9, 90]),
        }

        self.pose_queue = {
            3: "FAR_WALL",
            10: "HOME",
            20: "HALF_METER_FORWARD",
            30: "FAR_WALL"
        }
        self.low_health_pose = self.named_poses["HEAL"]

        self.create_subscription(Bool, 'match_start', self._match_cb, 10)
        self.create_subscription(Int16, 'health', self._health_cb, 10)
        self.create_timer(0.5, self._tick)

    def _make_pose(self, lst):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = lst[0]
        pose.pose.position.y = lst[1]
        if len(lst) >= 3:
            yaw = lst[2]
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    def _match_cb(self, msg):
        if msg.data and not self.match_started:
            self.start_time = time.time()
            self.match_started = True
            self.get_logger().info('Match started')

    def _health_cb(self, msg):
        self.low_health = msg.data < 300

    def _tick(self):
        if not self.match_started:
            self.get_logger().info("Publishing (Match not started)")
            self.navigator.publish_nav_status(NAV_STATUS['NOT_STARTED'])
            return

        if self.low_health:
            self.navigator.send_goal(self.low_health_pose)
            return

       # THREE POSSIBLE RESULTS FOR A POSE:
       # 1. Pose succeeds
       # 2. Next pose in queue reaches time to run, current pose gets canceled and next gets sent
       # 3. Pose exceeds timeout without another being queued, cancel and wait for next pose

        # Cancel stale goals
        time_active = self.navigator.time_since_goal_sent()
        if time_active and time_active > self.goal_timeout:
            self.get_logger().warn("Goal timeout reached. Cancelling and moving on.")
            self.navigator.cancel_goal()
            return

       	# Check if we have reached time to queue a new goal
        # Does NOT wait for current pose to complete/cancel. Queues immediately when reaching specified time
        elapsed = time.time() - self.start_time
        for t, pose_name in sorted(self.pose_queue.items()):
            if elapsed >= t:
                if pose_name in self.named_poses:
                    self.navigator.cancel_goal()
                    self.get_logger().info(f' [*] Queueing next pose: time={t}, name={pose_name}')
                    self.navigator.send_goal(self.named_poses[pose_name])
                    del self.pose_queue[t]
                    break
                else:
                    self.get_logger().warn(f"Pose '{pose_name}' not found in named poses")

        if len(self.pose_queue) == 0:
            self.get_logger().info(" [*] All poses have been sent")

        self.get_logger().info(f'Current nav time: {elapsed} sec')

        if self.navigator.is_navigating():
            self.navigator.publish_nav_status(NAV_STATUS['NAVIGATING'])
            return
        else:
            self.navigator.publish_nav_status(NAV_STATUS['IDLING'])
            return


def main(args=None):
    rclpy.init(args=args)

    navigator = Navigator()
    # navigator.set_initial_pose(PoseWithCovarianceStamped())
    navigator.wait_for_initial_pose()

    scheduler = PoseScheduler(navigator)
    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    executor.add_node(scheduler)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()