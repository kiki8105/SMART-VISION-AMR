"""Forward planned paths to Nav2's FollowPath controller action."""

import rclpy
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json

from smart_vision_amr.path_geometry import paths_are_equivalent


class PathFollower(Node):
    def __init__(self):
        super().__init__("path_follower")
        self.declare_parameter("action_name", "/follow_path")
        self.declare_parameter("controller_id", "")
        self.declare_parameter("goal_checker_id", "")
        self.client = ActionClient(self, FollowPath, self.get_parameter("action_name").value)
        self.status_pub = self.create_publisher(String, "/smart_vision_amr/navigation_status", 10)
        self.active_goal = None
        self.goal_request_pending = False
        self.pending_path = None
        self.cancel_requested = False
        self.cancel_when_accepted = False
        self.latest_path_key = None
        self.active_path = None
        self.create_subscription(Path, "/smart_vision_amr/path", self.on_path, 10)
        self.create_service(Trigger, "/smart_vision_amr/cancel_navigation", self.cancel_navigation)

    def cancel_navigation(self, request, response):
        if self.active_goal is None:
            response.success, response.message = True, "No active navigation goal"
            return response
        self.active_goal.cancel_goal_async()
        response.success, response.message = True, "Cancellation requested"
        return response

    def on_path(self, path):
        if not path.poses:
            self.get_logger().error("Planner reported no path; cancelling active navigation")
            self.pending_path = None
            self.latest_path_key = None
            self.active_path = None
            if self.active_goal is not None:
                self._cancel_active_goal()
            elif self.goal_request_pending:
                self.cancel_when_accepted = True
            message = String()
            message.data = json.dumps({"status": "FAILED", "message": "Planner returned no path"})
            self.status_pub.publish(message)
            return
        key = self._path_key(path)
        if key == self.latest_path_key:
            return
        self.latest_path_key = key
        if self.goal_request_pending or self.active_goal is not None:
            reference = self.pending_path or self.active_path
            if reference is not None and paths_are_equivalent(
                    self._points(reference), self._points(path)):
                self.get_logger().debug("Keeping active FollowPath goal; update only advances along the same route")
                return
            self.pending_path = path
            self._cancel_active_goal()
            return
        self._send_path(path)

    @staticmethod
    def _path_key(path):
        return tuple((round(p.pose.position.x, 3), round(p.pose.position.y, 3),
                      round(p.pose.orientation.z, 3), round(p.pose.orientation.w, 3))
                     for p in path.poses)

    @staticmethod
    def _points(path):
        return [(p.pose.position.x, p.pose.position.y) for p in path.poses]

    def _cancel_active_goal(self):
        if self.active_goal is not None and not self.cancel_requested:
            self.cancel_requested = True
            self.active_goal.cancel_goal_async()

    def _send_path(self, path):
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 FollowPath action is unavailable")
            self.publish_status("FAILED", "Nav2 FollowPath action is unavailable")
            return
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.get_parameter("controller_id").value
        goal.goal_checker_id = self.get_parameter("goal_checker_id").value
        self.active_path = path
        self.goal_request_pending = True
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)
        self.get_logger().info(f"Sent path with {len(path.poses)} poses to Nav2")

    def on_goal_response(self, future):
        self.goal_request_pending = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected FollowPath goal")
            if self.pending_path is None:
                self.publish_status("FAILED", "Nav2 rejected FollowPath goal")
            self.cancel_when_accepted = False
            self._send_pending_path()
            return
        self.active_goal = goal_handle
        if self.pending_path is not None or self.cancel_when_accepted:
            self._cancel_active_goal()
        goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, feedback):
        pass

    def on_result(self, future):
        result = future.result()
        self.get_logger().info(f"FollowPath finished with status {result.status}")
        from action_msgs.msg import GoalStatus
        status = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                  GoalStatus.STATUS_CANCELED: "CANCELED"}.get(result.status, "FAILED")
        if self.pending_path is None:
            self.publish_status(status, f"Nav2 status {result.status}")
        self.active_goal = None
        self.cancel_requested = False
        self.cancel_when_accepted = False
        self._send_pending_path()

    def _send_pending_path(self):
        if self.pending_path is None or self.active_goal is not None or self.goal_request_pending:
            return
        path, self.pending_path = self.pending_path, None
        self._send_path(path)

    def publish_status(self, status, message_text):
        message = String()
        message.data = json.dumps({"status": status, "message": message_text})
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
