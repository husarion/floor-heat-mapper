#! /usr/bin/env python3
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import  Point, PoseStamped
from std_msgs.msg import ColorRGBA
# from nav_msgs

import math

GOAL_POSES_TOPIC_NAME = 'goal_pose_markers'
MAP_FRAME_NAME = 'map'
BASE_LINK_FRAME_NAME = 'base_link'


def get_black() -> ColorRGBA():
    color = ColorRGBA()
    color.a = 1.0
    return color

def get_yellow() -> ColorRGBA():
    color = ColorRGBA()
    color.a = 1.0
    color.g = 1.0
    color.r = 1.0
    return color

def get_green() -> ColorRGBA():
    color = ColorRGBA()
    color.a = 1.0
    color.g = 1.0
    return color

def get_red() -> ColorRGBA():
    color = ColorRGBA()
    color.a = 1.0
    color.r = 1.0
    return color


class HeatmappingPlanner(Node):
    def __init__(self) -> None:
        super().__init__('HeatmappingPlanner')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_pose = PoseStamped()
        self.navigator = BasicNavigator()
        self.get_logger().info('Init HeatmappingPlanner...')
        self.navigator.waitUntilNav2Active()
        self.visited_indexes = []
        self.goal_poses = []
        self.closest_pose_index = 0
        self.robot_position = Point()
        self.got_robot_pose = False
        self.going_to_pose = False


        self.goal_poses_marker_sub = self.create_subscription(
            Marker, GOAL_POSES_TOPIC_NAME, self.__goal_poses_marker_callback, 10)
        self.goal_poses_marker_pub = self.create_publisher(
            Marker, GOAL_POSES_TOPIC_NAME, 10)
        self.timer = self.create_timer(1, self.__timer_callback)
        self.get_logger().info('Created HeatmappingPlanner!')


    def __timer_callback(self):
        if not len(self.goal_poses) or self.going_to_pose:
            return

        self.__get_robot_position()
        self.__get_closest_pose_index()
        self.__active_goal_pose()
        self.__check_navigator_feedback()
        self.__check_navigator_result()
        self.got_robot_pose = False


    def __goal_poses_marker_callback(self, msg) -> None:
        self.goal_poses_marker_ = msg
        self.goal_poses = []
        for goal_point in self.goal_poses_marker_.points:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_point.x
            goal_pose.pose.position.y = goal_point.y
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            self.goal_poses.append(goal_pose)

        self.get_logger().info('Got %d markers' %len(self.goal_poses))

    def __get_closest_pose_index(self):
        closest_pose_index = 0
        min_distance = 1000000.0
        closest_pose = PoseStamped()
        i = 0
        for goal_pose in self.goal_poses:
            distance = math.sqrt((self.robot_position.x - goal_pose.pose.position.x)*(self.robot_position.x - goal_pose.pose.position.x)
                                 + (self.robot_position.y - goal_pose.pose.position.y)*(self.robot_position.y - goal_pose.pose.position.y))
            if min_distance > distance and not i in self.visited_indexes:
                closest_pose_index = i
                min_distance = distance
                closest_pose = self.goal_poses[closest_pose_index]
            i += 1

        self.get_logger().info('Set goal pose x: %f, y: %f, index: %d, distance: %f' %
                               (closest_pose.pose.position.x,closest_pose.pose.position.y, closest_pose_index, min_distance))
        self.closest_pose_index = closest_pose_index

    def __active_goal_pose(self):
        self.going_to_pose = True
        self.goal_poses_marker_.colors[self.closest_pose_index] = get_yellow()
        self.goal_poses_marker_pub.publish(self.goal_poses_marker_)
        self.navigator.goToPose(self.goal_poses[self.closest_pose_index])

    def __check_navigator_feedback(self):
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info('Estimated time of arrival: %s seconds.' % '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9))
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

    def __check_navigator_result(self):
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.goal_poses_marker_.colors[self.closest_pose_index] = get_green()
            self.goal_poses_marker_pub.publish(self.goal_poses_marker_)
            self.visited_indexes.append(self.closest_pose_index)
            self.going_to_pose = False
            self.get_logger().info('Goal succeeded!')

        elif result == TaskResult.CANCELED:
            self.goal_poses_marker_.colors[self.closest_pose_index] = get_red()
            self.goal_poses_marker_pub.publish(self.goal_poses_marker_)
            self.visited_indexes.append(self.closest_pose_index)
            self.going_to_pose = False
            self.get_logger().info('Goal was canceled!')

        elif result == TaskResult.FAILED:
            self.goal_poses_marker_.colors[self.closest_pose_index] = get_red()
            self.goal_poses_marker_pub.publish(self.goal_poses_marker_)
            self.visited_indexes.append(self.closest_pose_index)
            self.going_to_pose = False
            self.get_logger().info('Goal failed!')

        else:
            self.get_logger().info('Goal has an invalid return status!')
            self.goal_poses_marker_.colors[self.closest_pose_index] = get_black()
            self.goal_poses_marker_pub.publish(self.goal_poses_marker_)
            self.visited_indexes.append(self.closest_pose_index)
            self.going_to_pose = False
            self.get_logger().info('Goal failed!')

    def __get_robot_position(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                MAP_FRAME_NAME,
                BASE_LINK_FRAME_NAME,
                now)
            self.robot_position.x = trans.transform.translation.x
            self.robot_position.y = trans.transform.translation.y
            self.get_logger().info('Robot position: (%f,%f)' %
                                (self.robot_position.x, self.robot_position.y))

            robot_pose = PoseStamped()
            robot_pose.pose.position = self.robot_position
            robot_pose.header.frame_id = MAP_FRAME_NAME
            robot_pose.header.stamp = self.navigator.get_clock().now().to_msg()

            self.got_robot_pose = True

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {MAP_FRAME_NAME} to {BASE_LINK_FRAME_NAME}: {ex}')
            return


def main(args=None):
    rclpy.init(args=args)

    heatmapping_planner = HeatmappingPlanner()

    rclpy.spin(heatmapping_planner)
    heatmapping_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
