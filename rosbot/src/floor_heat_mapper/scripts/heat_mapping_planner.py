#! /usr/bin/env python3
import math
import time
import numpy as np
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, Trigger
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav2_msgs.srv import LoadMap

GOAL_POSES_TOPIC_NAME = 'goal_pose_markers'
MAP_TOPIC_NAME = 'map'
GLOBAL_COSTMAP_TOPIC_NAME = 'global_costmap/costmap'

MAP_FRAME_NAME = 'map'
BASE_LINK_FRAME_NAME = 'base_link'
TAKE_THERMAL_IMAGE_SERVICE_NAME = 'take_thermal_image'

MEASUREMENT_STEP_X = 0.15
MEASUREMENT_STEP_Y = 0.2
COSTMAP_THRESHOLD = 90


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


def get_cyan() -> ColorRGBA():
    color = ColorRGBA()
    color.a = 1.0
    color.g = 1.0
    color.b = 1.0
    return color


class HeatmappingPlanner(Node):
    def __init__(self) -> None:
        super().__init__('HeatMappingPlanner')
        self.get_logger().info('Init HeatMappingPlanner...')
        self.__initialize_variables()
        self.__get_robot_position()
        self.get_logger().info('Created HeatmappingPlanner! Waiting for /%s and /%s...'
                               % (MAP_TOPIC_NAME, GLOBAL_COSTMAP_TOPIC_NAME))
        self.send_load_map_request()
        self.get_logger().info('Restore old position...')
        # self.navigator.setInitialPose(self.robot_pose)

    def __initialize_variables(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_pose = PoseStamped()
        self.initial_robot_pose = PoseStamped()
        self.navigator = BasicNavigator()
        self.visited_indexes = []
        self.goal_poses = []
        self.closest_pose_index = 0
        self.robot_position = Point()
        self.got_robot_pose = False
        self.going_to_pose = False
        self.got_map = False
        self.got_global_costmap = False
        self.created_goal_poses = False
        self.goal_poses_marker = Marker()

        self.navigator.waitUntilNav2Active()

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.localization_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "amcl_pose",
            self.__amclPoseCallback,
            amcl_pose_qos,
        )

        self.map_msg = OccupancyGrid()
        self.global_costmap_msg = OccupancyGrid()

        self.goal_poses_marker_pub = self.create_publisher(
            Marker, GOAL_POSES_TOPIC_NAME, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, MAP_TOPIC_NAME, self.__map_callback, 10)
        self.global_costmap = self.create_subscription(
            OccupancyGrid, GLOBAL_COSTMAP_TOPIC_NAME, self.__globalcost_map_callback, 10)
        self.timer = self.create_timer(1, self.__timer_callback)
        self.take_thermal_image_cli = self.create_client(
            Trigger, TAKE_THERMAL_IMAGE_SERVICE_NAME)
        self.take_thermal_image_req = Trigger.Request()
        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.map_request = LoadMap.Request()
        while not self.map_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Waiting for service')

    def __request_thermal_image(self):
        self.future = self.take_thermal_image_cli.call_async(
            self.take_thermal_image_req)
        rclpy.spin_once(self)


    def send_load_map_request(self):
        self.get_logger().info('Loading map...')
        self.map_request.map_url = "/maps/map.yaml"
        wait = self.map_client.call_async(self.map_request)
        rclpy.spin_until_future_complete(self, wait)
        if wait.result() is not None:
            self.get_logger().info('Request was responded')
        else:
            self.get_logger().error('Request Failed')

    def __amclPoseCallback(self, msg):
        # self.robot_pose = msg.pose
        self.initial_pose_received = True
        return

    def __timer_callback(self):
        if self.got_global_costmap and self.got_map and not self.created_goal_poses:
            self.__create_goal_markers()
            self.__create_goal_poses()
            self.created_goal_poses = True
        if not len(self.goal_poses) or self.going_to_pose:
            return

        self.__get_robot_position()
        # self.__get_closest_pose_index()
        self.__active_goal_pose()
        self.__check_navigator_feedback()
        self.__check_navigator_result()
        self.got_robot_pose = False

    def __create_goal_poses(self) -> None:
        self.goal_poses = []
        x = 0
        orientation = Quaternion()
        orientation.w = 0.0
        orientation.z = 1.0

        for goal_point in self.goal_poses_marker.points:

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = MAP_FRAME_NAME
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_point.x
            goal_pose.pose.position.y = goal_point.y
            goal_pose.pose.position.z = 0.0
            if x != goal_pose.pose.position.x:
                pam = orientation.w
                orientation.w = orientation.z
                orientation.z = pam

            goal_pose.pose.orientation = orientation
            self.goal_poses.append(goal_pose)
            x = goal_pose.pose.position.x

        self.get_logger().info('Created %d goal poses' % len(self.goal_poses))

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
                               (closest_pose.pose.position.x, closest_pose.pose.position.y, closest_pose_index, min_distance))
        if self.closest_pose_index == closest_pose_index:
            self.navigator.lifecycleShutdown()
            self.get_logger().info('Job done!')
        self.closest_pose_index = closest_pose_index

    def __active_goal_pose(self):
        self.going_to_pose = True
        self.goal_poses_marker.colors[self.closest_pose_index] = get_yellow()
        self.goal_poses_marker_pub.publish(self.goal_poses_marker)
        self.navigator.goToPose(self.goal_poses[self.closest_pose_index])

    def __check_navigator_feedback(self):
        i = 0
        ignored = False
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info('Estimated time of arrival: %s seconds.' % '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9))
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=15.0):
                    self.navigator.cancelTask()
                    self.get_logger().info('Task canceled!')


    def __check_navigator_result(self):
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.goal_poses_marker.colors[self.closest_pose_index] = get_green(
            )
            self.goal_poses_marker_pub.publish(self.goal_poses_marker)
            self.visited_indexes.append(self.closest_pose_index)
            self.going_to_pose = False
            self.get_logger().info('Goal succeeded!')
            self.__request_thermal_image()
            time.sleep(2.0)

        elif result == TaskResult.CANCELED:
            self.goal_poses_marker.colors[self.closest_pose_index] = get_black(
            )
            self.goal_poses_marker_pub.publish(self.goal_poses_marker)
            self.visited_indexes.append(self.closest_pose_index)
            self.going_to_pose = False
            self.get_logger().info('Goal was canceled!')

        elif result == TaskResult.FAILED:
            self.goal_poses_marker.colors[self.closest_pose_index] = get_black(
            )
            self.goal_poses_marker_pub.publish(self.goal_poses_marker)
            self.visited_indexes.append(self.closest_pose_index)
            self.going_to_pose = False
            self.get_logger().info('Goal failed!')

        else:
            self.get_logger().info('Goal has an invalid return status!')
            self.goal_poses_marker.colors[self.closest_pose_index] = get_black(
            )
            self.goal_poses_marker_pub.publish(self.goal_poses_marker)
            self.visited_indexes.append(self.closest_pose_index)
            self.going_to_pose = False
            self.get_logger().info('Goal failed!')

        self.closest_pose_index += 1

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

    def __map_callback(self, msg):
        if self.got_map:
            return
        self.map_msg = msg
        self.got_map = True

    def __globalcost_map_callback(self, msg):
        if self.got_global_costmap:
            return
        self.global_costmap_msg = msg
        self.got_global_costmap = True

    def __create_goal_markers(self):
        self.goal_poses_marker.header.stamp = self.get_clock().now().to_msg()
        self.goal_poses_marker.header.frame_id = MAP_FRAME_NAME
        self.goal_poses_marker.type = Marker.POINTS
        self.goal_poses_marker.action = Marker.ADD
        self.goal_poses_marker.scale.x = 0.05
        self.goal_poses_marker.scale.y = 0.05
        self.goal_poses_marker.scale.z = 0.05
        self.goal_poses_marker.pose.position.z = 0.1
        self.goal_poses_marker.ns = 'goal_poses'
        self.goal_poses_marker.color = get_cyan()

        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        corner_x = origin_x + self.map_msg.info.width * self.map_msg.info.resolution
        corner_y = origin_y + self.map_msg.info.height * self.map_msg.info.resolution
        iter = -1
        for i in np.arange(origin_y, corner_y, MEASUREMENT_STEP_Y):
            iter += 1
            left = 0
            right = 0
            step = 0
            if iter%2 ==0:
                left = origin_x
                right = corner_x
                step = +MEASUREMENT_STEP_X
            else:
                left = corner_x
                right = origin_x
                step = -MEASUREMENT_STEP_X
            for j in np.arange(left, right, step):
                index_x = int((j - left)/self.map_msg.info.resolution)
                index_y = int((i - origin_y)/self.map_msg.info.resolution)
                index = index_x + index_y*self.map_msg.info.width

                if self.global_costmap_msg.data[index] > COSTMAP_THRESHOLD or self.map_msg.data[index] != 0:
                    continue
                point = Point()
                point.x = j
                point.y = i
                self.goal_poses_marker.points.append(point)
                self.goal_poses_marker.colors.append(get_cyan())

        self.goal_poses_marker_pub.publish(self.goal_poses_marker)
        self.get_logger().info('Created %d markers' %
                               len(self.goal_poses_marker.points))


def main(args=None):
    rclpy.init(args=args)
    heatmapping_planner = HeatmappingPlanner()
    rclpy.spin(heatmapping_planner)
    heatmapping_planner.navigator.destroyNode()
    heatmapping_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
