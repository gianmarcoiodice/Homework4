#! /usr/bin/env python3
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node

from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs
import yaml
from math import atan2, sqrt

base_rispetto_GAZEBO = None
T_FROM_BASE_TO_GAZEBO = None
T_FROM_ARUCO_TO_CAMERA = None
T_FROM_CAMERA_TO_BASE = None

# Percorso per il file YAML
package_share_path = get_package_share_directory('rl_fra2mo_description')
goal_yaml_path = os.path.join(package_share_path, 'config', 'ARUCO_DET.yaml')

# Carica i waypoint dal file YAML
with open(goal_yaml_path, "r") as file:
    waypoints = yaml.safe_load(file)
    print("Waypoints caricati:", waypoints)
    goal_order = [0] + list(range(1, 12)) + [12]  # Ordine dei waypoint

class VisionControlNode(Node):
    def __init__(self, navigator, goal_poses):
        super().__init__('vision_control_node')
        self.navigator = navigator
        self.goal_poses = goal_poses
        self.marker_state_available = False
        self.aruco_detected = False

        # Creazione del buffer TF2 e del listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Creazione del broadcaster per pubblicare la trasformazione dell'ArUco
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Creazione del subscriber per il marker
        self.marker_subscriber = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.marker_pose_callback,
            10
        )

    def publish_aruco_tf(self, position, orientation):
        """
        Pubblica la trasformazione dell'ArUco marker come TF.
        :param position: Lista [x, y, z] con la posizione del marker.
        :param orientation: Lista [qx, qy, qz, qw] con l'orientamento del marker.
        """
        t = tf2_ros.TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'  # Frame della base CON MAP FUNZIONA
        t.child_frame_id = 'aruco_marker'

        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        t.transform.rotation.x = orientation[0]
        t.transform.rotation.y = orientation[1]
        t.transform.rotation.z = orientation[2]
        t.transform.rotation.w = orientation[3]

        # Pubblica la trasformazione
        self.tf_broadcaster.sendTransform(t)

    def marker_pose_callback(self, msg):
        if not self.aruco_detected:
            T_FROM_CAMERA_TO_BASE = np.array([
                [0, 0, 1, 0],
                [-1, 0, 0, 0],
                [0, 1, 0, 0.05],
                [0, 0, 0, 1]
            ])

            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=2.0))

                self.p_map_to_base = np.array([transform.transform.translation.x,
                                               transform.transform.translation.y,
                                               transform.transform.translation.z])

                q_base = transform.transform.rotation
                quaternion_base = np.array([q_base.x, q_base.y, q_base.z, q_base.w])
                self.rot_map_to_base = R.from_quat(quaternion_base).as_matrix()

                rotation_90_neg = np.array([
                    [0, 1, 0],
                    [-1, 0, 0],
                    [0, 0, 1]
                ])

                self.rot_rover_in_map = np.dot(self.rot_map_to_base, rotation_90_neg)
                self.p_rover_in_map = np.array([-3.0, 3.5, 0.1]) + np.dot(rotation_90_neg, self.p_map_to_base)

                T_FROM_BASE_TO_GAZEBO = np.eye(4)
                T_FROM_BASE_TO_GAZEBO[:3, :3] = self.rot_rover_in_map
                T_FROM_BASE_TO_GAZEBO[:3, 3] = self.p_rover_in_map
            except tf2_ros.TransformException as ex:
                self.get_logger().error(f"Transform error: {ex}")

            self.aruco_detected = True
            self.marker_state_available = True

            aruco_x = msg.pose.position.x
            aruco_y = msg.pose.position.y
            aruco_z = msg.pose.position.z
            aruco_qx = msg.pose.orientation.x
            aruco_qy = msg.pose.orientation.y
            aruco_qz = msg.pose.orientation.z
            aruco_qw = msg.pose.orientation.w

            T_FROM_ARUCO_TO_CAMERA= np.eye(4)  # Crea una matrice identità 4x4
            T_FROM_ARUCO_TO_CAMERA[:3, :3] = R.from_quat([aruco_qx,aruco_qy,aruco_qz,aruco_qw]).as_matrix()  # Inserisce la rotazione nella parte 3x3
            T_FROM_ARUCO_TO_CAMERA[:3, 3] = [aruco_x,aruco_y,aruco_z]

            T_FROM_ARUCO_TO_GAZEBO=T_FROM_BASE_TO_GAZEBO @ T_FROM_CAMERA_TO_BASE @ T_FROM_ARUCO_TO_CAMERA

            print("T_FROM_CAMERA_TO_GAZEBO:",T_FROM_ARUCO_TO_GAZEBO)

            R_matrix = T_FROM_ARUCO_TO_GAZEBO[:3, :3]
            # Creazione dell'oggetto Rotation dalla matrice di rotazione
            rotation = R.from_matrix(R_matrix)
            # Conversione in quaternioni (ritorna un array [x, y, z, w])
            quaternion_FROM_ARUCO_TO_GAZEBO = rotation.as_quat()
            position_FROM_ARUCO_TO_GAZEBO=T_FROM_ARUCO_TO_GAZEBO[:3,3]

           # aruco_position_camera = np.array([aruco_x, aruco_y, aruco_z, 1])
          #  aruco_position_map = T_FROM_BASE_TO_GAZEBO @ T_FROM_CAMERA_TO_BASE @ aruco_position_camera
            aruco_x_map, aruco_y_map, aruco_z_map = T_FROM_ARUCO_TO_GAZEBO[:3,3]

            # Pubblica la trasformazione
            aruco_orientation = quaternion_FROM_ARUCO_TO_GAZEBO
            self.publish_aruco_tf([aruco_x_map, aruco_y_map, aruco_z_map], aruco_orientation)

            # Log per debug
            self.get_logger().info(f"ARUCO: {aruco_x_map}, {aruco_y_map},{aruco_z_map}")
            print("ROBOT:", T_FROM_BASE_TO_GAZEBO @ [0, 0, 0, 1])
            #print("ARUCO:", aruco_position_map)

            # Naviga verso l'ultimo goal
            self.navigator.followWaypoints([self.goal_poses[-1]])
            self.aruco_detected = True


def main():
    rclpy.init()
    navigator = BasicNavigator()

    def create_pose(transform):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = -(transform["position"]["y"] - 3.5)
        pose.pose.position.y = (transform["position"]["x"] + 3)
        pose.pose.position.z = transform["position"]["z"]
        pose.pose.orientation.x = transform["orientation"]["x"]
        pose.pose.orientation.y = transform["orientation"]["y"]
        pose.pose.orientation.z = transform["orientation"]["z"]
        pose.pose.orientation.w = transform["orientation"]["w"]
        return pose

    goal_poses = [create_pose(waypoints["waypoints"][i]) for i in goal_order]
    print("Goal poses:", goal_poses)

    navigator.waitUntilNav2Active(localizer="smoother_server")
    vision_control_node = VisionControlNode(navigator, goal_poses)
    navigator.followWaypoints([goal_poses[0]])

    rclpy.spin(vision_control_node)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'Executing current waypoint: {feedback.current_waypoint + 1}/{len(goal_poses)}')

        if vision_control_node.aruco_detected:
            navigator.cancelTask()
            break

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    vision_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
