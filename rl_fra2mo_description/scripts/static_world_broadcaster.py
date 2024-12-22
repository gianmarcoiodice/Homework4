#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class StaticWorldBroadcaster(Node):
    def __init__(self):
        super().__init__('static_world_broadcaster')
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Definizione della trasformazione
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'world'  # Frame genitore
        static_transform.child_frame_id = 'map'     # Frame figlio

        # Traslazione (offset noto tra world e map)
        static_transform.transform.translation.x = -3.00
        static_transform.transform.translation.y = 3.50
        static_transform.transform.translation.z = 0.10

        # Rotazione (nessuna rotazione tra world e map)
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = -0.7071
        static_transform.transform.rotation.w = 0.7071

        # Pubblica la trasformazione
        self.static_broadcaster.sendTransform(static_transform)
        self.get_logger().info('Static transform between world and map published.')

def main(args=None):
    rclpy.init(args=args)
    node = StaticWorldBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
