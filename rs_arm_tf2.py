import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')

        # Define the translation and rotation
        # x: down
        # y: back
        # z: left
        translation = (-0.13151, +0.01990, +1.5e-2)
        ang = (90.0 - 67.65) / 180.0 * np.pi
        rotation_matrix = np.array(
            [
                [0, np.cos(ang), np.sin(ang)],
                [0, np.sin(ang), -np.cos(ang)],
                [-1, 0, 0]
            ]
        )
        
        # Convert rotation matrix to quaternion
        rotation = R.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()

        # Create a TransformStamped message
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'link3'
        transform_stamped.child_frame_id = 'rs_arm'
        transform_stamped.transform.translation.x = translation[0]
        transform_stamped.transform.translation.y = translation[1]
        transform_stamped.transform.translation.z = translation[2]
        transform_stamped.transform.rotation.x = quaternion[0]
        transform_stamped.transform.rotation.y = quaternion[1]
        transform_stamped.transform.rotation.z = quaternion[2]
        transform_stamped.transform.rotation.w = quaternion[3]

        # Create a static transform broadcaster
        broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        broadcaster.sendTransform(transform_stamped)
        print('ok')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
