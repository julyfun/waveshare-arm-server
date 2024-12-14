import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener_node')
        
        # Create a buffer and listener for TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call the function to get the transform after a delay
        self.create_timer(1.0, self.get_transform)  # Wait for 1 second before attempting to get the transform

    def get_transform(self):
        try:
            # Wait for the transform from base_link to link3
            trans = self.tf_buffer.lookup_transform('base_link', 'link3', rclpy.time.Time())
            self.get_logger().info(f'Transform from base_link to link3:\n'
                                    f'Translation: x={trans.transform.translation.x}, '
                                    f'y={trans.transform.translation.y}, '
                                    f'z={trans.transform.translation.z}\n'
                                    f'Rotation: x={trans.transform.rotation.x}, '
                                    f'y={trans.transform.rotation.y}, '
                                    f'z={trans.transform.rotation.z}, '
                                    f'w={trans.transform.rotation.w}')
        except Exception as e:
            self.get_logger().error(f'Could not transform base_link to link3: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
