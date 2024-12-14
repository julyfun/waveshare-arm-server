import rclpy
from rclpy.node import Node
import tf2_ros
from rclpy.duration import Duration

class Tf2Echo(Node):
    def __init__(self, source_frame, target_frame):
        super().__init__('tf2_echo')
        self.source_frame = source_frame
        self.target_frame = target_frame

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                now,
                timeout=Duration(seconds=1.0)
            )
            self.get_logger().info(f"Transform from {self.source_frame} to {self.target_frame}:")
            self.get_logger().info(f"Translation: x={trans.transform.translation.x}, y={trans.transform.translation.y}, z={trans.transform.translation.z}")
            self.get_logger().info(f"Rotation: x={trans.transform.rotation.x}, y={trans.transform.rotation.y}, z={trans.transform.rotation.z}, w={trans.transform.rotation.w}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Could not transform {self.source_frame} to {self.target_frame}: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    source_frame = 'base_link'
    target_frame = 'link3'
    node = Tf2Echo(source_frame, target_frame)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()