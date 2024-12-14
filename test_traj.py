import rclpy
import sys
import time
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/hand_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.cnt = 0

    def timer_callback(self):
        self.cnt += 1
        msg = JointTrajectory()
        msg.joint_names = ['base_link_to_link1', 'link1_to_link2', 'link2_to_link3']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, 1.0, 2.0 - self.cnt % 2]
        point.time_from_start.sec = 3  # Adjust the duration as needed

        msg.points.append(point)
        
        self.publisher_.publish(msg)
        self.get_logger().info('Published joint states: %s' % msg)

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

