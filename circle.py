import rclpy
from rclpy.node import Node
from roarm_moveit.srv import MoveCircleCmd

def main(args=None):
    rclpy.init(args=args)

    node = Node('move_circle_client')
    client = node.create_client(MoveCircleCmd, '/move_circle_cmd')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    request = MoveCircleCmd.Request()
    request.x = 0.2
    request.y = 0.0
    request.z = 0.0
    request.radius = 0.1

    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('Result: %r' % future.result())
    else:
        node.get_logger().error('Exception while calling service: %r' % future.exception())

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

