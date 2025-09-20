import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


def parse_args():
    parser = argparse.ArgumentParser(
        description='Publish a 2D goal (X forward, Y left) to /goal_point'
    )
    parser.add_argument('x', type=float, help='Goal X (meters, forward)')
    parser.add_argument('y', type=float, help='Goal Y (meters, left)')
    parser.add_argument('--count', type=int, default=3, help='Times to publish the goal [default: 3]')
    parser.add_argument('--delay', type=float, default=0.1, help='Delay between publishes in seconds [default: 0.1]')
    parser.add_argument('--topic', type=str, default='/goal_point', help='Goal topic name [default: /goal_point]')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Node('set_goal_cli')
    pub = node.create_publisher(Point, args.topic, 10)

    msg = Point(x=args.x, y=args.y, z=0.0)
    node.get_logger().info(
        f"Publishing goal to {args.topic}: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f} (count={args.count})"
    )

    # Publish a few times to improve chance of delivery
    for _ in range(max(1, args.count)):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(max(0.0, args.delay))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
