import argparse
import time
from pathlib import Path
from typing import List

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


def default_config_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        return str(Path(get_package_share_directory('point_nav')) / 'config' / 'goal_inputs.defaults.yaml')
    except Exception:
        return str(Path(__file__).resolve().parents[1] / 'config' / 'goal_inputs.defaults.yaml')


def parse_args():
    parser = argparse.ArgumentParser(description='Publish goals to point_nav with queue/override support.')

    parser.add_argument('x', nargs='?', type=float, help='Goal X in meters (forward)')
    parser.add_argument('y', nargs='?', type=float, help='Goal Y in meters (left)')
    parser.add_argument('--queue', action='store_true', help='Append goal(s) to queue (default behavior is override)')

    parser.add_argument('--throttle-scale', type=float, default=1.0, help='Per-goal throttle scale in (0,1]')
    parser.add_argument('--count', type=int, default=3, help='Times to publish each message [default: 3]')
    parser.add_argument('--delay', type=float, default=0.1, help='Delay between publishes [default: 0.1]')

    parser.add_argument('--goal-topic', type=str, default='/goal_point', help='Override goal topic')
    parser.add_argument('--queue-topic', type=str, default='/goal_point/add', help='Queue add topic')
    parser.add_argument('--estop-topic', type=str, default='/point_nav/estop', help='E-stop topic')

    parser.add_argument('--file', type=str, default='', help='YAML file containing goal_inputs list')

    parser.add_argument('--estop', action='store_true', help='Engage e-stop and halt rover immediately')
    parser.add_argument('--clear-estop', action='store_true', help='Release e-stop')

    return parser.parse_args()


def publish_bool(node: Node, topic: str, value: bool, count: int, delay: float) -> None:
    pub = node.create_publisher(Bool, topic, 10)
    msg = Bool(data=value)
    for _ in range(max(1, count)):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(max(0.0, delay))


def publish_point(node: Node, topic: str, x: float, y: float, throttle_scale: float, count: int, delay: float) -> None:
    pub = node.create_publisher(Point, topic, 10)
    msg = Point(x=float(x), y=float(y), z=float(max(0.05, min(1.0, throttle_scale))))
    for _ in range(max(1, count)):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(max(0.0, delay))


def load_goals_from_yaml(path: Path) -> List[Point]:
    if yaml is None:
        raise RuntimeError('PyYAML is unavailable; cannot read goal file.')

    data = yaml.safe_load(path.read_text()) or {}

    goal_inputs = None
    if isinstance(data, dict):
        if isinstance(data.get('goal_inputs'), list):
            goal_inputs = data.get('goal_inputs')
        elif isinstance(data.get('point_nav'), dict):
            pn = data.get('point_nav')
            if isinstance(pn.get('goal_inputs'), list):
                goal_inputs = pn.get('goal_inputs')
            elif isinstance(pn.get('ros__parameters'), dict):
                if isinstance(pn['ros__parameters'].get('goal_inputs'), list):
                    goal_inputs = pn['ros__parameters']['goal_inputs']

    goals: List[Point] = []
    if not isinstance(goal_inputs, list):
        return goals

    for item in goal_inputs:
        if not isinstance(item, dict):
            continue
        if 'x' not in item or 'y' not in item:
            continue
        throttle = float(item.get('throttle_scale', 1.0))
        goals.append(
            Point(
                x=float(item['x']),
                y=float(item['y']),
                z=float(max(0.05, min(1.0, throttle))),
            )
        )

    return goals


def main():
    args = parse_args()
    rclpy.init()
    node = Node('set_goal_cli')

    if args.estop and args.clear_estop:
        raise SystemExit('Choose only one of --estop or --clear-estop')

    if args.estop:
        publish_bool(node, args.estop_topic, True, args.count, args.delay)
        node.get_logger().warn(f'Published E-STOP=true to {args.estop_topic}')
        node.destroy_node()
        rclpy.shutdown()
        return

    if args.clear_estop:
        publish_bool(node, args.estop_topic, False, args.count, args.delay)
        node.get_logger().info(f'Published E-STOP=false to {args.estop_topic}')
        node.destroy_node()
        rclpy.shutdown()
        return

    goals: List[Point] = []

    file_path = args.file.strip() if args.file else ''
    if not file_path and args.x is None and args.y is None:
        file_path = default_config_path()

    if file_path:
        path = Path(file_path)
        if not path.exists():
            raise SystemExit(f'Goal file not found: {path}')
        goals.extend(load_goals_from_yaml(path))
        node.get_logger().info(f'Loaded {len(goals)} goals from {path}')

    if args.x is not None or args.y is not None:
        if args.x is None or args.y is None:
            raise SystemExit('Provide both x and y for a direct goal')
        goals.append(Point(x=args.x, y=args.y, z=max(0.05, min(1.0, args.throttle_scale))))

    if not goals:
        raise SystemExit('No goals to publish. Provide x y or --file.')

    override_mode = not args.queue

    if override_mode:
        first = goals[0]
        publish_point(
            node,
            args.goal_topic,
            first.x,
            first.y,
            first.z,
            args.count,
            args.delay,
        )
        node.get_logger().info(
            f'Override goal -> {args.goal_topic}: x={first.x:.3f} y={first.y:.3f} throttle={first.z:.2f}'
        )

        if len(goals) > 1:
            for g in goals[1:]:
                publish_point(node, args.queue_topic, g.x, g.y, g.z, args.count, args.delay)
            node.get_logger().info(f'Queued {len(goals)-1} additional goal(s) on {args.queue_topic}')
    else:
        for g in goals:
            publish_point(node, args.queue_topic, g.x, g.y, g.z, args.count, args.delay)
        node.get_logger().info(f'Queued {len(goals)} goal(s) on {args.queue_topic}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
