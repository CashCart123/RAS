import argparse
import time
from pathlib import Path
from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


def package_share_config_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory('point_nav')) / 'config'
    except Exception:
        return Path(__file__).resolve().parents[1] / 'config'


def default_params_path() -> Path:
    return package_share_config_dir() / 'point_nav.defaults.yaml'


def default_goal_file_path() -> Path:
    return package_share_config_dir() / 'goal_inputs.defaults.yaml'


def load_yaml(path: Path) -> Dict:
    if yaml is None:
        return {}
    if not path.exists():
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    return data if isinstance(data, dict) else {}


def extract_param_defaults(data: Dict) -> Dict:
    params = {}
    point_nav = data.get('point_nav')
    if isinstance(point_nav, dict):
        ros_params = point_nav.get('ros__parameters')
        if isinstance(ros_params, dict):
            params = ros_params
    return params


def extract_cli_defaults(data: Dict) -> Dict:
    cli_defaults = data.get('cli_defaults')
    return cli_defaults if isinstance(cli_defaults, dict) else {}


def extract_goal_inputs(data: Dict) -> List[Point]:
    goal_inputs = None
    if isinstance(data.get('goal_inputs'), list):
        goal_inputs = data.get('goal_inputs')
    elif isinstance(data.get('point_nav'), dict):
        point_nav = data.get('point_nav')
        if isinstance(point_nav.get('goal_inputs'), list):
            goal_inputs = point_nav.get('goal_inputs')
        elif isinstance(point_nav.get('ros__parameters'), dict):
            ros_params = point_nav['ros__parameters']
            if isinstance(ros_params.get('goal_inputs'), list):
                goal_inputs = ros_params.get('goal_inputs')

    goals: List[Point] = []
    if not isinstance(goal_inputs, list):
        return goals

    for item in goal_inputs:
        if not isinstance(item, dict):
            continue
        if 'x' not in item or 'y' not in item:
            continue
        throttle = max(0.05, min(1.0, float(item.get('throttle_scale', 1.0))))
        goals.append(Point(x=float(item['x']), y=float(item['y']), z=throttle))
    return goals


def build_parser(defaults: Dict) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Publish point_nav goals with override/queue modes and file-based goal loading.'
    )

    parser.add_argument('x', nargs='?', type=float, help='Goal X in meters (forward)')
    parser.add_argument('y', nargs='?', type=float, help='Goal Y in meters (left)')

    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument('--queue', action='store_true', help='Queue goal(s) onto goal_add_topic')
    mode_group.add_argument('--override', action='store_true', help='Override current goal and clear queued goals')

    parser.add_argument('--throttle-scale', type=float, default=1.0, help='Per-goal throttle scale in (0,1]')
    parser.add_argument('--count', type=int, default=int(defaults['count']), help='Times to publish each goal message')
    parser.add_argument('--delay', type=float, default=float(defaults['delay']), help='Delay between publishes in seconds')

    parser.add_argument('--goal-topic', type=str, default=str(defaults['goal_topic']), help='Override goal topic')
    parser.add_argument('--queue-topic', type=str, default=str(defaults['queue_topic']), help='Queue add topic')

    parser.add_argument('--params-file', type=str, default=str(defaults['params_file']), help='Config file for CLI/topic defaults')
    parser.add_argument('--file', type=str, default='', help='YAML file containing goal_inputs')
    parser.add_argument('--no-file', action='store_true', help='Do not auto-load goals from the default file')

    return parser


def initial_defaults() -> Dict:
    params_path = default_params_path()
    params_data = load_yaml(params_path)
    params = extract_param_defaults(params_data)
    cli_defaults = extract_cli_defaults(params_data)

    return {
        'params_file': params_path,
        'goal_file': Path(str(params.get('goal_file_path') or default_goal_file_path())),
        'goal_topic': params.get('goal_override_topic', '/goal_point'),
        'queue_topic': params.get('goal_add_topic', '/goal_point/add'),
        'count': cli_defaults.get('publish_count', 3),
        'delay': cli_defaults.get('publish_delay', 0.1),
        'mode': cli_defaults.get('mode', 'override'),
    }


def resolve_defaults() -> Tuple[argparse.Namespace, Dict]:
    boot_defaults = initial_defaults()
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument('--params-file', type=str, default=str(boot_defaults['params_file']))
    known_args, _ = pre_parser.parse_known_args()

    params_path = Path(known_args.params_file)
    params_data = load_yaml(params_path)
    params = extract_param_defaults(params_data)
    cli_defaults = extract_cli_defaults(params_data)

    resolved = {
        'params_file': params_path,
        'goal_file': Path(str(params.get('goal_file_path') or default_goal_file_path())),
        'goal_topic': params.get('goal_override_topic', '/goal_point'),
        'queue_topic': params.get('goal_add_topic', '/goal_point/add'),
        'count': cli_defaults.get('publish_count', 3),
        'delay': cli_defaults.get('publish_delay', 0.1),
        'mode': str(cli_defaults.get('mode', 'override')).strip().lower(),
    }

    parser = build_parser(resolved)
    return parser.parse_args(), resolved


def publish_point(node: Node, topic: str, goal: Point, count: int, delay: float) -> None:
    pub = node.create_publisher(Point, topic, 10)
    for _ in range(max(1, count)):
        pub.publish(goal)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(max(0.0, delay))


def main():
    args, defaults = resolve_defaults()
    rclpy.init()
    node = Node('set_goal_cli')

    goals: List[Point] = []

    file_path = '' if args.no_file else str(args.file).strip()
    if not file_path and args.x is None and args.y is None and not args.no_file:
        file_path = str(defaults['goal_file'])

    if file_path:
        path = Path(file_path)
        if not path.exists():
            raise SystemExit(f'Goal file not found: {path}')
        goals.extend(extract_goal_inputs(load_yaml(path)))
        node.get_logger().info(f'Loaded {len(goals)} goals from {path}')

    if args.x is not None or args.y is not None:
        if args.x is None or args.y is None:
            raise SystemExit('Provide both x and y for a direct goal')
        throttle = max(0.05, min(1.0, float(args.throttle_scale)))
        goals.append(Point(x=float(args.x), y=float(args.y), z=throttle))

    if not goals:
        raise SystemExit('No goals to publish. Provide x y, use --file, or rely on the default goal file.')

    override_mode = args.override or (not args.queue and defaults['mode'] != 'queue')

    if override_mode:
        first = goals[0]
        publish_point(node, args.goal_topic, first, args.count, args.delay)
        node.get_logger().info(
            f'Override goal -> {args.goal_topic}: x={first.x:.3f} y={first.y:.3f} throttle={first.z:.2f}'
        )
        for goal in goals[1:]:
            publish_point(node, args.queue_topic, goal, args.count, args.delay)
        if len(goals) > 1:
            node.get_logger().info(f'Queued {len(goals) - 1} additional goal(s) on {args.queue_topic}')
    else:
        for goal in goals:
            publish_point(node, args.queue_topic, goal, args.count, args.delay)
        node.get_logger().info(f'Queued {len(goals)} goal(s) on {args.queue_topic}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
