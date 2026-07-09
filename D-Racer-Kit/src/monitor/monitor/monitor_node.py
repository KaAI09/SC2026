import os
from pathlib import Path
import shutil

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from dracer_msgs.msg import Battery
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import yaml

from .flask_app_factory import FLASK_IMPORT_ERROR, FlaskServerThread, create_app
from .image_utils import extract_jpeg_dimensions
from .monitor_state import MonitorState

PACKAGE_ROOT = Path(__file__).resolve().parent


def get_default_vehicle_config_path():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'config' / 'vehicle_config.yaml'
        if candidate.exists():
            return str(candidate)
    return '/home/topst/SC2026/D-Racer-Kit/src/config/vehicle_config.yaml'


def resolve_resource_path(filename):
    candidates = []

    try:
        share_dir = Path(get_package_share_directory('monitor'))
        candidates.append(share_dir / 'resource' / filename)
    except PackageNotFoundError:
        pass

    candidates.append(PACKAGE_ROOT.parent / 'resource' / filename)

    for candidate in candidates:
        if candidate.exists():
            return candidate

    raise FileNotFoundError(f'Unable to find resource file: {filename}')


class MonitorNode(Node):
    """Lightweight web dashboard: live camera + storage + battery only.

    Deliberately minimal — it subscribes to just the camera image and battery
    topics (plus a storage poll timer) so the web view stays real-time and does
    not add ROS-graph load. Control/joystick/debug-image/ROS-graph panels were
    removed to keep latency low during track tests.
    """

    def __init__(self):
        super().__init__('monitor_node')

        if FLASK_IMPORT_ERROR is not None:
            raise RuntimeError(
                'Flask is not installed. Install "python3-flask" and try again.'
            ) from FLASK_IMPORT_ERROR

        self.declare_parameter('vehicle_config_file', get_default_vehicle_config_path())
        self.declare_parameter('battery_topic', 'battery_status')
        self.declare_parameter('image_topic', '')   # '' -> vehicle_config IMAGE_TOPIC; set to override
        self.declare_parameter('storage_path', '/')
        self.declare_parameter('storage_poll_interval_sec', 1.0)
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 5000)
        self.declare_parameter('page_title', 'D-Racer Monitor')
        self.declare_parameter('refresh_interval_ms', 1000)
        self.declare_parameter('image_refresh_interval_ms', 100)
        self.declare_parameter('stale_timeout_sec', 3.0)
        self.declare_parameter('image_source_width', 160)
        self.declare_parameter('image_source_height', 120)
        self.declare_parameter('image_display_width', 160)
        self.declare_parameter('image_display_height', 120)
        self.declare_parameter('debug_log', False)

        self.vehicle_config_file = os.path.expanduser(
            str(self.get_parameter('vehicle_config_file').value)
        )
        yaml_config = self.load_vehicle_config()

        self.battery_topic = self.get_yaml_or_param_str(yaml_config, 'BATTERY_TOPIC', 'battery_topic')
        _img_param = str(self.get_parameter('image_topic').value).strip()
        self.image_topic = (_img_param                                  # launch override wins
                            or self.get_yaml_or_param_str(yaml_config, 'IMAGE_TOPIC', 'image_topic')
                            or '/camera/image/compressed')

        requested_storage_path = Path(
            self.get_yaml_or_param_str(yaml_config, 'STORAGE_PATH', 'storage_path')
        ).expanduser()
        self.storage_poll_interval_sec = float(
            self.get_parameter('storage_poll_interval_sec').value
        )
        self.web_host = self.get_yaml_or_param_str(yaml_config, 'WEB_HOST', 'web_host')
        self.web_port = self.get_yaml_or_param_int(yaml_config, 'WEB_PORT', 'web_port')
        self.page_title = str(self.get_parameter('page_title').value)
        self.refresh_interval_ms = int(self.get_parameter('refresh_interval_ms').value)
        self.image_refresh_interval_ms = int(
            self.get_parameter('image_refresh_interval_ms').value
        )
        self.debug_log = bool(self.get_parameter('debug_log').value)
        stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)
        image_source_width = int(self.get_parameter('image_source_width').value)
        image_source_height = int(self.get_parameter('image_source_height').value)
        self.image_source_width = image_source_width
        self.image_source_height = image_source_height
        self.image_display_width = self.get_yaml_or_param_int(
            yaml_config, 'IMAGE_DISPLAY_WIDTH', 'image_display_width'
        )
        self.image_display_height = self.get_yaml_or_param_int(
            yaml_config, 'IMAGE_DISPLAY_HEIGHT', 'image_display_height'
        )

        if requested_storage_path.exists():
            self.storage_path = str(requested_storage_path)
        else:
            self.storage_path = '/'
            self.get_logger().warning(
                f'Storage path {requested_storage_path} does not exist. Falling back to /.'
            )

        self.telechips_logo_path = resolve_resource_path('Telechips-CI-White.png')
        self.topst_logo_path = resolve_resource_path('TOPST-Logo(White).png')
        self.state = MonitorState(
            stale_timeout_sec,
            image_source_width,
            image_source_height,
        )
        self.app = create_app(
            self.state,
            self.page_title,
            self.battery_topic,
            self.image_topic,
            self.storage_path,
            self.refresh_interval_ms,
            self.image_refresh_interval_ms,
            self.telechips_logo_path,
            self.topst_logo_path,
            self.image_display_width,
            self.image_display_height,
        )
        self.server_thread = FlaskServerThread(self.app, self.web_host, self.web_port)

        self.create_subscription(
            Battery,
            self.battery_topic,
            self.battery_callback,
            10,
        )
        self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            10,
        )
        self.storage_timer = self.create_timer(
            self.storage_poll_interval_sec,
            self.storage_timer_callback,
        )
        self.storage_timer_callback()

        self.server_thread.start()

        display_host = '127.0.0.1' if self.web_host == '0.0.0.0' else self.web_host
        self.get_logger().info(
            f'[Monitor node started] \n'
            f'battery_topic={self.battery_topic} \n'
            f'image_topic={self.image_topic} \n'
            f'storage_path={self.storage_path}, \n'
            f'web=http://{display_host}:{self.web_port} \n'
            f'vehicle_config_file={self.vehicle_config_file} \n'
        )

    def load_vehicle_config(self):
        if not os.path.exists(self.vehicle_config_file):
            return {}

        try:
            with open(self.vehicle_config_file, 'r', encoding='utf-8') as config_stream:
                return yaml.safe_load(config_stream) or {}
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to read vehicle config file {self.vehicle_config_file}: {exc}'
            )
            return {}

    def get_yaml_or_param_str(self, yaml_config, yaml_key, param_key):
        raw_value = yaml_config.get(yaml_key)
        if raw_value is not None:
            text_value = str(raw_value).strip()
            if text_value:
                return text_value
        return str(self.get_parameter(param_key).value)

    def get_yaml_or_param_int(self, yaml_config, yaml_key, param_key):
        raw_value = yaml_config.get(yaml_key)
        if raw_value is not None:
            text_value = str(raw_value).strip()
            if text_value:
                return int(raw_value)
        return int(self.get_parameter(param_key).value)

    def battery_callback(self, msg):
        self.state.update_battery(msg.battery_status)

        if self.debug_log:
            self.get_logger().info(f'Battery status updated: {msg.battery_status:.1f}%')

    def image_callback(self, msg):
        try:
            frame_bytes = bytes(msg.data)
            width, height = extract_jpeg_dimensions(frame_bytes)
            if width is None or height is None:
                width = self.image_source_width
                height = self.image_source_height

            self.state.update_image(frame_bytes, width, height)
        except Exception as exc:
            self.get_logger().error(f'Failed to process {self.image_topic} frame: {exc}')

    def storage_timer_callback(self):
        try:
            usage = shutil.disk_usage(self.storage_path)
            self.state.update_storage(usage.used, usage.total)
        except Exception as exc:
            self.get_logger().error(f'Failed to read storage usage from {self.storage_path}: {exc}')

    def destroy_node(self):
        if hasattr(self, 'server_thread') and self.server_thread is not None:
            self.server_thread.shutdown()
            self.server_thread.join(timeout=2.0)

        super().destroy_node()


def main(args=None):
    node = None
    rclpy.init(args=args)

    try:
        node = MonitorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None:
            node.get_logger().info('Shutting down monitor node.')
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
