import json
from importlib import import_module
from typing import Any, Dict, List, Type

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from .config_loader import ConfigLoader, MonitorConfig, QoSConfig
from .topic_monitor import TopicMonitor
from .metrics.base_metric import BaseMetric
from .metrics.frequency_metric import FrequencyMetric
from .metrics.covariance_metric import CovarianceMetric


class MonitorNode(Node):

    METRIC_REGISTRY: Dict[str, Type[BaseMetric]] = {
        'frequency': FrequencyMetric,
        'covariance': CovarianceMetric,
    }

    def __init__(self):
        super().__init__('monitor_node')

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value

        if not config_path:
            self.get_logger().error('Missing config_path parameter')
            raise ValueError('config_path is required')

        self.get_logger().info(f'Loading config from: {config_path}')
        loader = ConfigLoader(config_path)
        self.config: MonitorConfig = loader.load()

        self._monitors: Dict[str, TopicMonitor] = {}
        self._subscriptions = []

        for topic_name, topic_config in self.config.topics.items():
            self._setup_topic_monitor(
                topic_name,
                topic_config.msg_type,
                topic_config.metrics,
                topic_config.qos
            )

        self._metrics_pub = self.create_publisher(String, '/monitor/metrics', 10)

        timer_period = 1.0 / self.config.publish_rate
        self._timer = self.create_timer(timer_period, self._publish_metrics)

        self.get_logger().info(
            f'Monitor started: {len(self._monitors)} topics, '
            f'publishing every {timer_period:.2f}s'
        )

    def _setup_topic_monitor(
        self,
        topic_name: str,
        msg_type_str: str,
        metric_names: List[str],
        qos_config: QoSConfig | None
    ) -> None:
        metrics = self._create_metrics(metric_names)
        monitor = TopicMonitor(topic_name, metrics)
        self._monitors[topic_name] = monitor

        msg_type = self._get_msg_type(msg_type_str)
        if msg_type is None:
            self.get_logger().warn(f'Cannot load type {msg_type_str} for {topic_name}')
            return

        qos_profile = self._build_qos_profile(qos_config)

        sub = self.create_subscription(
            msg_type,
            topic_name,
            monitor.callback,
            qos_profile
        )
        self._subscriptions.append(sub)

        qos_info = f', QoS: {qos_config.durability}' if qos_config else ''
        self.get_logger().info(
            f'Monitoring {topic_name} ({msg_type_str}) with metrics: {metric_names}{qos_info}'
        )

    def _create_metrics(self, metric_names: List[str]) -> List[BaseMetric]:
        metrics = []
        for name in metric_names:
            if name not in self.METRIC_REGISTRY:
                self.get_logger().warn(f'Unknown metric: {name}')
                continue

            metric_class = self.METRIC_REGISTRY[name]
            metric_config = self.config.metrics.get(name, {})
            metrics.append(metric_class(name, metric_config))

        return metrics

    def _build_qos_profile(self, qos_config: QoSConfig | None) -> QoSProfile:
        if qos_config is None:
            return QoSProfile(depth=10)

        reliability_map = {
            'reliable': ReliabilityPolicy.RELIABLE,
            'best_effort': ReliabilityPolicy.BEST_EFFORT,
        }
        durability_map = {
            'volatile': DurabilityPolicy.VOLATILE,
            'transient_local': DurabilityPolicy.TRANSIENT_LOCAL,
        }

        return QoSProfile(
            reliability=reliability_map.get(qos_config.reliability, ReliabilityPolicy.RELIABLE),
            durability=durability_map.get(qos_config.durability, DurabilityPolicy.VOLATILE),
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_config.depth
        )

    def _get_msg_type(self, type_string: str) -> type | None:
        try:
            parts = type_string.split('/')
            if len(parts) != 3:
                return None

            package, submodule, class_name = parts
            module = import_module(f'{package}.{submodule}')
            return getattr(module, class_name)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f'Import error for {type_string}: {e}')
            return None

    def _publish_metrics(self) -> None:
        results = {
            'topics': {},
            'all_healthy': True
        }

        for topic_name, monitor in self._monitors.items():
            topic_results = monitor.get_results()
            results['topics'][topic_name] = topic_results

            if not topic_results.get('healthy', True):
                results['all_healthy'] = False

        msg = String()
        msg.data = json.dumps(results, indent=2)
        self._metrics_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
