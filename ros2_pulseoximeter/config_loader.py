from dataclasses import dataclass
from typing import Any, Dict, List, Optional
import yaml


@dataclass
class QoSConfig:
    reliability: str = 'reliable'
    durability: str = 'volatile'
    depth: int = 10


@dataclass
class TopicConfig:
    msg_type: str
    metrics: List[str]
    qos: Optional[QoSConfig] = None


@dataclass
class MonitorConfig:
    metrics: Dict[str, Dict[str, Any]]
    topics: Dict[str, TopicConfig]
    publish_rate: float = 1.0


class ConfigLoader:

    def __init__(self, config_path: str):
        self.config_path = config_path

    def load(self) -> MonitorConfig:
        with open(self.config_path, 'r') as f:
            raw_config = yaml.safe_load(f)

        self._validate(raw_config)

        metrics = raw_config.get('metrics', {})
        publish_rate = raw_config.get('publish_rate', 1.0)

        topics = {}
        for topic_name, topic_data in raw_config.get('topics', {}).items():
            qos_data = topic_data.get('qos')
            qos_config = None
            if qos_data:
                qos_config = QoSConfig(
                    reliability=qos_data.get('reliability', 'reliable'),
                    durability=qos_data.get('durability', 'volatile'),
                    depth=qos_data.get('depth', 10)
                )
            topics[topic_name] = TopicConfig(
                msg_type=topic_data.get('type', ''),
                metrics=topic_data.get('metrics', []),
                qos=qos_config
            )

        return MonitorConfig(
            metrics=metrics,
            topics=topics,
            publish_rate=publish_rate
        )

    def _validate(self, config: Dict[str, Any]) -> None:
        if 'topics' not in config:
            raise ValueError("Missing 'topics' section in config")

        if 'metrics' not in config:
            raise ValueError("Missing 'metrics' section in config")

        defined_metrics = set(config.get('metrics', {}).keys())
        for topic_name, topic_data in config.get('topics', {}).items():
            topic_metrics = set(topic_data.get('metrics', []))
            undefined = topic_metrics - defined_metrics
            if undefined:
                raise ValueError(
                    f"Topic '{topic_name}' uses undefined metrics: {undefined}"
                )
