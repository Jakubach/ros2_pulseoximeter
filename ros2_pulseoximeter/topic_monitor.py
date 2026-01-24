import time
from typing import Any, Dict, List

from .metrics.base_metric import BaseMetric


class TopicMonitor:

    def __init__(self, topic_name: str, metrics: List[BaseMetric]):
        self.topic_name = topic_name
        self.metrics = metrics
        self._message_count = 0

    def callback(self, msg: Any) -> None:
        timestamp = time.time()
        self._message_count += 1

        for metric in self.metrics:
            metric.update(msg, timestamp)

    def get_results(self) -> Dict[str, Any]:
        results = {
            'topic': self.topic_name,
            'message_count': self._message_count,
            'metrics': {}
        }

        for metric in self.metrics:
            results['metrics'][metric.name] = metric.get_result()

        results['healthy'] = all(m.is_healthy for m in self.metrics)

        return results

    def reset_all(self) -> None:
        self._message_count = 0
        for metric in self.metrics:
            metric.reset()
