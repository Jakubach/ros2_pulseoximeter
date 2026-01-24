import time
from typing import Any, Dict, List

from .metrics.base_metric import BaseMetric


class TopicMonitor:

    def __init__(self, topic_name: str, metrics: List[BaseMetric]):
        self.topic_name = topic_name
        self.metrics = metrics

    def callback(self, msg: Any) -> None:
        timestamp = time.time()
        for metric in self.metrics:
            metric.update(msg, timestamp)

    def get_results(self) -> Dict[str, Any]:
        results = {
            'topic': self.topic_name,
            'metrics': {}
        }

        for metric in self.metrics:
            results['metrics'][metric.name] = metric.get_result()

        results['healthy'] = all(m.is_healthy for m in self.metrics)

        return results

    def reset_all(self) -> None:
        for metric in self.metrics:
            metric.reset()
