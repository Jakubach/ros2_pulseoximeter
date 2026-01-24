from collections import deque
from typing import Any, Dict
import numpy as np

from .base_metric import BaseMetric


class FrequencyMetric(BaseMetric):

    def __init__(self, name: str, config: Dict[str, Any]):
        super().__init__(name, config)
        self.window_size = config.get('window_size', 100)
        self.std_threshold = config.get('std_threshold', 0.1)
        self._timestamps: deque = deque(maxlen=self.window_size)

    def update(self, msg: Any, timestamp: float) -> None:
        self._timestamps.append(timestamp)

    def get_result(self) -> Dict[str, Any]:
        if len(self._timestamps) < 2:
            return {
                'metric': self.name,
                'mean_dt': 0.0,
                'std_dt': 0.0,
                'max_gap': 0.0,
                'frequency_hz': 0.0,
                'samples': len(self._timestamps),
                'healthy': True
            }

        dts = np.diff(list(self._timestamps))
        mean_dt = np.mean(dts)
        std_dt = np.std(dts)
        max_gap = np.max(dts)
        frequency_hz = 1.0 / mean_dt if mean_dt > 0 else 0.0

        return {
            'metric': self.name,
            'mean_dt': float(mean_dt),
            'std_dt': float(std_dt),
            'max_gap': float(max_gap),
            'frequency_hz': float(frequency_hz),
            'samples': len(self._timestamps),
            'healthy': bool(std_dt < self.std_threshold)
        }

    def reset(self) -> None:
        self._timestamps.clear()

    @property
    def is_healthy(self) -> bool:
        if len(self._timestamps) < 2:
            return True
        dts = np.diff(list(self._timestamps))
        return bool(np.std(dts) < self.std_threshold)
