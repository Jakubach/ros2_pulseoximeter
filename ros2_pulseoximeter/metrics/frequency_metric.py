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
        self._gap_count = 0

    def update(self, msg: Any, timestamp: float) -> None:
        if len(self._timestamps) > 0:
            dt = timestamp - self._timestamps[-1]
            mean_dt = self._calculate_mean_dt()
            if mean_dt > 0 and dt > 2 * mean_dt:
                self._gap_count += 1

        self._timestamps.append(timestamp)

    def get_result(self) -> Dict[str, Any]:
        if len(self._timestamps) < 2:
            return {
                'metric': self.name,
                'mean_dt': 0.0,
                'std_dt': 0.0,
                'max_gap': 0.0,
                'frequency_hz': 0.0,
                'gap_count': 0,
                'samples': len(self._timestamps),
                'healthy': True
            }

        dts = self._calculate_dts()
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
            'gap_count': self._gap_count,
            'samples': len(self._timestamps),
            'healthy': bool(self.is_healthy)
        }

    def reset(self) -> None:
        self._timestamps.clear()
        self._gap_count = 0

    @property
    def is_healthy(self) -> bool:
        if len(self._timestamps) < 2:
            return True

        dts = self._calculate_dts()
        std_dt = np.std(dts)
        return bool(std_dt < self.std_threshold)

    def _calculate_dts(self) -> np.ndarray:
        timestamps = list(self._timestamps)
        return np.diff(timestamps)

    def _calculate_mean_dt(self) -> float:
        if len(self._timestamps) < 2:
            return 0.0
        dts = self._calculate_dts()
        return float(np.mean(dts))
