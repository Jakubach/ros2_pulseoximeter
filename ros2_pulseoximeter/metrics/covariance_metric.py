from typing import Any, Dict
import numpy as np

from .base_metric import BaseMetric


class CovarianceMetric(BaseMetric):

    def __init__(self, name: str, config: Dict[str, Any]):
        super().__init__(name, config)
        self.trace_position_threshold = config.get('trace_position_threshold', 0.5)

        self._last_covariance: np.ndarray | None = None
        self._last_trace_position = 0.0
        self._last_trace_orientation = 0.0
        self._last_max_eigenvalue = 0.0
        self._last_determinant = 0.0
        self._sample_count = 0

    def update(self, msg: Any, timestamp: float) -> None:
        covariance = self._extract_covariance(msg)
        if covariance is None:
            return

        self._last_covariance = covariance
        self._sample_count += 1

        cov_matrix = covariance.reshape(6, 6)

        self._last_trace_position = cov_matrix[0, 0] + cov_matrix[1, 1] + cov_matrix[2, 2]
        self._last_trace_orientation = cov_matrix[3, 3] + cov_matrix[4, 4] + cov_matrix[5, 5]

        try:
            eigenvalues = np.linalg.eigvalsh(cov_matrix)
            self._last_max_eigenvalue = float(np.max(eigenvalues))
            self._last_determinant = float(np.linalg.det(cov_matrix))
        except np.linalg.LinAlgError:
            self._last_max_eigenvalue = 0.0
            self._last_determinant = 0.0

    def get_result(self) -> Dict[str, Any]:
        return {
            'metric': self.name,
            'trace_position': float(self._last_trace_position),
            'trace_orientation': float(self._last_trace_orientation),
            'trace_total': float(self._last_trace_position + self._last_trace_orientation),
            'max_eigenvalue': float(self._last_max_eigenvalue),
            'determinant': float(self._last_determinant),
            'samples': self._sample_count,
            'healthy': bool(self.is_healthy)
        }

    def reset(self) -> None:
        self._last_covariance = None
        self._last_trace_position = 0.0
        self._last_trace_orientation = 0.0
        self._last_max_eigenvalue = 0.0
        self._last_determinant = 0.0
        self._sample_count = 0

    @property
    def is_healthy(self) -> bool:
        if self._last_covariance is None:
            return True
        return bool(self._last_trace_position < self.trace_position_threshold)

    def _extract_covariance(self, msg: Any) -> np.ndarray | None:
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'covariance'):
            return np.array(msg.pose.covariance)

        if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose') and hasattr(msg.pose, 'covariance'):
            return np.array(msg.pose.covariance)

        if hasattr(msg, 'covariance'):
            return np.array(msg.covariance)

        return None
