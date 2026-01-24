import math
from typing import Any, Dict, Optional

from .base_metric import BaseMetric


class PoseJumpMetric(BaseMetric):

    def __init__(self, name: str, config: Dict[str, Any]):
        super().__init__(name, config)
        self.jump_threshold = config.get('jump_threshold', 0.5)
        self._last_position: Optional[tuple] = None
        self._last_jump = 0.0
        self._jump_detected = False

    def update(self, msg: Any, timestamp: float) -> None:
        position = self._extract_position(msg)
        if position is None:
            return

        if self._last_position is not None:
            dx = position[0] - self._last_position[0]
            dy = position[1] - self._last_position[1]
            distance = math.sqrt(dx*dx + dy*dy)

            if distance > self.jump_threshold:
                self._jump_detected = True
                self._last_jump = distance

        self._last_position = position

    def get_result(self) -> Dict[str, Any]:
        return {
            'metric': self.name,
            'last_jump': float(self._last_jump),
            'jump_detected': self._jump_detected,
            'healthy': bool(not self._jump_detected)
        }

    def reset(self) -> None:
        self._last_position = None
        self._last_jump = 0.0
        self._jump_detected = False

    @property
    def is_healthy(self) -> bool:
        return not self._jump_detected

    def _extract_position(self, msg: Any) -> Optional[tuple]:
        # PoseWithCovarianceStamped
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
            p = msg.pose.pose.position
            return (p.x, p.y)
        # PoseStamped or Odometry
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'position'):
            p = msg.pose.position
            return (p.x, p.y)
        return None
