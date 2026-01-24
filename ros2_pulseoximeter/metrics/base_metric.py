from abc import ABC, abstractmethod
from typing import Any, Dict


class BaseMetric(ABC):

    def __init__(self, name: str, config: Dict[str, Any]):
        self.name = name
        self.config = config

    @abstractmethod
    def update(self, msg: Any, timestamp: float) -> None:
        pass

    @abstractmethod
    def get_result(self) -> Dict[str, Any]:
        pass

    @abstractmethod
    def reset(self) -> None:
        pass

    @property
    @abstractmethod
    def is_healthy(self) -> bool:
        pass
