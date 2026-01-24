import pytest
import time
from ros2_pulseoximeter.metrics.frequency_metric import FrequencyMetric


class TestFrequencyMetric:
    def test_init(self):
        metric = FrequencyMetric('frequency', {'window_size': 50, 'std_threshold': 0.05})
        assert metric.name == 'frequency'
        assert metric.window_size == 50
        assert metric.std_threshold == 0.05

    def test_empty_metric_is_healthy(self):
        metric = FrequencyMetric('frequency', {})
        assert metric.is_healthy is True
        result = metric.get_result()
        assert result['healthy'] is True
        assert result['samples'] == 0

    def test_single_sample_is_healthy(self):
        metric = FrequencyMetric('frequency', {})
        metric.update(None, 1.0)
        assert metric.is_healthy is True
        assert metric.get_result()['samples'] == 1

    def test_stable_frequency_is_healthy(self):
        metric = FrequencyMetric('frequency', {'std_threshold': 0.1})
        for i in range(20):
            metric.update(None, i * 0.1)

        result = metric.get_result()
        assert result['healthy'] is True
        assert abs(result['frequency_hz'] - 10.0) < 0.5
        assert result['std_dt'] < 0.01

    def test_unstable_frequency_is_unhealthy(self):
        metric = FrequencyMetric('frequency', {'std_threshold': 0.01})
        timestamps = [0, 0.1, 0.3, 0.35, 0.6, 0.65, 0.9, 1.2]
        for t in timestamps:
            metric.update(None, t)

        result = metric.get_result()
        assert result['std_dt'] > 0.01
        assert result['healthy'] is False

    def test_gap_detection(self):
        metric = FrequencyMetric('frequency', {})
        for i in range(10):
            metric.update(None, i * 0.1)
        metric.update(None, 1.5)

        result = metric.get_result()
        assert result['gap_count'] >= 1
        assert result['max_gap'] > 0.4

    def test_reset(self):
        metric = FrequencyMetric('frequency', {})
        for i in range(10):
            metric.update(None, i * 0.1)

        metric.reset()
        result = metric.get_result()
        assert result['samples'] == 0
        assert result['gap_count'] == 0
