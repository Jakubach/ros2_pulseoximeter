import pytest
import numpy as np
from unittest.mock import MagicMock
from ros2_pulseoximeter.metrics.covariance_metric import CovarianceMetric


class TestCovarianceMetric:
    def test_init(self):
        metric = CovarianceMetric('covariance', {'trace_position_threshold': 0.3})
        assert metric.name == 'covariance'
        assert metric.trace_position_threshold == 0.3

    def test_empty_metric_is_healthy(self):
        metric = CovarianceMetric('covariance', {})
        assert metric.is_healthy is True
        result = metric.get_result()
        assert result['healthy'] is True
        assert result['samples'] == 0

    def test_low_covariance_is_healthy(self):
        metric = CovarianceMetric('covariance', {'trace_position_threshold': 0.5})

        msg = MagicMock()
        cov = np.zeros(36)
        cov[0] = 0.01
        cov[7] = 0.01
        cov[14] = 0.01
        cov[21] = 0.001
        cov[28] = 0.001
        cov[35] = 0.001
        msg.pose.covariance = cov.tolist()

        metric.update(msg, 1.0)

        result = metric.get_result()
        assert result['healthy'] is True
        assert result['trace_position'] == pytest.approx(0.03, rel=0.01)
        assert result['samples'] == 1

    def test_high_covariance_is_unhealthy(self):
        metric = CovarianceMetric('covariance', {'trace_position_threshold': 0.1})

        msg = MagicMock()
        cov = np.zeros(36)
        cov[0] = 0.5
        cov[7] = 0.5
        cov[14] = 0.1
        msg.pose.covariance = cov.tolist()

        metric.update(msg, 1.0)

        result = metric.get_result()
        assert result['healthy'] is False
        assert result['trace_position'] > 0.1

    def test_extract_from_pose_with_covariance(self):
        metric = CovarianceMetric('covariance', {})

        msg = MagicMock()
        msg.pose.covariance = [0.01] * 36

        metric.update(msg, 1.0)
        assert metric.get_result()['samples'] == 1

    def test_no_covariance_field(self):
        metric = CovarianceMetric('covariance', {})

        msg = MagicMock(spec=[])

        metric.update(msg, 1.0)
        assert metric.get_result()['samples'] == 0

    def test_reset(self):
        metric = CovarianceMetric('covariance', {})

        msg = MagicMock()
        msg.pose.covariance = [0.01] * 36
        metric.update(msg, 1.0)

        metric.reset()
        result = metric.get_result()
        assert result['samples'] == 0
        assert result['trace_position'] == 0.0
