import pytest
import tempfile
import os
from ros2_pulseoximeter.config_loader import ConfigLoader, QoSConfig, TopicConfig


class TestConfigLoader:
    def test_load_valid_config(self):
        config_content = """
metrics:
  frequency:
    window_size: 50
  covariance:
    trace_position_threshold: 0.3

topics:
  /test_topic:
    type: std_msgs/msg/String
    metrics: [frequency]

publish_rate: 2.0
"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(config_content)
            f.flush()

            loader = ConfigLoader(f.name)
            config = loader.load()

            assert config.publish_rate == 2.0
            assert 'frequency' in config.metrics
            assert '/test_topic' in config.topics
            assert config.topics['/test_topic'].msg_type == 'std_msgs/msg/String'

        os.unlink(f.name)

    def test_load_config_with_qos(self):
        config_content = """
metrics:
  frequency:
    window_size: 100

topics:
  /test_topic:
    type: std_msgs/msg/String
    metrics: [frequency]
    qos:
      reliability: reliable
      durability: transient_local
      depth: 5

publish_rate: 1.0
"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(config_content)
            f.flush()

            loader = ConfigLoader(f.name)
            config = loader.load()

            topic_config = config.topics['/test_topic']
            assert topic_config.qos is not None
            assert topic_config.qos.reliability == 'reliable'
            assert topic_config.qos.durability == 'transient_local'
            assert topic_config.qos.depth == 5

        os.unlink(f.name)

    def test_missing_topics_raises_error(self):
        config_content = """
metrics:
  frequency:
    window_size: 100
"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(config_content)
            f.flush()

            loader = ConfigLoader(f.name)
            with pytest.raises(ValueError, match="topics"):
                loader.load()

        os.unlink(f.name)

    def test_undefined_metric_raises_error(self):
        config_content = """
metrics:
  frequency:
    window_size: 100

topics:
  /test_topic:
    type: std_msgs/msg/String
    metrics: [frequency, undefined_metric]

publish_rate: 1.0
"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(config_content)
            f.flush()

            loader = ConfigLoader(f.name)
            with pytest.raises(ValueError, match="undefined_metric"):
                loader.load()

        os.unlink(f.name)
