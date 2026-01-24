# ros2_pulseoximeter

ROS2 package for monitoring topic health with pluggable metrics.

## Features

- **Frequency monitoring** - detects lag, gaps, unstable publishing rates
- **Covariance monitoring** - tracks localization uncertainty (for AMCL, EKF, etc.)
- **Pluggable architecture** - easy to add custom metrics
- **Configurable QoS** - supports transient_local, reliable, best_effort
- **JSON output** - easy integration with other systems

## Installation

```bash
cd ~/ros2_ws/src
git clone <repo_url> ros2_pulseoximeter
cd ~/ros2_ws
colcon build --packages-select ros2_pulseoximeter
```

## Usage

```bash
ros2 launch ros2_pulseoximeter monitor.launch.py
ros2 topic echo /monitor/metrics --full-length
```

## Configuration

Edit `config/monitor_config.yaml`:

```yaml
metrics:
  frequency:
    window_size: 100
    std_threshold: 0.1
  covariance:
    trace_position_threshold: 0.5

topics:
  /amcl_pose:
    type: geometry_msgs/msg/PoseWithCovarianceStamped
    metrics: [frequency, covariance]
    qos:
      reliability: reliable
      durability: transient_local
      depth: 1
  /odom:
    type: nav_msgs/msg/Odometry
    metrics: [frequency]

publish_rate: 1.0
```

## Adding Custom Metrics

1. Create class inheriting from `BaseMetric` in `metrics/`
2. Implement: `update()`, `get_result()`, `reset()`, `is_healthy`
3. Register in `METRIC_REGISTRY` in `monitor_node.py`

## Output Format

```json
{
  "topics": {
    "/amcl_pose": {
      "topic": "/amcl_pose",
      "healthy": true,
      "message_count": 150,
      "metrics": {
        "frequency": {
          "metric": "frequency",
          "frequency_hz": 10.0,
          "std_dt": 0.002,
          "mean_dt": 0.1,
          "max_gap": 0.12,
          "gap_count": 0,
          "samples": 100,
          "healthy": true
        },
        "covariance": {
          "metric": "covariance",
          "trace_position": 0.003,
          "trace_orientation": 0.0001,
          "trace_total": 0.0031,
          "max_eigenvalue": 0.002,
          "determinant": 0.0,
          "samples": 150,
          "healthy": true
        }
      }
    },
    "/odom": {
      "topic": "/odom",
      "healthy": true,
      "message_count": 1000,
      "metrics": {
        "frequency": {
          "metric": "frequency",
          "frequency_hz": 15.2,
          "std_dt": 0.001,
          "mean_dt": 0.066,
          "max_gap": 0.08,
          "gap_count": 0,
          "samples": 100,
          "healthy": true
        }
      }
    }
  },
  "all_healthy": true
}
```

## License

MIT
