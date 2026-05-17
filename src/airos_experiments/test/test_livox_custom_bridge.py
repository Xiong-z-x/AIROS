from __future__ import annotations

from pathlib import Path


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _read_text(relative_path: str) -> str:
    return (_repo_root() / relative_path).read_text(encoding='utf-8')


def test_fast_lio_uses_livox_custom_msg_for_airos_sim() -> None:
    fast_lio_config = _read_text('src/fast_lio/config/airos_sim.yaml')

    assert 'lid_topic: "/livox/lidar"' in fast_lio_config
    assert 'lidar_type: 1' in fast_lio_config


def test_native_gazebo_pointcloud_is_converted_to_livox_custom_msg() -> None:
    sim_launch = _read_text('src/airos_sim/launch/sim.launch.py')
    setup_py = _read_text('src/airos_experiments/setup.py')
    bridge_node = _read_text(
        'src/airos_experiments/airos_experiments/livox_custom_bridge.py'
    )

    assert "executable='livox_custom_bridge'" in sim_launch
    assert "'input_topic': '/livox/lidar_points'" in sim_launch
    assert "'output_topic': '/livox/lidar'" in sim_launch
    assert (
        'livox_custom_bridge = '
        'airos_experiments.livox_custom_bridge:main'
    ) in setup_py
    assert 'livox_ros_driver2.msg' in bridge_node
    assert 'CustomMsg' in bridge_node
    assert 'CustomPoint' in bridge_node


def test_gazebo_pointcloud_bridge_keeps_raw_topic_separate_from_custom_msg() -> None:
    bridge_yaml = _read_text('src/airos_sim/config/ros_gz_bridge.yaml')

    assert 'ros_topic_name: /livox/lidar_points' in bridge_yaml
    assert 'topic_name: /unitree_lidar/points' in bridge_yaml
    assert 'ros_topic_name: /imu/data' in bridge_yaml
