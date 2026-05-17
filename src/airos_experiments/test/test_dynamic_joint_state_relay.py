from __future__ import annotations

from pathlib import Path

from control_msgs.msg import DynamicJointState, InterfaceValue

from airos_experiments.dynamic_joint_state_relay import joint_state_from_dynamic


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _read_text(relative_path: str) -> str:
    return (_repo_root() / relative_path).read_text(encoding='utf-8')


def test_dynamic_joint_state_relay_converts_ros2_control_state() -> None:
    msg = DynamicJointState()
    msg.header.frame_id = 'base_link'
    msg.joint_names = ['lf_hip_joint', 'lf_upper_leg_joint']
    msg.interface_values = [
        InterfaceValue(
            interface_names=['position', 'velocity'],
            values=[0.12, 0.34],
        ),
        InterfaceValue(
            interface_names=['position', 'velocity', 'effort'],
            values=[-0.56, 0.78, 1.25],
        ),
    ]

    joint_state = joint_state_from_dynamic(msg)

    assert joint_state.header.frame_id == 'base_link'
    assert joint_state.name == ['lf_hip_joint', 'lf_upper_leg_joint']
    assert list(joint_state.position) == [0.12, -0.56]
    assert list(joint_state.velocity) == [0.34, 0.78]
    assert list(joint_state.effort) == [0.0, 1.25]


def test_dynamic_joint_state_relay_is_registered_for_legged_sim() -> None:
    setup_py = _read_text('src/airos_experiments/setup.py')
    package_xml = _read_text('src/airos_experiments/package.xml')
    sim_launch = _read_text('src/airos_sim/launch/sim.launch.py')

    assert (
        'dynamic_joint_state_relay = '
        'airos_experiments.dynamic_joint_state_relay:main'
    ) in setup_py
    assert '<exec_depend>control_msgs</exec_depend>' in package_xml
    assert "executable='dynamic_joint_state_relay'" in sim_launch
    assert "'input_topic': '/dynamic_joint_states'" in sim_launch
    assert "'output_topic': '/joint_states'" in sim_launch
