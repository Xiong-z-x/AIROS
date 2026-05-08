# AIROS Environment Baseline

Date: 2026-05-07  
Scope: Task 1 of `docs/AIROS_phased_execution_plan.md`

## Final Environment Decision

The project baseline is fixed as:

```text
OS: Ubuntu 22.04.5 LTS on WSL2
ROS: ROS 2 Humble
Gazebo: Ignition Gazebo Fortress 6.16.0
Gazebo command: ign gazebo
Gazebo GUI render backend: ogre
Gazebo GUI render args: --render-engine ogre --render-engine-gui ogre
ROS/Gazebo bridge: ros-humble-ros-gz 0.244.23
Nav2: 1.1.20
SLAM: slam_toolbox 2.6.10
```

Do not use `gz sim` for Fortress on this machine. The `gz` command is not installed; the valid command family is `ign gazebo`.

## GPU / OpenGL Evidence

`scripts/check_gpu_gazebo_stack.sh` confirmed:

```text
GPU: NVIDIA GeForce RTX 3050 Laptop GPU
NVIDIA driver: 595.97
GPU memory: 4096 MiB
OpenGL direct rendering: Yes
OpenGL accelerated: yes
OpenGL renderer: D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)
OpenGL core profile version: 4.2
OpenGL vendor: Microsoft Corporation
Mesa version: 23.2.1
```

Interpretation:

- Gazebo GUI can use the WSLg D3D12/NVIDIA OpenGL acceleration path.
- Gazebo GUI rendering is not judged by CUDA availability.
- Software renderers such as `llvmpipe`, `softpipe`, or `Software Rasterizer` are rejected by the check script.

## Gazebo GUI Render Probe

The hard gate is a real Gazebo GUI probe, not only `glxinfo`.

Passing command:

```bash
env \
  LIBGL_ALWAYS_SOFTWARE=0 \
  MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
  ign gazebo -v 4 \
    --render-engine ogre \
    --render-engine-gui ogre \
    /usr/share/ignition/ignition-gazebo6/worlds/empty.sdf
```

Observed passing evidence:

```text
Ignition Gazebo Server v6.16.0
Ignition Gazebo GUI    v6.16.0
Loaded plugin [MinimalScene]
Loaded plugin [GzSceneManager]
Loaded plugin [InteractiveViewControl]
Loaded plugin [WorldStats]
[PASS] Gazebo GUI render probe stayed alive for 8s using ogre with accelerated NVIDIA/D3D12 OpenGL.
```

The probe log is written to:

```text
/tmp/airos_gazebo_gpu_probe.log
```

## Rejected Render Backend

The default `ogre2` backend is not accepted on this WSLg/D3D12 machine.

Observed failure:

```text
terminate called after throwing an instance of 'Ogre::UnimplementedException'
what(): OGRE EXCEPTION(9:UnimplementedException): in GL3PlusTextureGpu::copyTo
Aborted
```

Decision:

```text
Use --render-engine ogre --render-engine-gui ogre for all Gazebo GUI launches in this project.
```

## ROS / Nav2 Package Evidence

`scripts/check_ros_nav_stack.sh` confirmed:

```text
[OK] slam_toolbox -> /opt/ros/humble
[OK] nav2_route -> /opt/ros/humble
[OK] nav2_mppi_controller -> /opt/ros/humble
[OK] nav2_smac_planner -> /opt/ros/humble
[OK] nav2_collision_monitor -> /opt/ros/humble
[OK] nav2_velocity_smoother -> /opt/ros/humble
[OK] ros_gz_bridge -> /opt/ros/humble
[OK] ros_gz_sim -> /opt/ros/humble
[PASS] Required ROS 2 Humble navigation and Gazebo bridge packages are available.
```

## Commands

Run the baseline gate with:

```bash
scripts/check_gpu_gazebo_stack.sh
scripts/check_ros_nav_stack.sh
```

Both scripts must pass before starting Task 2.

