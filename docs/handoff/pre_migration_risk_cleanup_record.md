# Pre-Migration Risk Cleanup Record

Status: current risk list and cleanup log.
Last updated: 2026-05-15.

## Risk Summary

| ID | Risk | Status | Resolution |
|---|---|---|---|
| R1 | Historical docs imply the project is still only a flat Nav2 demo. | Fixed in handoff docs; existing docs marked as historical/background where needed. | New handoff package states current FAST-LIO2/PCT route and source priority. |
| R2 | Older FAST-LIO notes say cross-level path generation remains unaccepted. | Fixed by new status snapshot. | Current evidence separates high `/pct_path` acceptance from physical high-deck arrival, avoiding both underclaiming and overclaiming. |
| R3 | Speed-chain changes could be inconsistent across planner, Nav2 smoother, and base controller. | Fixed and tested. | Base, smoother, RPP, and direct planner limits are aligned and covered by tests. |
| R4 | SDF and SLAM-cloud planner responsibilities can be confused. | Fixed in handoff docs. | Active demo uses `/Laser_map_world`; SDF remains environment/test/legacy mode. |
| R5 | Root planning files `task_plan.md`, `findings.md`, and `progress.md` were stale. | Fixed by adding migration sections. | These files now point readers to `docs/handoff/`. |
| R6 | Generated directories and user documents can be mistaken for source. | Fixed by documentation and `.gitignore` check. | Handoff explicitly lists ignored/generated and user-material paths. |
| R7 | Live Gazebo/RViz sessions can interfere with verification. | Fixed for this run. | `scripts/cleanup_airos_runtime.sh` returned `[PASS] AIROS runtime processes cleaned.` before handoff verification. |
| R8 | Sparse FAST-LIO ramp/step samples can split high-floor graph components. | Fixed in code and tests. | Component sparse step bridges plus regression tests cover sparse non-floor gaps and wall-base rejection. |
| R9 | Frontier scoring can choose misleading ramp/step entries or too-local low ends. | Fixed in code and tests. | Entry attractor logic distinguishes high attractors from entry attractors and rejects backward/local low-end entries. |
| R10 | High `/pct_path` might be treated as full physical navigation success. | Not fully fixable in documentation only; explicitly constrained. | Handoff says high path generation and active commands are accepted, physical high-deck arrival is not. |

## Executed Cleanup Actions

- Cleared stale runtime processes with `scripts/cleanup_airos_runtime.sh`.
- Audited tracked and ignored directories with `git status --short`, `git ls-files`, `find`, and `du`.
- Created a dedicated `docs/handoff/` package to avoid spreading migration facts across historical notes.
- Updated root planning files to mark prior deep-upgrade plan as historical and route new work through this handoff package.
- Documented root-level PDF/DOCX files as user materials that should not be committed by default.
- Kept `build/`, `install/`, `log/`, and `results/` out of source status; they remain ignored generated artifacts.

## Verification Commands Used Or Required

Final pre-handoff gate executed on 2026-05-15:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest \
  src/airos_experiments/test/test_visual_pointcloud_config.py \
  src/airos_experiments/test/test_slam_traversability_graph.py \
  src/airos_experiments/test/test_terrain_pointcloud_planner.py \
  src/airos_experiments/test/test_control_command_chain.py -q
git diff --check
colcon build --symlink-install
bash scripts/cleanup_airos_runtime.sh
```

Observed results:

```text
pytest: 117 passed in 29.96s
git diff --check: no output
colcon build: 8 packages finished [1.28s]
cleanup: [PASS] AIROS runtime processes cleaned.
```

Runtime acceptance gate for the next conversation:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch airos_experiments visual_fast_lio_navigation.launch.py \
  gui:=false \
  rviz:=false \
  terrain_send_nav2_goals:=true \
  terrain_execution_mode:=direct
```

Then publish a terrain goal near `(6.0, 13.0, 2.2)` on `/terrain_goal_pose` and
monitor `/Laser_map_world`, `/pct_path`, `/cmd_vel_nav`, and
`/fast_lio_odom_world`. Accept path generation only if `max(path.z) > 2.0`.
Accept physical cross-level navigation only if odometry and Gazebo pose show
the robot actually climbs to the high deck.

## Risks Left Open

- Physical high-deck arrival remains unaccepted.
- Full route-constrained batch on advanced multi-level worlds remains unaccepted.
- Native Gazebo LiDAR under WSL stable mode is usable for raw-sensor tests but hardware `ogre2` remains unstable on this machine.
- Upstream CUDA PCT-planner and RL runtime remain research targets, not integrated production modules.
- Root-level course PDF/DOCX files remain in the workspace as user materials and are intentionally not tracked.
