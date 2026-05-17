# AIROS Handoff Index

Status: current entry point for migration to a new conversation.
Last updated: 2026-05-17.

## Purpose

This directory is the authoritative handoff package for the current AIROS
workspace. It exists because older project documents still contain useful
history, but some of them describe earlier phases or older acceptance evidence.
New work should start here, then follow the reading order below.

## Required Reading Order

1. `docs/handoff/pre_migration_handoff_report.md`
2. `docs/handoff/current_status_overview.md`
3. `docs/handoff/pre_migration_risk_cleanup_record.md`
4. `docs/handoff/key_file_reading_order.md`
5. `docs/handoff/next_model_cautions.md`
6. `docs/handoff/single_floor_final_report_2026-05-16.md`
7. `docs/handoff/phase1_perfected_baseline_2026-05-17.md`
8. `docs/handoff/fast_display_and_next_actions_2026-05-16.md`
9. `docs/handoff/objective_completion_audit_2026-05-16.md`
10. `docs/handoff/new_model_initialization_prompt.md`
11. `README.md`
12. `docs/go2w_fast_lio_upgrade_notes.md`
13. `docs/advanced_planning_research_profile.md`
14. `docs/planner_comparison_showcase_2026-05-17.md`
15. `docs/environment_baseline.md`
16. `task_plan.md`, `findings.md`, `progress.md`

## Source Priority

Use sources in this order when documents disagree:

1. Current code, launch files, tests, and command output.
2. This `docs/handoff/` package.
3. `README.md`.
4. Focused technical notes under `docs/`.
5. Historical `task_plan.md`, `findings.md`, and `progress.md`.

## Current Effective Baseline

- OS/runtime: Ubuntu 22.04 WSL2, ROS 2 Humble, Ignition Gazebo Fortress.
- Main runtime: `visual_fast_lio_navigation.launch.py`.
- Main planning source: FAST-LIO2 aligned SLAM map `/Laser_map_world`.
- Main path output: `/pct_path` from `terrain_pct_planner`.
- Safety/control chain: `/cmd_vel_nav -> velocity_smoother -> collision_monitor -> diff_drive_controller`.
- SDF is the Gazebo environment and test data source, not the default planner truth for the current FAST-LIO2 demo.
- Fastest verified display route: single-floor FAST-LIO/PCT/direct demo via
  `DEMO_TARGET=long_corridor bash scripts/run_fast_lio_single_floor_demo.sh`.
- Final single-floor freeze report:
  `docs/handoff/single_floor_final_report_2026-05-16.md`.
- First perfected SLAM/Nav2/FAST-LIO costmap display baseline:
  `phase1-perfect-slam-nav-fastlio-costmap` at commit `826e525`.
- Planner-only comparison showcase:
  `docs/planner_comparison_showcase_2026-05-17.md`.
- Cross-level remains partial: high `/pct_path` and command propagation are
  verified, but Gazebo physical ascent/high-goal arrival is not.

## Do Not Assume

- Do not assume a high `/pct_path` means the robot physically climbed to the high deck.
- Do not use old "fully unfinished" notes to ignore the latest high-path evidence.
- Do not use old "fully accepted" language to claim high-deck physical arrival.
- Do not treat `results/`, `log/`, `build/`, or `install/` as source.
- Do not commit root-level PDF/DOCX course materials unless explicitly asked.
