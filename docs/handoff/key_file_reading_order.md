# Key Files and Reading Order

Status: current handoff index for code and document orientation.
Last updated: 2026-05-15.

## Read First

1. `docs/handoff/pre_migration_handoff_report.md`
2. `docs/handoff/current_status_overview.md`
3. `docs/handoff/pre_migration_risk_cleanup_record.md`
4. `docs/handoff/next_model_cautions.md`
5. `docs/handoff/new_model_initialization_prompt.md`

## Runtime Entry Points

- `src/airos_experiments/launch/visual_fast_lio_navigation.launch.py`
- `src/airos_experiments/airos_experiments/terrain_pct_planner.py`
- `src/airos_experiments/airos_experiments/slam_traversability_graph.py`
- `src/airos_nav/config/nav2_params.yaml`
- `src/airos_control/config/go2w_controllers.yaml`
- `scripts/cleanup_airos_runtime.sh`

## Regression Tests

- `src/airos_experiments/test/test_slam_traversability_graph.py`
- `src/airos_experiments/test/test_terrain_pointcloud_planner.py`
- `src/airos_experiments/test/test_visual_pointcloud_config.py`
- `src/airos_experiments/test/test_control_command_chain.py`

## Historical Context

- `README.md`
- `docs/go2w_fast_lio_upgrade_notes.md`
- `docs/advanced_planning_research_profile.md`
- `docs/AIROS_phased_execution_plan.md`
- `docs/AIROS_autonomous_navigation_technical_route.md`
- `task_plan.md`
- `findings.md`
- `progress.md`

## Do Not Treat As Source

- `build/`
- `install/`
- `log/`
- `results/`
- `.pytest_cache/`
- root-level PDF/DOCX course materials unless the user explicitly asks to process them.

## Source Priority

When files conflict, use this order:

1. Current code, tests, launch files, and fresh command output.
2. `docs/handoff/`.
3. Updated `README.md`.
4. Focused technical notes under `docs/`.
5. Historical planning/progress files.
