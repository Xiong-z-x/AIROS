# Objective Completion Audit - 2026-05-16

This audit maps the current user objective to concrete artifacts and runtime
evidence. It is intentionally conservative: proxy signals such as passing tests,
high `/pct_path`, or RViz visuals are not treated as cross-level completion.

## Objective Restatement

The requested end state is:

1. Keep the current Fortress + Go2W surrogate + FAST-LIO2 + PCT-style chain.
2. First prove single-floor autonomous navigation works end to end.
3. Then prove cross-level autonomous navigation end to end:
   live SLAM map growth, high `/pct_path`, direct execution command chain, and
   Gazebo physical pose actually climbing and reaching the high target.
4. Fix real path-planning and execution issues found on the way.
5. Clean obvious generated garbage without deleting source or evidence.
6. Consider open-source code, Unitree-style models, and better maps only when
   they shorten the path to a verified result.

## Requirement To Evidence Checklist

| Requirement | Evidence | Status |
| --- | --- | --- |
| Current launch uses FAST-LIO2 SLAM-cloud terrain chain | `visual_fast_lio_navigation.launch.py` defaults to `terrain_map_source:=slam_cloud`, `/Laser_map_world`, `/pct_path`, `nav_stack_mode:=safety_only`, `terrain_execution_mode:=direct` | Satisfied |
| Single-floor SLAM map growth | `near_goal_after_scan_index_20260516_223232`: `/Laser_map_world` max `248275`; `long_corridor_after_scan_index_20260516_223440`: max `351188` | Satisfied |
| Single-floor PCT-style planning | `long_corridor_after_scan_index_20260516_223440`: `/pct_path_poses=10`, `/pct_path_max_z=-0.044546`; launch log shows final goal became reachable after FAST-LIO map update | Satisfied |
| Single-floor direct execution command chain | `near_goal`: `/cmd_vel_nav=200`, smoother/base `235`; `long_corridor`: `/cmd_vel_nav=573`, smoother `645`, base `660` | Satisfied |
| Single-floor physical arrival | `near_goal` final Gazebo distance `0.222402m`; `long_corridor` final Gazebo distance `0.294605m`; both under `0.30m` script threshold | Satisfied |
| `/slam_scan` freshness and safety chain | `scan_freshness_after_support_index_20260516_222625`: stale warning count `0`, base command `1033`; later single-floor and cross-level runs also stale warning count `0` | Satisfied |
| Cross-level live SLAM growth | `cross_level_after_zigzag_lookahead_fix_20260516_225624`: `/Laser_map_world` max `621554`; earlier `cross_level_after_single_floor_refresh_20260516_223943`: max `793960` | Satisfied |
| Cross-level high `/pct_path` | `cross_level_after_zigzag_lookahead_fix_20260516_225624`: `/pct_path_max_z=2.249888`; `accepted_high_path=true` | Satisfied |
| Cross-level direct command chain | `cross_level_after_zigzag_lookahead_fix_20260516_225624`: `/cmd_vel_nav=2335`, smoother/base `2686`, final command ages below `0.1s` | Satisfied |
| Cross-level physical Gazebo ascent | `cross_level_after_zigzag_lookahead_fix_20260516_225624`: `gazebo_z_max=-0.005001`, `accepted_physical_high=false` | Not satisfied |
| Cross-level high-target arrival | Same run: final Gazebo goal distance `11.985970m` | Not satisfied |
| Path-planning risk fixes | Added/fixed frontier entry scoring, high-drop rejection, final-path regression guard, height-debt lookahead, path-tangent lookahead, `/slam_scan` support-index filtering, lifecycle retry | Partially satisfied |
| Avoid wasting time on repeated long failed runs | Latest handoff says to stop repeating long cross-level trials on the same surrogate without model/map/execution change | Satisfied as process boundary |
| Clean generated garbage | Removed `log/build_*` and `.pytest_cache`; `log` reduced from `68M` to `9.1M`; source and key evidence kept | Satisfied |
| Consider open-source/Unitree model shortcuts | Decision recorded: do not import large external model/code in this turn because current verified blocker is physical cross-level execution and fastest deliverable is single-floor; model/map replacement should be a separate branch with acceptance tests | Deferred |

## Completion Verdict

The full cross-level objective is **not complete**.

2026-05-17 scope update: the user explicitly narrowed the active work to
finishing the single-floor navigation baseline, leaving cross-level interfaces
and a complete problem report, then stopping. Under that narrowed scope, the
single-floor deliverable is frozen in
`docs/handoff/single_floor_final_report_2026-05-16.md`. The original cross-level
goal remains incomplete and must not be marked complete in the goal tool.

The single-floor demonstration objective is complete and reproducible:

```bash
DEMO_TARGET=long_corridor bash scripts/run_fast_lio_single_floor_demo.sh
```

The cross-level chain is partially complete:

- Fact: live SLAM map growth works.
- Fact: high `/pct_path` generation works.
- Fact: direct execution command chain works.
- Fact: `/slam_scan` freshness and collision-monitor activation are fixed.
- Pending: Gazebo pose still does not physically climb or reach the high target.

## Recommended Next Action

For fastest visible progress, ship the single-floor demo result now. Do not run
more long cross-level trials on the same wheel surrogate unless one of these
changes first:

1. a simpler validated multilevel demo map with explicit physical-z acceptance,
2. a ramp-capable model/control replacement branch,
3. or a targeted execution fix proven by a small ramp-entry smoke test before a
   full cross-level run.
