# Open Source Building Reference Asset

This visual-only model is derived from `ypat999/3d_dog_navi_ros2`.

- Source repository: <https://github.com/ypat999/3d_dog_navi_ros2>
- Source path:
  `src/ignition_models/gazebo_garden_migration/models/Building/Building.dae`
- License in source repository: Academic Free License 3.0
- Copied into AIROS as an optional visual asset for the
  `open_source_scene_assets:=true` launch profile.

The model intentionally omits mesh collision in AIROS. Navigation collision,
route validation, and Nav2 map generation remain tied to the verified local SDF
geometry in `advanced_indoor_ramp.sdf`.
