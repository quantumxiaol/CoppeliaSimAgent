# Skills / Tooling

这个目录当前提供一个不经过 LLM 的 `toolcli`，用于直接查看和调用 `coppeliasimagent` 已注册的工具。

## toolcli

位置：

- `skills/toolcli.py`
- 安装后也可直接使用项目脚本 `uv run coppelia-toolcli`

作用：

- 列出当前可用工具
- 查看某个工具的参数 schema
- 直接以 JSON payload 调用工具

示例：

```bash
uv run coppelia-toolcli --help
uv run coppelia-toolcli list
uv run coppelia-toolcli show load_model
uv run coppelia-toolcli call find_objects --payload '{"name_query":"jar","include_types":["shape"]}'
uv run coppelia-toolcli call get_plugin_status
uv run coppelia-toolcli call get_relative_pose --payload '{"source_handle":45,"target_handle":20}'
uv run coppelia-toolcli call step_simulation --payload '{"steps":5}'
uv run coppelia-toolcli call execute_joint_trajectory --payload '{"joint_handles":[31,32],"waypoints":[[0.1,0.2],[0.2,0.3]],"mode":"target_position"}'
uv run coppelia-toolcli call simulate_polishing_step --payload '{"tool_handle":42,"surface_cloud_handle":99,"contact_radius":0.03}'
python skills/toolcli.py list
```

## 当前工具清单

场景与对象：

- `get_simulation_state`
- `get_plugin_status`
- `collect_remote_api_diagnostics`
- `start_simulation`
- `pause_simulation`
- `stop_simulation`
- `step_simulation`
- `wait_seconds`
- `wait_until_state`
- `wait_until_object_pose_stable`
- `get_scene_graph`
- `find_objects`
- `get_object_pose`
- `get_relative_pose`
- `check_collision`
- `check_collision_monitor`
- `spawn_primitive`
- `spawn_visual_primitive`
- `spawn_visual_cylinder`
- `spawn_physics_proxy`
- `spawn_composite_object`
- `spawn_cuboid`
- `set_object_pose`
- `remove_object`
- `duplicate_object`
- `rename_object`
- `set_object_color`
- `set_object_visibility`
- `load_model`
- `set_parent_child`
- `spawn_waypoint`

关节与驱动：

- `get_joint_position`
- `get_joint_mode`
- `set_joint_mode`
- `get_joint_dyn_ctrl_mode`
- `set_joint_dyn_ctrl_mode`
- `set_joint_position`
- `set_joint_target_position`
- `get_joint_target_force`
- `set_joint_target_force`
- `get_joint_force`
- `set_joint_target_velocity`
- `configure_abb_arm_drive`
- `find_robot_joints`
- `setup_abb_arm_ik`
- `execute_joint_trajectory`
- `execute_cartesian_waypoints`
- `execute_stepped_ik_path_checked`
- `verify_joint_positions_reached`
- `verify_object_moved`
- `verify_object_velocity_below`
- `verify_force_threshold`

移动底盘与 IK：

- `set_youbot_wheel_velocities`
- `drive_youbot_base`
- `stop_youbot_base`
- `set_youbot_base_locked`
- `setup_ik_link`
- `setup_youbot_arm_ik`
- `move_ik_target`
- `move_ik_target_checked`

夹爪：

- `actuate_gripper`
- `actuate_youbot_gripper`
- `attach_object_to_gripper`
- `detach_object`
- `grasp_object`
- `release_object`

动力学属性：

- `get_object_velocity`
- `reset_dynamic_object`
- `set_shape_dynamics`

ABB task skills:

- `create_pusher_tool_for_abb`
- `create_tabletop_push_scene`
- `push_object_with_abb`

传感器与接触监控：

- `read_proximity_sensor`
- `read_force_sensor`
- `get_vision_sensor_image`
- `check_collision_monitor`

点云打磨：

- `create_point_cloud_surface_from_shape`
- `create_point_cloud_pottery_cylinder`
- `insert_points_into_point_cloud`
- `remove_points_near_tool`
- `get_point_cloud_stats`
- `simulate_polishing_step`
- `simulate_polishing_contact`
- `execute_polishing_groove`
- `execute_polishing_path`

运行态 live 验证脚本：

```bash
uv run test/live_task_abb_joint_trajectory.py \
  --model-path "robot_ttm/ABB IRB 4600-40-255.ttm"

uv run test/live_task_point_cloud_polishing.py
```

说明：

- 每个工具的参数定义来自 `agent/tool_registry.py` 中绑定的 Pydantic `input_model`
- 精确参数请用 `show` 查看，不要靠猜
- `call` 会真实调用底层工具；如果当前没有连上 CoppeliaSim，对需要仿真连接的工具会直接报错
- `get_plugin_status` 用来区分 simulator 侧插件问题（如 `simIK`）与 Python wrapper / `pyzmq` / `cbor2` 问题
- `collect_remote_api_diagnostics` 用于收集 Remote API 连接状态、插件状态、仿真状态、场景对象样本、匹配对象位姿和异常 traceback；当 ZMQ server 不响应时也会返回连接失败上下文。
- 不要并行开多个 `toolcli` 做机器人动作；涉及 stepping、IK 执行或接触动力学时，使用单个任务级工具独占完成，例如 `push_object_with_abb`。
- `create_point_cloud_pottery_cylinder` 默认 `use_explicit_points=True`，避免 cylinder 物理代理 workaround 把陶罐点云采成方盒表面。
- `spawn_visual_cylinder` 只负责真实视觉圆柱；需要动力学推动时，用 `spawn_physics_proxy(proxy_type="cylinder_proxy")` 或 `spawn_composite_object` 组合视觉体和隐藏物理代理。
- 机器人闭环任务优先用 `move_ik_target_checked` / `execute_stepped_ik_path_checked` / `create_tabletop_push_scene` / `push_object_with_abb`，它们会返回残差、关节变化、接触/移动结果和结构化失败原因。
- 涉及动力学运动时，先用 `get_simulation_state` 检查是否为 running；若未运行，先调用 `start_simulation`
- `start_simulation`、`pause_simulation`、`stop_simulation` 返回时可能还是过渡态；调用后要再执行一次 `get_simulation_state`，确认最终稳态
