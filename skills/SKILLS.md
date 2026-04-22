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
python skills/toolcli.py list
```

## 当前工具清单

场景与对象：

- `get_scene_graph`
- `find_objects`
- `check_collision`
- `spawn_primitive`
- `spawn_cuboid`
- `set_object_pose`
- `remove_object`
- `duplicate_object`
- `rename_object`
- `set_object_color`
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

移动底盘与 IK：

- `set_youbot_wheel_velocities`
- `drive_youbot_base`
- `stop_youbot_base`
- `set_youbot_base_locked`
- `setup_ik_link`
- `setup_youbot_arm_ik`
- `move_ik_target`

夹爪：

- `actuate_gripper`
- `actuate_youbot_gripper`

说明：

- 每个工具的参数定义来自 `agent/tool_registry.py` 中绑定的 Pydantic `input_model`
- 精确参数请用 `show` 查看，不要靠猜
- `call` 会真实调用底层工具；如果当前没有连上 CoppeliaSim，对需要仿真连接的工具会直接报错
