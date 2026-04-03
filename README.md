# CoppeliaSimAgent

面向 CoppeliaSim 的外部 Agent 工程骨架。目标是把底层 Remote API 封装为可被 LLM Function Calling 稳定调用的工具集。

## Reference

[apiFunctions](https://manual.coppeliarobotics.com/en/apiFunctions.htm)

[zmqRemoteApiOverview](https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm)

## 当前状态

已完成第一阶段：

- `core/connection.py`: 全局连接管理、重连、`simIK/simOMPL` 预加载
- `tools/*`: 场景感知、基础几何体、模型管理、IK 与夹爪信号控制
- `agent/tool_registry.py`: 基于 Pydantic 的工具注册和 JSON Schema 导出
- `tests/*`: 连接管理与工具层单元测试（mock/fake sim）
- `test/*`: 手动执行的真机脚本（连通性与逐工具验证）

## 目录结构

```text
src/coppeliasimagent/
├── __init__.py
├── core/
│   ├── __init__.py
│   ├── connection.py
│   └── exceptions.py
├── tools/
│   ├── __init__.py
│   ├── schemas.py
│   ├── scene.py
│   ├── primitives.py
│   ├── models.py
│   └── kinematics.py
└── agent/
    ├── __init__.py
    └── tool_registry.py
```

## 环境要求

- CoppeliaSim `v4.6.0+`
- Python `3.13+`
- `uv`

## 安装依赖

```bash
uv add coppeliasim-zmqremoteapi-client pyzmq cbor2 click pydantic openai python-dotenv numpy scipy
```

## 安装校验

```bash
uv pip show coppeliasimagent
```

如果输出包含 `Editable project location: /.../CoppeliaSimAgent`，说明当前环境已正确以可编辑模式指向本项目。

## 启动 CoppeliaSim

启动后确认日志包含：

```text
[sandboxScript:info] Simulator launched, welcome!
[Connectivity >> WebSocket remote API server@addOnScript:info] WebSocket Remote API server starting (port=23050)...
[Connectivity >> ZMQ remote API server@addOnScript:info] ZeroMQ Remote API server starting (rpcPort=23000)...
```

## 连通性测试

```bash
uv run test/connect_to_sim.py
```

可选参数：

```bash
uv run test/connect_to_sim.py --host 127.0.0.1 --zmq-port 23000 --timeout 3.0
uv run test/connect_to_sim.py --check-ws --ws-port 23050
```

注意：

- 默认不会探测 `23050`，以避免 `simWS ... handle_read_handshake ... End of File` 日志。
- 当你需要验证 WebSocket 端口时，再加 `--check-ws`。
- 以 `[PASS] ZMQ RPC OK ...` 为连接成功的关键判据。

## Live 工具脚本（逐个手动执行）

以下脚本会连接当前正在运行的 CoppeliaSim，并真实调用工具函数：

```bash
uv run test/live_tool_spawn_red_cuboid.py
uv run test/live_tool_move_red_cuboid.py
uv run test/live_tool_move_red_cuboid.py --handle 32 --target 0.2,0.0,0.3
uv run test/live_tool_remove_red_cuboid.py
uv run test/live_tool_remove_red_cuboid.py --handle 25
uv run test/live_tool_scene_graph.py --max-items 10
uv run test/live_tool_check_collision.py
uv run test/live_tool_set_parent_child.py
```

放置机器人模型（需要你提供 `.ttm` 路径）：

```bash
uv run test/live_tool_load_robot_model.py --model-path /absolute/path/to/robot.ttm
```

说明：

- 这些脚本不是 `unittest` 用例，因此文件名不以 `test_` 开头。
- `tests/test_*.py` 用于离线单元测试，`test/live_tool_*.py` 用于在线真实仿真测试。
- `test/live_tool_move_red_cuboid.py --handle <id>` 可移动指定句柄到 `--target`。
- `test/live_tool_remove_red_cuboid.py --handle <id>` 会删除指定句柄；如果目标是系统对象或无效句柄，会返回 `found invalid handles`。

## 坐标约定

- 位置向量统一使用 `[x, y, z]`。
- `z` 是高度（up 轴）。
- 示例：`[0.2, 0.0, 0.3]` 表示离地 0.3 m。

## 工具函数概览

### 场景感知

- `get_scene_graph(include_types, round_digits)`
- `check_collision(entity1, entity2)`

### 基础几何体

- `spawn_primitive(primitive, size, position, color, dynamic, relative_to)`
- `spawn_cuboid(size, position, color, dynamic, relative_to)`
- `set_object_pose(handle, position, orientation_deg, relative_to)`
- `remove_object(handle)`

### 模型与装配

- `load_model(model_path, position, orientation_deg, relative_to)`
- `set_parent_child(child_handle, parent_handle, keep_in_place)`

### 运动学与末端执行器

- `spawn_waypoint(position, size, relative_to)`
- `setup_ik_link(base_handle, tip_handle, target_handle, constraints_mask)`
- `move_ik_target(environment_handle, group_handle, target_handle, position, relative_to, steps)`
- `actuate_gripper(signal_name, closed)`

## Pydantic 校验约定

所有工具函数在执行前都会经过 Pydantic 校验，主要约束：

- 向量参数必须是长度为 3 的有限数值
- 颜色范围必须在 `[0, 1]`
- 尺寸参数必须大于 `0`
- 旋转输入统一使用角度制 (`orientation_deg`)，底层自动转换弧度

## Tool Registry（给 LLM 用）

`agent/tool_registry.py` 提供：

- `get_openai_tools()`: 导出 OpenAI Function Calling 所需 JSON Schema
- `invoke_tool(name, payload)`: 验证输入并调用工具函数

## 运行单元测试

```bash
uv run python -m unittest discover -s tests -p "test_*.py"
```

这些测试不依赖实时 CoppeliaSim 进程，主要覆盖：

- 连接管理（超时、重连、插件可用性）
- 工具函数参数校验与 API 调用行为
- Tool registry 的 schema 导出与校验拦截
