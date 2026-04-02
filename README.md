# CoppeliaSimAgent

面向 CoppeliaSim 的外部 Agent 开发脚手架，当前重点是先打通与仿真器的远程连接（ZMQ + WebSocket 端口可达性）。

## 环境要求

- CoppeliaSim `v4.6.0+`
- Python `3.13+`（与当前 `pyproject.toml` 一致）
- `uv`

## 安装依赖

在项目根目录执行：

```bash
uv add coppeliasim-zmqremoteapi-client pyzmq cbor2 click pydantic openai python-dotenv numpy scipy
```

如果你是首次执行，`uv` 会自动创建 `.venv`。

## 启动 CoppeliaSim

先启动 CoppeliaSim，再观察底部日志，确认两个服务都已启动：

```text
[sandboxScript:info] Simulator launched, welcome!
[Connectivity >> WebSocket remote API server@addOnScript:info] WebSocket Remote API server starting (port=23050)...
[Connectivity >> ZMQ remote API server@addOnScript:info] ZeroMQ Remote API server starting (rpcPort=23000)...
```

## 连接测试

项目提供了连接测试脚本：`tests/Connect2Sim.py`

```bash
uv run python tests/Connect2Sim.py
```

可选参数：

```bash
uv run python tests/Connect2Sim.py --host 127.0.0.1 --zmq-port 23000 --ws-port 23050 --timeout 3.0
```

测试内容：

- 检查 ZMQ 端口 (`23000`) TCP 可达
- 检查 WebSocket 端口 (`23050`) TCP 可达
- 通过 `coppeliasim-zmqremoteapi-client` 执行一次真实 RPC 调用（读取仿真时间步长）

全部通过会返回 `exit code 0`；任意失败会返回 `exit code 1`。

## 常见问题

- `Connection refused`：通常是 CoppeliaSim 未启动，或端口配置不是 `23000/23050`。
- 长时间无响应：先确认本机防火墙策略，再确认 CoppeliaSim 的 Remote API 插件是否被禁用。
- 主机不是本地：把 `--host` 改成远程 IP。
