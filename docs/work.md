你是 CoppeliaSim 外部控制 Agent。请使用 skills/SKILLS.md 中的 toolcli 工具完成一次“ABB IRB4600 在仿真运行态推动桌面圆柱”的物理演示。必须先查看相关工具 schema，不要猜参数。

目标：
1. 检查 CoppeliaSim 连接、仿真状态和 simIK 插件状态。
2. 在新场景中创建蓝色方块桌面，替代桌子模型：
   - 使用 spawn_primitive 创建 primitive="cuboid"。
   - 建议桌面尺寸 size=[1.10,0.70,0.08]，颜色 color=[0.1,0.25,0.9]。
   - 建议桌面中心 position=[0.65,0.0,0.74]，此时 table_top_z=0.78。
   - 桌面必须是静态可碰撞支撑面：使用 set_shape_dynamics 设置 static=true、respondable=true。
   - 可按 ABB 可达性在合理范围内调整桌面 x/y/z 和尺寸，但桌面底部必须不低于 z=0。
   - 读取 table handle、pose、size，并确认 table_top_z。
3. 加载 ABB IRB4600：
   - model_path="robot_ttm/ABB IRB 4600-40-255.ttm"
   - base z 必须为 0，禁止把机械臂放到地面以下。
   - 初始建议 position=[-0.55,0.0,0.0]
   - 可调整 x/y/yaw，但只能在地面平面内调整。
4. 创建桌面动态可碰撞圆柱：
   - primitive="cylinder"
   - size=[0.16,0.16,0.18]
   - center=[0.65,0.0,table_top_z+0.09]
   - dynamic=true
   - 使用 set_shape_dynamics 设置 static=false、respondable=true、mass=0.5、friction=0.6。
5. 建立 ABB IK：
   - 查找 /IRB4600 的 6 个关节。
   - 查找 /IRB4600/IkTip 和 /IRB4600/IkTarget。
   - 使用 setup_abb_arm_ik 建立 simIK 链路。
   - 验证 IkTip 能随 IkTarget 移动。
6. 在仿真运行态用 ABB 末端推动圆柱：
   - 先配置 ABB 关节驱动，查看 configure_abb_arm_drive schema 后调用。
   - 启动仿真后，不要直接移动圆柱。
   - 让 IkTarget/IkTip 走一条推送路径，例如：
     pre_contact=[0.45,0.0,table_top_z+0.09]
     contact_start=[0.56,0.0,table_top_z+0.09]
     push_end=[0.82,0.0,table_top_z+0.09]
   - 分 40-80 个小步移动 IkTarget，每步后 step_simulation。
   - 用实际 IkTip pose 和圆柱 pose 记录推动过程。
   - 如果 IkTip 不跟随或 ABB 在运行态跳离路径，请先尝试：
     - 停止仿真并 reset dynamics
     - configure_abb_arm_drive dynamic/position
     - 让 IkTarget 和 IkTip 在启动前重合
     - 调整机械臂 ground x/y/yaw 或圆柱桌面 x/y
     - 但禁止把 ABB base z 调到负数
7. 汇报结果：
   - simIK 是否可用
   - table handle、table pose、table size、table_top_z
   - ABB robot handle、joint handles、IkTip handle、IkTarget handle
   - ABB final base pose
   - 圆柱 handle、初始/最终 pose、位移距离
   - IkTip 初始/最终 pose
   - 是否发生真实仿真物理推动
   - 如果没有真实推动，要如实说明，不要把直接改圆柱位置或停止态 IK 说成仿真推动。
