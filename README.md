# K1 Project Refactor

这是 K1 官方 Demo 的重构实验。
目标是把原来集中在巨大 Brain 文件中的数据处理、策略选择和动作出口重新分开
使每条数据链能够被阅读、调试和逐步替换。

默认读者已经了解官方 Demo、ROS2 topic 和 K1 的基本运行方式。

## 核心想法

官方 Demo 真正的问题是定位、球状态、场地模型、目标方向等业务数据没有清晰的生产者、消费者和流向，处理逻辑因而与策略、执行和调试代码相互渗透。

新架构先把这些数据链显式化：各模块持续产出稳定的数据，Strategy 再读取它们并决定当前 Skill。

```text
外部输入
  ↓
状态与观测整理
  ↓
世界模型与估计
  ↓
Strategy 选择 Skill
  ↓
HeadRequest / MoveRequest / KickRequest
  ↓
Motion 运行模型并输出关节目标
```

这里的重点不是“模块数量”，而是数据关系：每个模块必须明确写出自己读取什么、产出什么。看到一个输出类型时，应能顺着依赖图找到它的来源和消费者。

## 当前架构

Brain 是固定 tick 的数据流执行器。启动时根据各模块声明的输入输出做依赖检查和拓扑排序；运行时只按固定顺序调用模块，不在每帧重新排图。

```text
Odometry, JointData, VisionState, FieldLines, GameState, HeadPose, RecoveryState
  ├─→ SelfState
  ├─→ Observations
  └─→ FieldProvider
          ↓
SelfState + Observations + FieldModel ─→ Localization
Observations ──────────────────────────→ BallState
FieldModel + Localization + BallState + Observations ─→ FieldPotential
BallState ─────────────────────────────→ Strategy
Strategy + BallState + FieldPotential ─→ Skill
Skill ─────────────────────────────────→ Actuation ─→ motion topics
```

各层的职责如下：

- **SelfState**：自身里程计、关节、头部和恢复状态的统一表达。
- **Observations**：将视觉目标和场线整理成机器人坐标系下的观测，不决定其场地语义。
- **FieldProvider**：唯一的场地尺寸和理论标线来源。
- **Localization**：维护机器人到场地坐标系的位姿估计。
- **BallState**：对球观测做确认、平滑和短时保持，避免单帧漏检直接扰动策略。
- **FieldPotential**：计算场地偏好、近处机器人/人员斥力和建议出球方向，不决定是否踢球，也不代替身体路径规划。
- **Strategy**：选择当前高层 Skill，例如找球（FindBall）或带球（Dribble）。
- **Skill**：将 Strategy 翻译成头部、移动和踢球请求。
- **Actuation**：Brain 与运控包之间的薄适配层，只转发请求，不保存动作状态机。
- **Motion**：独立的低层模型运行包，读取请求和 LowState，运行官方导出模型，最终准备关节目标。
- **Rerun**：只观察数据流和世界状态，不向业务链路写回数据。

## 模块约定

一个 Module 是围绕明确算法职责组织的计算单元，不等于 ROS 节点、线程、行为树节点或单个函数。

模块自身文件应集中放置：输入绑定、输出类型、配置、私有历史、`info()`、`run()` 和算法实现。小模块保持 header-only；只有状态、依赖重量或代码规模确实需要时，才拆出 `.cpp`。

新增模块通常只需做两件事：

1. 在 `src/brain/modules/` 新建模块头文件，声明输入输出并实现算法。
2. 在 `src/brain/include/brain.h` 中创建实例、绑定上游输出，并在 `Brain::init()` 注册。

不要为新增模块修改调度器、建立全局数据字典，或让模块持有整个 Brain / 上游模块对象。下游只读取上游输出数据；模块自己的跨帧历史留在自己内部。

## Strategy、Skill 与 Motion

三者的边界必须保持清楚：

```text
Strategy：现在想做什么？
Skill：为完成它，需要哪些高层请求？
Motion：如何把请求变成身体关节目标？
```

`KickRequest` 包含球相对位置、出球方向和 `model_slot`。`model_slot` 是策略明确选择的模型槽位；Motion 不应暗中根据 power 或其他启发式重新路由模型。

Motion 当前兼容官方导出的行走模型 `exported_policy_k1.bin`、原地转向模型与 Kick138 模型 `k1_kick_policy_138_v1.bin`。它不复刻旧 Demo 的高层 VisualKick RPC，而是通过 Custom 接管、站立准备和低层关节输出运行模型。当前带球技能统一叫 Dribble，底层模型文件与协议名称保持兼容。

踢球默认 slot 0 保留 Kick138，slot 1/2 分别为 B-Human 2026 的 K1 `kick_inner` / `kick_inner_soft`。两者使用独立的 59 维观测与 12 腿关节输出适配，复用现有 Eigen 推理；Brain 当前临时统一选择 slot 1。来源许可见 `src/motion/models/BHuman-LICENSE.md`。

低层输出具有实机风险。当前随包配置为 `Motion.enabled: true`，但仍需手柄 A 进入 Custom，B 选择 Brain 控制，X 选择手柄控制；Start 退出到 Walking，Back 切到 Damping。不要把“启动节点”与“允许机器人运动”混为一谈；测试前确认周围安全与退出方式。

## 配置

ROS 参数文件保留 `节点名 → ros__parameters` 外层。业务参数的第一层使用拥有它的模块名，且与 `ModuleInfo::name` 完全一致；参数名使用 `snake_case`。

```yaml
brain_node:
  ros__parameters:
    BallState:
      confirm_frames: 2
```

这是开发约定，不是调度器自动拼接出的规则。模块自行声明完整参数路径，配置、代码和文档必须同步修改。

## 调试

Rerun 是默认调试入口：

- `brain/graph`：启动时生成的数据依赖图；节点表示数据类型，而非模块实例。
- `field/map`：场地模型。
- `field/localization`：定位状态与位姿。
- `field/ball`、`field/kick_dir`：球位与势能场建议的出球方向。

需要新增调试信息时，优先让模块通过自己的输出类型或少量明确的 debug 数据表达；不要为了临时排查把字符串摘要和日志分支塞回 Brain 主程序。

## 设计取舍

- 保留 C++、ROS2 和固定 tick；不迁移成 Python Brain。
- 抽象是为了减少理解成本，不是为了“框架化”。避免无必要的模板层、回调注册、宏和隐式控制流。
- 不按“一个概念一对 hpp/cpp”机械拆文件；同一语义内容应放在一起。
- 调度图只描述同帧数据依赖；跨帧反馈应作为模块私有历史显式保存，不能形成隐式环。
- Rerun、日志和 ROS 发布都是出口，不应反向污染业务计算。

## 目录

```text
src/
  brain/
    common/       调度、端口、公共输入与几何
    modules/      Brain 业务模块
    include/      Brain 组装
    src/          ROS 节点入口
    config/       Brain 参数
  motion/
    msg/          Brain 到低层的请求类型
    include/      模型读取与推理基础
    src/          Motion 节点入口
    config/       模型路径与安全开关
```

## 功能进度与 TODO

当前目标是先稳定单机闭环，再逐步补齐比赛功能，不逐节点照搬旧 Demo。
以下对照以本地 `K1_5v5_Demo_v1.6` 的功能入口为参考；该目录也含自研改动，不等同于未经修改的官方发布版。
勾选表示代码链路已经实现，不代表全部工况完成验收；实机结果单独注明。

### 已完成的基础链路

- [x] 模块基础：最新值输入端口、更新事件、启动依赖检查与拓扑排序、固定 tick。
- [x] 感知整理：自身状态、关节、视觉物体及场线观测进入独立数据模块。
- [x] 球状态：连续帧确认、位置平滑、短时保持，减少漏检导致的技能反复切换。
- [x] 头控与找球：对应旧 Demo 的 `CamTrackBall`、`CamFindBall`、`RobotFindBall`，实现盯球、扫球与原地转身。
- [x] 标志点定位：全局初始化、局部校正与里程计预测；初始化假定在己方负 x 半场。仍有对称跳变，见 P0。
- [x] 场地与出球方向：场地尺寸集中管理；势能场覆盖边界、球门、对方角落，并接入 Dribble 请求。
- [x] Dribble 执行：`Strategy → Skill → Motion` 闭环，Kick138 直接运行；用户已有实机反馈“能正常踢球”，不代表比赛策略已完整。
- [x] Motion 基础：Custom 接管、头控、手柄接管与退出、walk/turn 默认路由、速度加减速缓冲、共享 walk 观测历史。
- [x] 里程计：随包提供 walk/RVK 模型，内部选槽，共享历史并对齐已检查固件的姿态与坐标处理；Brain 直接订阅输出。
- [x] 调试：兼容旧版 Rerun 的网络传输、自动数据依赖图、动态 detail、场地/定位/球/方向显示，以及低频性能汇总。

### P0：先解决当前闭环问题

- [ ] **新机器人视觉启动兼容**：补齐实际相机类型与对应配置，解决 `.248` 的 `camera.type` 缺失；验收为持续输出有效观测，而不只是节点不崩溃。
- [ ] **半场对称防跳变**：持续比较新解及其 180° 对称解与里程计预测的朝向；区分“已初始化”与“当前有效”。验收覆盖场地中部、转身、短暂失效后恢复及跨中线。
- [ ] **带球稳定性回归**：核对踢球时的里程计漂移、walk/turn/Dribble 切换和控制卡顿；以实测决定是否继续改时序，不先堆保护与状态机。

### P1：补齐单机比赛必需功能

- [ ] **裁判状态与比赛策略**：当前保留基础裁判输入；重新实现独立状态整理，并让 Initial / Ready / Set / Playing / Finished、处罚和开球权真正约束行动。用于验证规范的 GameController 测试实现已撤销。
- [ ] **场地走位**：补齐 `MoveToPoseOnField`、Ready/定位球站位和 `GoBackInField` 对应能力；验收为能到目标位置并正确停止。
- [ ] **自主起身与恢复**：对齐 `CheckAndStandUp` 的能力，明确 Custom 控制与起身控制的交接，以及恢复后的定位处理。
- [ ] **场线定位融合**：候选与校正代码已有，但因漂移默认关闭；先核对线观测、关联和校正效果，确认不引入漂移后再启用。

### P2：扩展比赛能力

- [ ] **对手状态与路径避障**：已将机器人/人员观测接入出球势能的近距离斥力；尚无障碍物跟踪和身体路径避障，势能项仍需实机验证。
- [ ] **角色与团队协作**：对齐 `StrikerDecide`、`GoalieDecide`、`RoleSwitchIfNeeded`、`Assist` 的职责，按需要加入抢球分工、协作和守门能力。
- [ ] **专用技能**：按比赛需要补齐 `Intercept`、守门封堵、绕球调整和独立射门等能力，不把 Dribble 等同于完整追球/调整/射门策略。

声音、挥手、调试动作及自动标定等辅助入口暂不列为主链迁移优先项。完成任务时更新对应条目与验收结果，不因有同名类或接口就勾选。
