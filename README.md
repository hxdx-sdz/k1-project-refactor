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
FieldModel + Localization + BallState ─→ FieldPotential
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
- **FieldPotential**：只计算场地偏好和建议出球方向，不决定是否踢球。
- **Strategy**：选择当前高层 Skill，例如找球或视觉踢球。
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

Motion 当前先兼容官方导出的视觉行走模型 `exported_policy_k1.bin` 与 Kick138 模型 `k1_kick_policy_138_v1.bin`。它不是复刻旧 Demo 的高层 VisualKick RPC；后续由它以最小接管流程进入 Custom 模式、准备站立并持续发送低层关节命令。

低层输出具有实机风险。默认 `Motion.enabled: false`，只执行模型 dry-run，不发布 `/joint_ctrl`。在确认关节顺序、控制类型、增益、prepare 流程和紧急退出前，不得直接打开。

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
- 场地图中可继续增加球、出球方向和其他世界模型可视化。

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

## 当前边界

这个项目正在迁移中。已经形成的主链是“观测 → 自身状态 / 定位 / 球状态 → 势能方向 → Strategy → Skill → Motion”。团队策略、协作、完整场线融合、低层 Custom 接管的实机验收仍是后续工作。
