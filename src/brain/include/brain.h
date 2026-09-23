#pragma once

#include "modules/field_provider.hpp"
#include "modules/self_state.hpp"
#include "modules/observations.hpp"
#include "modules/localization.hpp"
#include "modules/ball_state.hpp"
#include "modules/field_potential.hpp"
#include "modules/strategy.hpp"
#include "modules/skill.hpp"
#include "modules/actuation.hpp"
#include "modules/rerun.hpp"
#include "common/scheduler.h"
#include <functional>

namespace brain {

using namespace std;

// Brain 把独立算法组织成数据流：从端口取输入，按顺序调用模块，
// 再把结果交给日志出口。这里只组装模块，模块内部集中定义自己的执行逻辑。
// 外部输入先创建，模块随后绑定；两者的地址在整个运行期间保持不变。
class Brain {
public:
    typedef function<void(const string&)> LogSink;
    InputPort<Odometry> odometry;
    InputPort<JointData> joint;
    InputPort<GameState> game;
    InputPort<VisionState> vision;
    InputPort<FieldLines> fieldLines;
    InputPort<HeadPose> headPose;
    InputPort<RecoveryState> recovery;

    explicit Brain(const LogSink& sink) : sink_(sink) {}

    void init(rclcpp::Node& node) {
        // 模块清单：只注册实例，不在这里编写模块计算或输出保存逻辑。
        scheduler_.add(field_);
        scheduler_.add(localization_);
        scheduler_.add(observations_);
        scheduler_.add(self_);
        scheduler_.add(ball_);
        scheduler_.add(fieldPotential_);
        scheduler_.add(strategy_);
        scheduler_.add(skill_);
        scheduler_.add(actuation_);
        scheduler_.add(rerun_);
        scheduler_.configure(node);
        scheduler_.prepare({"Odometry", "JointData", "GameState", "VisionState", "FrameTime",
                            "FieldLines", "HeadPose", "RecoveryState"});
        const string dependencies = scheduler_.describe();
        rerun_.setSchedule(dependencies, scheduler_.planInfos(), scheduler_.externalInputs());
        sink_(dependencies);
        if(rerun_.enabled()) sink_("[rerun] connecting: " + rerun_.serverIp());
    }

    void tick(double now) {
        frame_.now = now;
        frame_.odometry = odometry.read();
        frame_.joint = joint.read();
        frame_.game = game.read();
        frame_.vision = vision.read();
        frame_.fieldLines = fieldLines.read();
        frame_.headPose = headPose.read();
        frame_.recovery = recovery.read();
        scheduler_.run();
    }

    const LocalizationState& localization() const { return localization_.output; }

private:
    // 外部输入与模块实例分开管理；输入先于使用它的模块构造。
    InputFrame frame_;

    // 模块实例区：声明和输入绑定放在一起，新增模块只需在此添加并在 init 注册。
    // 需要上游结果时绑定其输出数据，不把整个上游模块传入。
    FieldProviderModule field_;
    SelfStateModule self_{frame_.odometry, frame_.joint, frame_.headPose, frame_.recovery};
    ObservationModule observations_{frame_.vision, frame_.fieldLines};
    LocalizationModule localization_{field_.output, self_.output, observations_.output, frame_.now};
    BallStateModule ball_{observations_.output, frame_.now};
    FieldPotentialModule fieldPotential_{field_.output, localization_.output, ball_.output};
    StrategyModule strategy_{ball_.output, frame_.now};
    SkillModule skill_{strategy_.output, ball_.output, field_.output, localization_.output,
                       fieldPotential_.output, self_.output, frame_.now};
    ActuationModule actuation_{skill_.head, skill_.motion, skill_.kick};
    RerunModule rerun_{field_.output, self_.output, observations_.output,
                       localization_.output, ball_.output, fieldPotential_.output, strategy_.output, skill_.head,
                       skill_.motion, skill_.kick, actuation_.output, frame_.now};

    Scheduler scheduler_;
    LogSink sink_;
};

}
