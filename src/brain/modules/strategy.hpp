#pragma once

#include "modules/ball_state.hpp"
#include "common/module.h"

namespace brain {

using namespace std;

// Strategy 只选择当前要执行的高层技能，不携带速度、关节角或话题细节。
// 后续策略可以替换这里的选择规则，而不影响具体技能和动作出口。
struct Strategy {
    enum Skill { Dribble, FindBall } skill = FindBall;

    string detail() const {
        return skill == Dribble ? "skill=dribble" : "skill=find_ball";
    }
};

class StrategyModule : public Module {
public:
    struct Config {
        double dribbleExitDelay = 1.0;
        double dribbleExitCooldown = 2.0;
    };

    StrategyModule(const BallState& ball, const double& now) : ball_(ball), now_(now) {}

    Strategy output;

    ModuleInfo info() const override {
        return {"Strategy", {"BallState", "FrameTime"}, {"Strategy"}};
    }

    void configure(rclcpp::Node& node) override {
        config_.dribbleExitDelay = node.declare_parameter<double>(
            "Strategy.dribble_exit_delay", config_.dribbleExitDelay);
        config_.dribbleExitCooldown = node.declare_parameter<double>(
            "Strategy.dribble_exit_cooldown", config_.dribbleExitCooldown);
    }

    void run() override {
        if(output.skill == Strategy::Dribble) {
            if(ball_.visible || now_ - ball_.seenAt < config_.dribbleExitDelay) return;

            // 已退出的 Dribble 需要完整冷却一段时间，不能因下一帧又看到球立刻重入。
            // 这是给底层动作控制器完成退出和重新接管的稳定窗口。
            output.skill = Strategy::FindBall;
            dribbleAvailableAt_ = now_ + config_.dribbleExitCooldown;
            return;
        }

        if(ball_.visible && now_ >= dribbleAvailableAt_)
            output.skill = Strategy::Dribble;
    }

private:
    const BallState& ball_;
    const double& now_;
    Config config_;
    double dribbleAvailableAt_ = 0;
};

}
