#pragma once

#include "modules/ball_state.hpp"
#include "modules/field_potential.hpp"
#include "modules/localization.hpp"
#include "modules/strategy.hpp"
#include "modules/self_state.hpp"
#include "common/module.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace brain {

using namespace std;

struct HeadRequest {
    double pitch = 0.4;
    double yaw = 0;
    string mode = "search_left";

    string detail() const {
        ostringstream out;
        out << fixed << setprecision(2) << "mode=" << mode
            << "\npitch=" << pitch << " yaw=" << yaw;
        return out.str();
    }
};

struct MotionRequest {
    // -1 表示交给 Motion 的默认路由；非负值明确指定 walk_models 的下标。
    int walkSlot = -1;
    double forward = 0;
    double left = 0;
    double turn = 0;

    string detail() const {
        ostringstream out;
        out << fixed << setprecision(2) << "slot=" << walkSlot
            << " forward=" << forward
            << " left=" << left << "\nturn=" << turn;
        return out.str();
    }
};

struct KickRequest {
    bool active = false;
    unsigned modelSlot = 0;
    double x = 0, y = 0;
    double direction = 0;
    double power = 2;

    string detail() const {
        ostringstream out;
        out << fixed << setprecision(2) << "active=" << active
            << " slot=" << modelSlot
            << " ball=(" << x << ',' << y << ")"
            << "\ndir=" << direction << " power=" << power;
        return out.str();
    }
};

// SkillModule 是行动层：一次只执行一个 Strategy 选出的技能。
// 这里的盯球与扫球迁自原 Demo 的 HTWK 头控：新图像只计算一次目标，
// 头部以连续时间平滑跟随；丢球后保留最后方向，再无突变地进入扫描。
class SkillModule : public Module {
public:
    struct Config {
        double localBallHold = 2.0;
        double searchTurn = 0.4;
    };

    SkillModule(const Strategy& strategy, const BallState& ball,
                const LocalizationState& localization, const FieldPotential& potential,
                const SelfState& self, const double& now)
        : strategy_(strategy), ball_(ball), localization_(localization),
          potential_(potential), self_(self), now_(now) {}

    HeadRequest head;
    MotionRequest motion;
    KickRequest kick;

    ModuleInfo info() const override {
        return {"Skill", {"Strategy", "BallState", "LocalizationState",
                           "FieldPotential", "SelfState", "FrameTime"},
                {"HeadRequest", "MotionRequest", "KickRequest"}};
    }

    void configure(rclcpp::Node& node) override {
        config_.localBallHold = node.declare_parameter<double>("Skill.local_ball_hold", config_.localBallHold);
        config_.searchTurn = node.declare_parameter<double>("Skill.search_turn", config_.searchTurn);
    }

    void run() override {
        head = updateHead();
        motion.forward = 0;
        motion.left = 0;
        motion.turn = strategy_.skill == Strategy::FindBall ? config_.searchTurn : 0;
        kick.active = strategy_.skill == Strategy::VisualKick && potential_.valid;
        if(kick.active) {
            kick.modelSlot = 0;
            // Potential 给出的是场地坐标系中的出球方向；踢球接口需要机器人坐标系。
            kick.x = ball_.ball.relative.x;
            kick.y = ball_.ball.relative.y;
            kick.direction = wrapAngle(potential_.directionToField - localization_.poseToField.yaw);
            kick.power = 2;
        }
    }

private:
    struct HeadControlState {
        bool initialized = false;
        bool hasBallDirection = false;
        uint64_t observedRevision = 0;
        int lastBallSide = 1;
        double lastSeen = 0, lastUpdate = 0, scanStart = 0, scanOffset = 0;
        double targetPitch = 0.4, targetYaw = 0;
        double commandedPitch = 0.4, commandedYaw = 0;
    } headState_;
    static double smooth(double current, double target, double dt, double tau) {
        if(dt <= 0) return current;
        const double alpha = 1 - exp(-dt / tau);
        return current + alpha * (target - current);
    }

    static double triangle(double time, double frequency, double amplitude) {
        const double pi = 3.141592653589793;
        double phase = time * frequency;
        while(phase < 0) phase += 2 * pi;
        while(phase > 2 * pi) phase -= 2 * pi;
        const double slow = 0.1;
        const double a = 1 / (2 * slow), b = slow / 2;
        if(phase <= slow) return amplitude * ((a * phase * phase + b) / pi * 2 - 1);
        if(phase < pi - slow) return amplitude * (phase / pi * 2 - 1);
        if(phase <= pi) {
            const double x = pi - phase;
            return amplitude * ((1 - (a * x * x + b) / pi) * 2 - 1);
        }
        if(phase < 2 * pi - slow)
            return amplitude * ((2 * pi - phase) / pi * 2 - 1);
        const double x = phase - 2 * pi;
        return amplitude * ((1 - (a * x * x + b) / pi) * 2 - 1);
    }

    double nearestScanTime(double pitch, double yaw, int side, double pitchAmplitude) const {
        const double pi = 3.141592653589793;
        const double period = 2 * pi / 3.0;
        double bestTime = 0, bestDistance = 1e9;
        for(double time = 0; time < period; time += 0.1 / 3.0) {
            const double scanYaw = side * 0.4 + triangle(time, side * 3.0, 0.523598775598);
            const double scanPitch = 0.518879020478 + triangle(time, 6.0, pitchAmplitude);
            const double distance = hypot(scanPitch - pitch, scanYaw - yaw);
            if(distance < bestDistance) { bestDistance = distance; bestTime = time; }
        }
        return bestTime;
    }

    HeadRequest updateHead() {
        const double currentPitch = self_.headAnglesValid ? self_.headPitch : 0.4;
        const double currentYaw = self_.headAnglesValid ? self_.headYaw : 0;
        if(!headState_.initialized || now_ < headState_.lastUpdate) {
            headState_.initialized = true;
            headState_.commandedPitch = currentPitch;
            headState_.commandedYaw = currentYaw;
            headState_.targetPitch = headState_.commandedPitch;
            headState_.targetYaw = headState_.commandedYaw;
            headState_.lastUpdate = now_;
        }

        const double dt = min(0.1, max(0.0, now_ - headState_.lastUpdate));
        HeadRequest result;
        if(ball_.visible) {
            headState_.scanStart = 0;
            if(ball_.detectionRevision != headState_.observedRevision) {
                // 投影坐标已是机身坐标。直接用它给出绝对头目标，避免把异步图像
                // 的像素误差叠加到更新后的关节反馈，形成延迟造成的越追越偏。
                headState_.targetYaw = ball_.ball.yaw;
                headState_.targetPitch = ball_.ball.pitch;
                headState_.observedRevision = ball_.detectionRevision;
                headState_.lastSeen = now_;
                headState_.hasBallDirection = true;
                if(fabs(headState_.targetYaw) > 0.05)
                    headState_.lastBallSide = headState_.targetYaw > 0 ? 1 : -1;
            }
            result.mode = "track_local_ball";
            headState_.commandedPitch = smooth(headState_.commandedPitch, headState_.targetPitch, dt, 0.014);
            headState_.commandedYaw = smooth(headState_.commandedYaw, headState_.targetYaw, dt, 0.022);
        } else if(headState_.hasBallDirection && now_ - headState_.lastSeen < config_.localBallHold) {
            result.mode = "hold_last_ball_direction";
            headState_.commandedPitch = smooth(headState_.commandedPitch, headState_.targetPitch, dt, 0.014);
            headState_.commandedYaw = smooth(headState_.commandedYaw, headState_.targetYaw, dt, 0.022);
        } else {
            if(headState_.scanStart == 0) {
                const double pitchAmplitude = 0.318879020478;
                headState_.scanStart = now_;
                headState_.scanOffset = nearestScanTime(headState_.commandedPitch,
                                                        headState_.commandedYaw,
                                                        headState_.lastBallSide,
                                                        pitchAmplitude);
            }
            const double time = now_ - headState_.scanStart + headState_.scanOffset;
            const double pitchCenter = 0.518879020478;
            const double pitchAmplitude = 0.318879020478;
            headState_.commandedPitch = pitchCenter + triangle(time, 6.0, pitchAmplitude);
            headState_.commandedYaw = headState_.lastBallSide * 0.4
                                     + triangle(time, headState_.lastBallSide * 3.0, 0.523598775598);
            result.mode = headState_.lastBallSide > 0 ? "search_left" : "search_right";
        }
        headState_.lastUpdate = now_;
        result.pitch = headState_.commandedPitch;
        result.yaw = headState_.commandedYaw;
        return result;
    }

    const Strategy& strategy_;
    const BallState& ball_;
    const LocalizationState& localization_;
    const FieldPotential& potential_;
    const SelfState& self_;
    const double& now_;
    Config config_;
};

}
