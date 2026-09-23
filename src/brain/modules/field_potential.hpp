#pragma once

#include "modules/ball_state.hpp"
#include "modules/field_provider.hpp"
#include "modules/localization.hpp"
#include "common/module.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace brain {

using namespace std;

// directionToField 是势场下降方向 = 应该朝哪个（全局）方向踢
struct FieldPotential {
    bool valid = false;
    Pose2D ballToField;
    double directionToField = 0;
    double score = 0;
    double borderScore = 0;
    double goalScore = 0;
    double cornerScore = 0;

    string detail() const {
        ostringstream out;
        out << fixed << setprecision(2) << "valid=" << valid;
        if(valid) {
            out << "\nball=(" << ballToField.x << ',' << ballToField.y << ')'
                << "\ndirection=" << directionToField << " score=" << score
                << "\nborder=" << borderScore << " goal=" << goalScore
                << " corner=" << cornerScore;
        }
        return out.str();
    }
};

// FieldPotential 是不关心比赛状态的基础场地算法
// 它只计算“球往哪里去更好”；是否踢、何时踢由 Strategy 和 Skill 决定。

class FieldPotentialModule : public Module {
public:
    struct Config {
        double borderRange = 0.8;
        double borderWeight = 3.0;
        double goalWeight = 1.5;
        double cornerWeight = 0.5;
    };

    FieldPotentialModule(const FieldModel& field, const LocalizationState& localization,
                         const BallState& ball)
        : field_(field), localization_(localization), ball_(ball) {}

    FieldPotential output;

    ModuleInfo info() const override {
        return {"FieldPotential", {"FieldModel", "LocalizationState", "BallState"},
                {"FieldPotential"}};
    }

    void configure(rclcpp::Node& node) override {
        config_.borderRange = node.declare_parameter<double>(
            "FieldPotential.border_range", config_.borderRange);
        config_.borderWeight = node.declare_parameter<double>(
            "FieldPotential.border_weight", config_.borderWeight);
        config_.goalWeight = node.declare_parameter<double>(
            "FieldPotential.goal_weight", config_.goalWeight);
        config_.cornerWeight = node.declare_parameter<double>(
            "FieldPotential.corner_weight", config_.cornerWeight);
    }

    void run() override {
        output = FieldPotential();
        if(!ball_.visible || !localization_.valid) return;

        output.valid = true;
        output.ballToField = compose(localization_.poseToField, ball_.ball.relative);
        const FieldDimensions& f = field_.dimensions;
        const double halfLength = f.length / 2;
        const double halfWidth = f.width / 2;
        double forceX = 0, forceY = 0;

        addBorder(halfWidth - output.ballToField.y, 0, -1, forceX, forceY);
        addBorder(output.ballToField.y + halfWidth, 0, 1, forceX, forceY);
        addBorder(output.ballToField.x + halfLength, 1, 0, forceX, forceY);

        // 对方底线的球门开口是可通行的；仅在球门两侧把球推回场内。
        if(abs(output.ballToField.y) > f.goalWidth / 2)
            addBorder(halfLength - output.ballToField.x, -1, 0, forceX, forceY);

        addGoalForce(halfLength, forceX, forceY);
        addCornerForce(halfLength, forceX, forceY);
        output.score = output.borderScore + output.goalScore + output.cornerScore;
        if(forceX != 0 || forceY != 0) output.directionToField = atan2(forceY, forceX);
    }

private:
    void addBorder(double distance, double directionX, double directionY,
                   double& forceX, double& forceY) {
        if(distance >= config_.borderRange) return;
        const double ratio = max(0.0, (config_.borderRange - distance) / config_.borderRange);
        output.borderScore += config_.borderWeight * ratio * ratio;
        const double strength = 2 * config_.borderWeight * ratio / config_.borderRange;
        forceX += directionX * strength;
        forceY += directionY * strength;
    }

    void addGoalForce(double goalX, double& forceX, double& forceY) {
        const double dx = goalX - output.ballToField.x;
        const double dy = -output.ballToField.y;
        const double distance = sqrt(dx * dx + dy * dy);
        const double range = hypot(field_.dimensions.length, field_.dimensions.width);
        output.goalScore = -config_.goalWeight * max(0.0, 1.0 - distance / range);
        if(distance > 0) {
            forceX += dx / distance * config_.goalWeight / range;
            forceY += dy / distance * config_.goalWeight / range;
        }
    }

    void addCornerForce(double goalX, double& forceX, double& forceY) {
        const FieldDimensions& f = field_.dimensions;
        const double cornerStart = goalX - f.penaltyAreaLength;
        if(output.ballToField.x <= cornerStart || abs(output.ballToField.y) <= f.goalWidth / 2) return;
        const double ratio = (output.ballToField.x - cornerStart) / f.penaltyAreaLength;
        output.cornerScore = config_.cornerWeight * ratio * ratio;
        const double strength = 2 * config_.cornerWeight * ratio / f.penaltyAreaLength;
        forceX -= strength;
    }

    const FieldModel& field_;
    const LocalizationState& localization_;
    const BallState& ball_;
    Config config_;
};

}
