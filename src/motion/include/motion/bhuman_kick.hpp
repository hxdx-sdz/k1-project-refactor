#pragma once

#include "motion/policy.hpp"

namespace motion {

// Copyright (c) 2026 B-Human. All rights reserved.
// 许可条件与免责声明见随包 models/BHuman-LICENSE.md。
// B-Human 2026 WalkingEngine 的 K1 kick_inner / kick_inner_soft 观测与输出约定。
// 只负责 12 个腿关节；头和手臂继续由 Motion 持有。
class BHumanKick {
public:
    void reset(const Eigen::Vector2d& ball) {
        phase_ = 0;
        frequencyOffset_ = 0;
        previousBall_ = limitedBall(ball);
    }

    Eigen::VectorXd observation(const Eigen::Vector3d& gravity, const Eigen::Vector3d& gyro,
                                const Eigen::VectorXd& position, const Eigen::VectorXd& velocity,
                                const Eigen::VectorXd& lastTarget, const Eigen::Vector2d& ball,
                                const Eigen::Vector2d& ballVelocity, double direction,
                                double power, bool soft) {
        const double pi = 3.14159265358979323846;
        const Eigen::Vector2d currentBall = limitedBall(ball);
        if((currentBall - previousBall_).norm() > 0.2) previousBall_ = currentBall;
        Eigen::VectorXd input = Eigen::VectorXd::Zero(59);
        input.segment(0, 3) = gravity;
        input.segment(3, 3) = gyro;
        input.segment(6, 2) = currentBall * 0.1;
        input[8] = std::atan2(std::sin(direction), std::cos(direction)) / pi;
        input[9] = std::cos(2 * pi * phase_);
        input[10] = std::sin(2 * pi * phase_);
        for(int i = 0; i < 12; ++i) {
            const int joint = 10 + i;
            const double measured = std::max(lowerLimit(i), std::min(upperLimit(i), position[joint]));
            input[11 + i] = measured - offset(i);
            input[23 + i] = velocity[joint] * 0.1;
            input[35 + i] = lastTarget[joint] - offset(i);
        }
        input[47] = frequencyOffset_;
        // 48..50：普通、允许过冲、精确踢；51..53：球速度（z 固定为零）。
        input.segment(51, 2) = limitedBall(ballVelocity) * 0.1;
        input[54] = std::sin(direction);
        input[55] = std::cos(direction);
        input[56] = std::max(soft ? 0.1 : 0.5, std::min(soft ? 1.7 : 3.4, power)) * 0.1;
        input.segment(57, 2) = previousBall_ * 0.1;
        previousBall_ = currentBall;
        return input;
    }

    void apply(const Eigen::VectorXd& action, Eigen::VectorXd& target, double dt) {
        if(action.size() != 13 || !action.allFinite())
            throw std::runtime_error("BHuman kick needs 13 finite actions");
        for(int i = 0; i < 12; ++i)
            target[10 + i] = std::max(-2.0, std::min(2.0, action[i])) + offset(i);
        frequencyOffset_ = std::max(-0.5, std::min(0.5, action[12]));
        phase_ = std::fmod(phase_ + dt * (1.5 + frequencyOffset_), 1.0);
    }

private:
    static Eigen::Vector2d limitedBall(const Eigen::Vector2d& value) {
        return value * (2.0 / std::max(2.0, value.norm()));
    }

    static double offset(int joint) {
        const double values[6] = {-0.2, 0, 0, 0.4, -0.25, 0};
        return values[joint % 6];
    }

    // 上游 K1 jointLimits.cfg；只裁剪测量输入，输出沿用上游 [-2, 2] 残差约定。
    static double lowerLimit(int joint) {
        const double degrees[12] = {-171, -22, -59, 0, -49.8, -19.76,
                                    -171, -89, -59, 0, -49.8, -19.76};
        return degrees[joint] * 3.14159265358979323846 / 180;
    }

    static double upperLimit(int joint) {
        const double degrees[12] = {126, 89, 59, 135, 19.76, 19.76,
                                   126, 22, 59, 135, 19.76, 19.76};
        return degrees[joint] * 3.14159265358979323846 / 180;
    }

    double phase_ = 0;
    double frequencyOffset_ = 0;
    Eigen::Vector2d previousBall_ = Eigen::Vector2d::Zero();
};

}
