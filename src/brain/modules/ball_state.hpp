#pragma once

#include "modules/observations.hpp"
#include "common/module.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace brain {

using namespace std;

// BallState 是“当前有没有可靠本机球”的单一出口。
// 之后的决策、头控和动作都不再各自遍历视觉检测列表。
struct BallState {
    bool visible = false;
    ObservedObject ball;
    uint64_t detectionRevision = 0;
    double seenAt = 0;

    string detail() const {
        if(!visible) return "not visible";
        ostringstream out;
        out << lround(ball.yaw * 180 / 3.141592653589793) << "° "
            << fixed << setprecision(2) << ball.range << "m";
        return out.str();
    }
};

class BallStateModule : public Module {
public:
    struct Config {
        double minConfidence = 50;
        double inputTimeout = 0.3;
        double holdDuration = 0.3;
        double positionSmoothing = 0.35;
        int confirmFrames = 2;
    };

    BallStateModule(const Observations& observations, const double& now)
        : observations_(observations), now_(now) {}

    BallState output;

    ModuleInfo info() const override {
        return {"BallState", {"Observations", "FrameTime"}, {"BallState"}};
    }

    void configure(rclcpp::Node& node) override {
        config_.minConfidence = node.declare_parameter<double>(
            "BallState.min_confidence", config_.minConfidence);
        config_.inputTimeout = node.declare_parameter<double>(
            "BallState.input_timeout", config_.inputTimeout);
        config_.holdDuration = node.declare_parameter<double>(
            "BallState.hold_duration", config_.holdDuration);
        config_.positionSmoothing = node.declare_parameter<double>(
            "BallState.position_smoothing", config_.positionSmoothing);
        config_.confirmFrames = node.declare_parameter<int>(
            "BallState.confirm_frames", config_.confirmFrames);
    }

    void run() override {
        // 保持窗口结束后，下一次球必须重新连续确认，不能拿旧估计直接恢复可见。
        if(confirmed_ && now_ - output.seenAt > config_.holdDuration) {
            confirmed_ = false;
            hasBall_ = false;
            seenFrames_ = 0;
        }
        // Brain 的 tick 比视觉快。同一帧观测只允许进入滤波器一次，
        // 否则会把一张旧图像误当成多次独立测量。
        if(observations_.detectionRevision != handledRevision_) {
            handledRevision_ = observations_.detectionRevision;
            updateFromObservation();
        }
        output.visible = confirmed_;
    }

private:
    void updateFromObservation() {
        if(now_ - observations_.receivedAt > config_.inputTimeout) return;
        const ObservedObject* best = nullptr;
        for(const ObservedObject& ball : observations_.balls) {
            if(ball.confidence < config_.minConfidence) continue;
            if(!best || ball.confidence > best->confidence) best = &ball;
        }
        if(!best) {
            seenFrames_ = 0;
            return;
        }

        const double alpha = max(0.0, min(1.0, config_.positionSmoothing));
        if(!hasBall_) {
            output.ball = *best;
            filteredX_ = best->relative.x;
            filteredY_ = best->relative.y;
            filteredPitch_ = best->pitch;
            hasBall_ = true;
        } else {
            filteredX_ += alpha * (best->relative.x - filteredX_);
            filteredY_ += alpha * (best->relative.y - filteredY_);
            filteredPitch_ += alpha * (best->pitch - filteredPitch_);
            output.ball = *best;
            output.ball.relative = {filteredX_, filteredY_, 0};
            output.ball.range = hypot(filteredX_, filteredY_);
            output.ball.yaw = atan2(filteredY_, filteredX_);
            output.ball.pitch = filteredPitch_;
        }
        output.seenAt = now_;
        output.detectionRevision = observations_.detectionRevision;
        ++seenFrames_;
        if(seenFrames_ >= max(1, config_.confirmFrames)) confirmed_ = true;
    }

    const Observations& observations_;
    const double& now_;
    Config config_;
    uint64_t handledRevision_ = 0;
    bool hasBall_ = false;
    bool confirmed_ = false;
    int seenFrames_ = 0;
    double filteredX_ = 0, filteredY_ = 0, filteredPitch_ = 0;
};

}
