#pragma once

#include "common/input_types.hpp"
#include "common/module.h"
#include <iomanip>
#include <sstream>

namespace brain {

using namespace std;

struct SelfState {
    Sample<Pose2D> odometry;
    Sample<HeadPose> headPose;
    Sample<RecoveryState> recovery;
    bool headAnglesValid = false;
    double headYaw = 0, headPitch = 0;
    uint64_t jointRevision = 0;

    string detail() const {
        ostringstream out;
        out << fixed << setprecision(2)
            << "odom=(" << odometry.value.x << ',' << odometry.value.y << ','
            << odometry.value.yaw << ")\nrecovery=";
        if(recovery.valid()) out << recovery.value.state << " planner=" << recovery.value.planner;
        else out << "waiting";
        return out.str();
    }
};

// 自身状态只解释身体反馈，不读取定位结果
class SelfStateModule : public Module {
public:
    struct Config { double odomFactor = 1.2; }; // Demo 比例值，需按实机校准

    SelfStateModule(const Sample<Odometry>& odom, const Sample<JointData>& joint,
                    const Sample<HeadPose>& head, const Sample<RecoveryState>& recovery)
        : odom_(odom), joint_(joint), head_(head), recovery_(recovery) {}
    SelfState output;
    ModuleInfo info() const override {
        return {"SelfState", {"Odometry", "JointData", "HeadPose", "RecoveryState"}, {"SelfState"}};
    }
    void configure(rclcpp::Node& node) override {
        config_.odomFactor = node.declare_parameter<double>("SelfState.odom_factor", config_.odomFactor);
    }
    void run() override { output = update(odom_, joint_, head_, recovery_, config_); }
    static SelfState update(const Sample<Odometry>& odom, const Sample<JointData>& joint,
                            const Sample<HeadPose>& head, const Sample<RecoveryState>& recovery,
                            const Config& config) {
        SelfState result;
        // 里程计
        result.odometry.value = {odom.value.x * config.odomFactor, odom.value.y * config.odomFactor, wrapAngle(odom.value.yaw)};
        result.odometry.revision = odom.revision;
        result.odometry.updatedAt = odom.updatedAt;

        result.headPose = head;
        result.recovery = recovery;
        result.jointRevision = joint.revision;
        
        // 头部关节角
        result.headAnglesValid = joint.valid() && joint.value.serial.size() >= 2;
        if(result.headAnglesValid) {
            result.headYaw = joint.value.serial[0].q;
            result.headPitch = joint.value.serial[1].q;
        }
        return result;
    }
private:
    const Sample<Odometry>& odom_;
    const Sample<JointData>& joint_;
    const Sample<HeadPose>& head_;
    const Sample<RecoveryState>& recovery_;
    Config config_;
};

}
