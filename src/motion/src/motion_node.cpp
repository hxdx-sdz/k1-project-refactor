#include "motion/policy.hpp"
#include <booster_interface/msg/low_cmd.hpp>
#include <booster_interface/msg/low_state.hpp>
#include <motion/msg/head_request.hpp>
#include <motion/msg/kick_request.hpp>
#include <motion/msg/move_request.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

using namespace std;

namespace {

const int kJointCount = 22;
const int kPolicyJointCount = 20;
const int kHistorySize = 10;

// 与 CupSim 的 K1 serial 模型一致：头部 0/1，之后是 20 个手臂和腿关节。
const int kRuntimeToLab[kPolicyJointCount] = {
    0, 4, 8, 14, 1, 5, 9, 15, 2, 6, 10, 16, 3, 7, 11, 17, 12, 18, 13, 19
};

Eigen::VectorXd defaultPose() {
    const double values[kJointCount] = {
        0, 0, 0.2, -1.25, 0, -0.5, 0.2, 1.25, 0, 0.5,
        -0.15, 0, 0, 0.3, -0.15, 0,
        -0.15, 0, 0, 0.3, -0.15, 0
    };
    Eigen::VectorXd result(kJointCount);
    for(int i = 0; i < kJointCount; ++i) result[i] = values[i];
    return result;
}

Eigen::Vector3d projectedGravity(const booster_interface::msg::ImuState& imu) {
    const double roll = imu.rpy[0];
    const double pitch = imu.rpy[1];
    return Eigen::Vector3d(sin(pitch), -cos(pitch) * sin(roll), -cos(pitch) * cos(roll));
}

struct PolicyContext {
    explicit PolicyContext(int frameSize) : history(frameSize), lastAction(Eigen::VectorXd::Zero(kPolicyJointCount)) {}

    motion::Policy policy;
    motion::History history;
    Eigen::VectorXd lastAction;

    void reset() {
        history.clear();
        lastAction.setZero();
    }
};

}

class MotionNode : public rclcpp::Node {
public:
    MotionNode()
        : Node("motion_node"), walk_(69) {
        enabled_ = declare_parameter<bool>("Motion.enabled", false);
        walkPath_ = declare_parameter<string>("Motion.walk_model", "");
        kickPaths_ = declare_parameter<vector<string> >("Motion.kick_models", vector<string>());

        for(size_t i = 0; i < kickPaths_.size(); ++i) kickPolicies_.push_back(PolicyContext(71));
        loadModels();

        lowStateSub_ = create_subscription<booster_interface::msg::LowState>(
            "/low_state", 1, [this](const booster_interface::msg::LowState::SharedPtr msg) {
                lowState_ = *msg;
                hasLowState_ = true;
            });
        moveSub_ = create_subscription<motion::msg::MoveRequest>(
            "/motion/move_request", 1, [this](const motion::msg::MoveRequest::SharedPtr msg) { move_ = *msg; });
        headSub_ = create_subscription<motion::msg::HeadRequest>(
            "/motion/head_request", 1, [this](const motion::msg::HeadRequest::SharedPtr msg) { head_ = *msg; });
        kickSub_ = create_subscription<motion::msg::KickRequest>(
            "/motion/kick_request", 1, [this](const motion::msg::KickRequest::SharedPtr msg) { kickRequest_ = *msg; });
        jointPublisher_ = create_publisher<booster_interface::msg::LowCmd>("/joint_ctrl", 1);
        timer_ = create_wall_timer(chrono::milliseconds(20), [this]() { tick(); });
    }

private:
    void loadModels() {
        try {
            walk_.policy.load(walkPath_);
            if(walk_.policy.inputSize() != 69 * kHistorySize || walk_.policy.outputSize() != kPolicyJointCount)
                throw runtime_error("walk policy must be 690 -> 20");
            for(size_t i = 0; i < kickPolicies_.size(); ++i) {
                kickPolicies_[i].policy.load(kickPaths_[i]);
                if(kickPolicies_[i].policy.inputSize() != 71 * kHistorySize || kickPolicies_[i].policy.outputSize() != kPolicyJointCount)
                    throw runtime_error("kick policy must be 710 -> 20");
            }
            modelsReady_ = true;
            RCLCPP_INFO(get_logger(), "loaded walk model and %zu kick model slots", kickPolicies_.size());
        } catch(const exception& error) {
            RCLCPP_ERROR(get_logger(), "motion models are not ready: %s", error.what());
        }
    }

    Eigen::VectorXd sharedFrame(const Eigen::VectorXd& lastAction) const {
        Eigen::VectorXd frame(66);
        const Eigen::Vector3d gravity = projectedGravity(lowState_.imu_state);
        for(int i = 0; i < 3; ++i) {
            frame[i] = lowState_.imu_state.gyro[i];
            frame[3 + i] = gravity[i];
        }
        const Eigen::VectorXd defaults = defaultPose();
        for(int lab = 0; lab < kPolicyJointCount; ++lab) {
            const int runtime = kRuntimeToLab[lab];
            const auto& joint = lowState_.motor_state_parallel[2 + runtime];
            frame[6 + lab] = joint.q - defaults[2 + runtime];
            frame[26 + lab] = joint.dq * 0.1;
            frame[46 + lab] = lastAction[lab];
        }
        return frame;
    }

    Eigen::VectorXd runWalk(const Eigen::VectorXd& shared) {
        Eigen::VectorXd frame(69);
        frame.segment(0, 66) = shared;
        frame[66] = move_.forward;
        frame[67] = move_.left;
        frame[68] = move_.turn;
        frame = walk_.history.push(frame);
        frame = frame.cwiseMax(-100).cwiseMin(100);
        walk_.lastAction = walk_.policy.run(frame).cwiseMax(-100).cwiseMin(100);
        return walk_.lastAction;
    }

    Eigen::VectorXd runKick(const Eigen::VectorXd& shared, PolicyContext& policy) {
        Eigen::VectorXd frame(71);
        frame.segment(0, 66) = shared;
        const double strength = max(0.0, kickRequest_.power);
        const double distance = hypot(kickRequest_.x, kickRequest_.y);
        const double scale = distance > 2.0 ? 2.0 / distance : 1.0;
        frame[66] = cos(kickRequest_.direction) * strength;
        frame[67] = sin(kickRequest_.direction) * strength;
        frame[68] = kickRequest_.x * scale;
        frame[69] = kickRequest_.y * scale;
        frame[70] = kickRequest_.ball_updated ? 1.0 : -1.0;
        frame = policy.history.push(frame);
        frame = frame.cwiseMax(-100).cwiseMin(100);
        policy.lastAction = policy.policy.run(frame).cwiseMax(-10).cwiseMin(10);
        return policy.lastAction;
    }

    void publishTarget(const Eigen::VectorXd& action) {
        Eigen::VectorXd target = defaultPose();
        target[0] = head_.yaw;
        target[1] = head_.pitch;
        for(int lab = 0; lab < kPolicyJointCount; ++lab) {
            const int runtime = kRuntimeToLab[lab];
            target[2 + runtime] += action[lab] * 0.25;
        }

        booster_interface::msg::LowCmd command;
        command.cmd_type = booster_interface::msg::LowCmd::CMD_TYPE_PARALLEL;
        command.motor_cmd.resize(kJointCount);
        for(int i = 0; i < kJointCount; ++i) {
            auto& motor = command.motor_cmd[i];
            motor.q = target[i];
            motor.dq = 0;
            motor.tau = 0;
            motor.kp = i < 10 ? 20 : (i == 14 || i == 15 || i == 20 || i == 21 ? 50 : 100);
            motor.kd = i < 10 ? 2 : (i == 14 || i == 15 || i == 20 || i == 21 ? 1 : 2);
            motor.weight = 1;
        }
        jointPublisher_->publish(command);
    }

    void tick() {
        if(!modelsReady_ || !hasLowState_) return;
        if(lowState_.motor_state_parallel.size() != kJointCount) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                  "parallel low state needs %d joints, got %zu", kJointCount,
                                  lowState_.motor_state_parallel.size());
            return;
        }
        Eigen::VectorXd action;
        string name = "walk";
        if(kickRequest_.active && kickRequest_.model_slot < kickPolicies_.size()) {
            const int slot = kickRequest_.model_slot;
            if(activeKickSlot_ != slot) {
                kickPolicies_[slot].reset();
                activeKickSlot_ = slot;
            }
            action = runKick(sharedFrame(kickPolicies_[slot].lastAction), kickPolicies_[slot]);
            name = "kick slot " + to_string(slot);
        } else {
            activeKickSlot_ = -1;
            action = runWalk(sharedFrame(walk_.lastAction));
        }
        if(enabled_) publishTarget(action);
        if(++ticks_ % 50 == 0)
            RCLCPP_INFO(get_logger(), "policy=%s output=%s", name.c_str(), enabled_ ? "joint_ctrl" : "dry_run");
    }

    bool enabled_ = false;
    bool modelsReady_ = false;
    bool hasLowState_ = false;
    int activeKickSlot_ = -1;
    unsigned ticks_ = 0;
    string walkPath_;
    vector<string> kickPaths_;
    booster_interface::msg::LowState lowState_;
    motion::msg::MoveRequest move_;
    motion::msg::HeadRequest head_;
    motion::msg::KickRequest kickRequest_;
    PolicyContext walk_;
    vector<PolicyContext> kickPolicies_;
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSub_;
    rclcpp::Subscription<motion::msg::MoveRequest>::SharedPtr moveSub_;
    rclcpp::Subscription<motion::msg::HeadRequest>::SharedPtr headSub_;
    rclcpp::Subscription<motion::msg::KickRequest>::SharedPtr kickSub_;
    rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr jointPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<MotionNode>());
    rclcpp::shutdown();
    return 0;
}
