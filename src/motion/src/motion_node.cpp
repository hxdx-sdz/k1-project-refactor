#include "motion/policy.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <booster_interface/message_utils.hpp>
#include <booster_interface/msg/low_cmd.hpp>
#include <booster_interface/msg/low_state.hpp>
#include <booster_interface/msg/remote_controller_state.hpp>
#include <booster_msgs/msg/rpc_req_msg.hpp>
#include <motion/msg/head_request.hpp>
#include <motion/msg/kick_request.hpp>
#include <motion/msg/move_request.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

namespace {

const int kJointCount = 22;
const int kPolicyJointCount = 20;
const int kHistorySize = 10;
const int kControlIntervalMs = 2;
const int kPolicyControlSteps = 10;
const int kBrainMoveTimeoutMs = 250;
const int kDefaultWalkSlot = -1;
const int kTurnWalkSlot = -2;

// 模型实验室顺序到 K1 serial 关节顺序的映射。头部在 0/1，模型控制 2..21。
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
        : Node("motion_node"), turn_(69), target_(defaultPose()) {
        config_.enabled = declare_parameter<bool>("Motion.enabled", false);
        config_.maxForward = declare_parameter<double>("Motion.max_forward", 1.25);
        config_.maxBackward = declare_parameter<double>("Motion.max_backward", 0.5);
        config_.maxLeft = declare_parameter<double>("Motion.max_left", 0.5);
        config_.maxTurn = declare_parameter<double>("Motion.max_turn", 2.5);
        config_.forwardAcceleration = declare_parameter<double>("Motion.forward_acceleration", 1.3);
        config_.forwardDeceleration = declare_parameter<double>("Motion.forward_deceleration", 2.0);
        config_.leftAcceleration = declare_parameter<double>("Motion.left_acceleration", 1.2);
        config_.leftDeceleration = declare_parameter<double>("Motion.left_deceleration", 1.2);
        config_.turnAcceleration = declare_parameter<double>("Motion.turn_acceleration", 2.0);
        config_.turnDeceleration = declare_parameter<double>("Motion.turn_deceleration", 2.0);
        config_.walkPaths = declare_parameter<vector<string> >("Motion.walk_models", vector<string>());
        config_.turnPath = declare_parameter<string>("Motion.turn_model", "");
        config_.kickPaths = declare_parameter<vector<string> >("Motion.kick_models", vector<string>());

        for(size_t i = 0; i < config_.walkPaths.size(); ++i) walkPolicies_.push_back(PolicyContext(69));
        for(size_t i = 0; i < config_.kickPaths.size(); ++i) kickPolicies_.push_back(PolicyContext(71));
        loadModels();

        lowStateSub_ = create_subscription<booster_interface::msg::LowState>(
            "/low_state", 1, [this](const booster_interface::msg::LowState::SharedPtr msg) {
                lowState_ = *msg;
                runtime_.hasLowState = true;
            });
        moveSub_ = create_subscription<motion::msg::MoveRequest>(
            "/motion/move_request", 1, [this](const motion::msg::MoveRequest::SharedPtr msg) {
                brain_.move = *msg;
                brain_.moveReceived = chrono::steady_clock::now();
                brain_.hasMove = true;
            });
        headSub_ = create_subscription<motion::msg::HeadRequest>(
            "/motion/head_request", 1, [this](const motion::msg::HeadRequest::SharedPtr msg) { brain_.head = *msg; });
        kickSub_ = create_subscription<motion::msg::KickRequest>(
            "/motion/kick_request", 1, [this](const motion::msg::KickRequest::SharedPtr msg) { brain_.kick = *msg; });
        controllerSub_ = create_subscription<booster_interface::msg::RemoteControllerState>(
            "/remote_controller_state", 1,
            [this](const booster_interface::msg::RemoteControllerState::SharedPtr msg) { controller(*msg); });
        jointPublisher_ = create_publisher<booster_interface::msg::LowCmd>("/joint_ctrl", 1);
        modePublisher_ = create_publisher<booster_msgs::msg::RpcReqMsg>("LocoApiTopicReq", 1);
        timer_ = create_wall_timer(chrono::milliseconds(kControlIntervalMs), [this]() { tick(); });
    }

private:
    enum ControllerMode { Idle, CustomReady, Running };

    // 配置只由 YAML 决定；运行中不改变。
    struct MotionConfig {
        bool enabled = false;
        double maxForward;
        double maxBackward;
        double maxLeft;
        double maxTurn;
        double forwardAcceleration;
        double forwardDeceleration;
        double leftAcceleration;
        double leftDeceleration;
        double turnAcceleration;
        double turnDeceleration;
        vector<string> walkPaths;
        string turnPath;
        vector<string> kickPaths;
    } config_;

    // 默认由 Brain 的高层请求驱动；X 键可临时改为手柄完整接管。
    struct BrainInput {
        motion::msg::MoveRequest move;
        motion::msg::HeadRequest head;
        motion::msg::KickRequest kick;
        chrono::steady_clock::time_point moveReceived;
        bool hasMove = false;
    } brain_;

    // 手柄输入：平时作为 Brain 移动请求超时后的备用，X 接管时作为唯一输入。
    struct ManualInput {
        motion::msg::MoveRequest move;
    } manual_;

    // 手柄同时提供备用移动输入和模式切换边沿。
    struct ButtonState {
        bool b = false;
        bool a = false;
        bool x = false;
        bool back = false;
        bool start = false;
    } previousButtons_;

    struct PendingAction {
        bool enterCustom = false;
        bool selectBrain = false;
        bool selectManual = false;
        bool damping = false;
        bool exitToWalking = false;
    } pending_;

    // 这些状态随控制周期演化，不和 YAML、输入消息混放。
    struct RuntimeState {
        ControllerMode mode = Idle;
        bool hasLowState = false;
        bool manualControl = false;
        int activeWalkSlot = -3;
        int activeKickSlot = -1;
        unsigned controlSteps = 0;
        uint64_t requestSequence = 0;
    } runtime_;

    // 这是模型实际看到的速度命令。它只负责让输入连续，不承担策略判断。
    motion::msg::MoveRequest appliedMove_;


    struct GainProfile {
        double upperBodyKp;
        double upperBodyKd;
        double legKp;
        double legKd;
        double ankleKp;
        double ankleKd;
    };

    struct JointGains {
        double kp;
        double kd;
    };

    void loadModels() {
        walkReady_.assign(walkPolicies_.size(), false);
        for(size_t i = 0; i < walkPolicies_.size(); ++i) {
            try {
                const string path = modelPath(config_.walkPaths[i]);
                walkPolicies_[i].policy.load(path);
                if(walkPolicies_[i].policy.inputSize() != 69 * kHistorySize
                   || walkPolicies_[i].policy.outputSize() != kPolicyJointCount)
                    throw runtime_error("walk policy must be 690 -> 20");
                walkReady_[i] = true;
                RCLCPP_INFO(get_logger(), "loaded walk slot %zu: %s", i, path.c_str());
            } catch(const exception& error) {
                RCLCPP_WARN(get_logger(), "walk slot %zu is unavailable: %s", i, error.what());
            }
        }

        try {
            const string path = modelPath(config_.turnPath);
            turn_.policy.load(path);
            if(turn_.policy.inputSize() != 69 * kHistorySize || turn_.policy.outputSize() != kPolicyJointCount)
                throw runtime_error("turn policy must be 690 -> 20");
            turnReady_ = true;
            RCLCPP_INFO(get_logger(), "loaded turn model: %s", path.c_str());
        } catch(const exception& error) {
            RCLCPP_WARN(get_logger(), "turn model is unavailable: %s", error.what());
        }

        kickReady_.assign(kickPolicies_.size(), false);
        for(size_t i = 0; i < kickPolicies_.size(); ++i) {
            try {
                kickPolicies_[i].policy.load(modelPath(config_.kickPaths[i]));
                if(kickPolicies_[i].policy.inputSize() != 71 * kHistorySize
                   || kickPolicies_[i].policy.outputSize() != kPolicyJointCount)
                    throw runtime_error("kick policy must be 710 -> 20");
                kickReady_[i] = true;
            } catch(const exception& error) {
                RCLCPP_WARN(get_logger(), "kick slot %zu is unavailable: %s", i, error.what());
            }
        }
    }

    string modelPath(const string& fileName) const {
        return ament_index_cpp::get_package_share_directory("motion") + "/models/" + fileName;
    }

    void controller(const booster_interface::msg::RemoteControllerState& joy) {
        manual_.move.forward = joy.ly < 0 ? -joy.ly * config_.maxForward
                                           : -joy.ly * config_.maxBackward;
        manual_.move.left = joy.lx * config_.maxLeft;
        manual_.move.turn = -joy.rx * config_.maxTurn;
        manual_.move.walk_slot = -1;

        if(joy.a && !previousButtons_.a) pending_.enterCustom = true;
        if(joy.b && !previousButtons_.b) pending_.selectBrain = true;
        if(joy.x && !previousButtons_.x) pending_.selectManual = true;
        if(joy.back && !previousButtons_.back) pending_.damping = true;
        if(joy.start && !previousButtons_.start) pending_.exitToWalking = true;
        previousButtons_.b = joy.b;
        previousButtons_.a = joy.a;
        previousButtons_.x = joy.x;
        previousButtons_.back = joy.back;
        previousButtons_.start = joy.start;
    }

    const motion::msg::MoveRequest& moveRequest() const {
        if(runtime_.manualControl) return manual_.move;
        if(brain_.hasMove && chrono::steady_clock::now() - brain_.moveReceived
            < chrono::milliseconds(kBrainMoveTimeoutMs)) return brain_.move;
        return manual_.move;
    }

    double smoothSpeed(double current, double requested, double acceleration, double deceleration) const {
        const double interval = kControlIntervalMs * kPolicyControlSteps / 1000.0;
        if(current * requested < 0) {
            const double step = deceleration * interval;
            if(abs(current) <= step) return 0;
            return current - copysign(step, current);
        }

        const double limit = abs(requested) > abs(current) ? acceleration : deceleration;
        const double step = limit * interval;
        if(requested > current) return min(current + step, requested);
        return max(current - step, requested);
    }

    void updateAppliedMove() {
        const motion::msg::MoveRequest& requested = moveRequest();
        appliedMove_.forward = smoothSpeed(appliedMove_.forward, requested.forward,
                                           config_.forwardAcceleration, config_.forwardDeceleration);
        appliedMove_.left = smoothSpeed(appliedMove_.left, requested.left,
                                        config_.leftAcceleration, config_.leftDeceleration);
        appliedMove_.turn = smoothSpeed(appliedMove_.turn, requested.turn,
                                        config_.turnAcceleration, config_.turnDeceleration);
        appliedMove_.walk_slot = requested.walk_slot;
    }

    bool hasWalkModel() const {
        return !walkReady_.empty() && walkReady_[0];
    }

    bool isTurnOnly(const motion::msg::MoveRequest& move) const {
        return abs(move.forward) < 0.01 && abs(move.left) < 0.01 && abs(move.turn) >= 0.01;
    }

    PolicyContext* selectWalkPolicy(const motion::msg::MoveRequest& move, int& selectedSlot) {
        const int requestedSlot = move.walk_slot;
        if(requestedSlot == kDefaultWalkSlot && isTurnOnly(move) && turnReady_) {
            selectedSlot = kTurnWalkSlot;
            return &turn_;
        }

        selectedSlot = requestedSlot == kDefaultWalkSlot ? 0 : requestedSlot;
        if(selectedSlot < 0 || selectedSlot >= static_cast<int>(walkPolicies_.size())
           || !walkReady_[selectedSlot]) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                  "walk slot %d is unavailable", selectedSlot);
            return 0;
        }
        return &walkPolicies_[selectedSlot];
    }

    Eigen::VectorXd sharedFrame(const Eigen::VectorXd& lastAction) const {
        Eigen::VectorXd frame(66);
        const Eigen::Vector3d gravity = projectedGravity(lowState_.imu_state);
        const Eigen::VectorXd defaults = defaultPose();
        for(int i = 0; i < 3; ++i) {
            frame[i] = lowState_.imu_state.gyro[i];
            frame[3 + i] = gravity[i];
        }
        for(int lab = 0; lab < kPolicyJointCount; ++lab) {
            const int runtime = kRuntimeToLab[lab];
            const auto& joint = lowState_.motor_state_serial[2 + runtime];
            frame[6 + lab] = joint.q - defaults[2 + runtime];
            frame[26 + lab] = joint.dq * 0.1;
            frame[46 + lab] = lastAction[lab];
        }
        return frame;
    }

    Eigen::VectorXd runWalk(const Eigen::VectorXd& shared, PolicyContext& policy) {
        Eigen::VectorXd frame(69);
        // exported_policy_k1 的每一帧顺序固定为：角速度、重力、速度命令、关节状态、上次动作。
        // 速度命令必须放在第 6..8 位，不能简单追加到 66 维共享观测末尾。
        frame.segment(0, 6) = shared.segment(0, 6);
        const motion::msg::MoveRequest& move = appliedMove_;
        frame[6] = move.forward;
        frame[7] = move.left;
        frame[8] = move.turn;
        frame.segment(9, 60) = shared.segment(6, 60);
        frame = policy.history.push(frame).cwiseMax(-100).cwiseMin(100);
        policy.lastAction = policy.policy.run(frame).cwiseMax(-100).cwiseMin(100);
        return policy.lastAction;
    }

    Eigen::VectorXd runKick(const Eigen::VectorXd& shared, PolicyContext& policy) {
        Eigen::VectorXd frame(71);
        frame.segment(0, 66) = shared;
        const double strength = max(0.0, brain_.kick.power);
        const double distance = hypot(brain_.kick.x, brain_.kick.y);
        const double ballScale = 2.0 / max(distance, 2.0);
        frame[66] = cos(brain_.kick.direction) * strength;
        frame[67] = sin(brain_.kick.direction) * strength;
        frame[68] = brain_.kick.x * ballScale;
        frame[69] = brain_.kick.y * ballScale;
        // 当前 Kick138 的实机基准没有接入遮挡语义，固定认为球可见。
        frame[70] = 1.0;
        frame = policy.history.push(frame).cwiseMax(-100).cwiseMin(100);
        policy.lastAction = policy.policy.run(frame).cwiseMax(-10).cwiseMin(10);
        return policy.lastAction;
    }

    Eigen::VectorXd targetFor(const Eigen::VectorXd& action) const {
        Eigen::VectorXd target = defaultPose();
        if(!runtime_.manualControl) {
            target[0] = brain_.head.yaw;
            target[1] = brain_.head.pitch;
        }
        for(int lab = 0; lab < kPolicyJointCount; ++lab) {
            const int runtime = kRuntimeToLab[lab];
            target[2 + runtime] += action[lab] * 0.25;
        }
        return target;
    }

    static bool isAnkleJoint(int joint) {
        return joint == 14 || joint == 15 || joint == 20 || joint == 21;
    }

    static GainProfile prepareGains() {
        return {20, 2, 200, 5, 100, 3};
    }

    static GainProfile runningGains() {
        return {20, 2, 100, 2, 50, 1};
    }

    static JointGains jointGains(int joint, const GainProfile& gains) {
        if(joint < 10) {
            return {gains.upperBodyKp, gains.upperBodyKd};
        }
        if(isAnkleJoint(joint)) return {gains.ankleKp, gains.ankleKd};
        return {gains.legKp, gains.legKd};
    }

    void publishTarget(const Eigen::VectorXd& target, const GainProfile& gains) {
        booster_interface::msg::LowCmd command;
        command.cmd_type = booster_interface::msg::LowCmd::CMD_TYPE_SERIAL;
        command.motor_cmd.resize(kJointCount);
        for(int i = 0; i < kJointCount; ++i) {
            auto& motor = command.motor_cmd[i];
            motor.q = target[i];
            motor.dq = 0;
            motor.tau = 0;
            const JointGains joint = jointGains(i, gains);
            motor.kp = joint.kp;
            motor.kd = joint.kd;
            motor.weight = 1;
        }
        jointPublisher_->publish(command);
    }

    string requestId() {
        ostringstream id;
        id << "00000000-0000-4000-8000-" << setw(12) << setfill('0') << ++runtime_.requestSequence;
        return id.str();
    }

    void changeMode(booster::robot::RobotMode mode) {
        const auto request = booster_interface::CreateChangeModeMsg(mode);
        booster_msgs::msg::RpcReqMsg message;
        message.uuid = requestId();
        message.header = "{\"api_id\":" + to_string(request.api_id) + "}";
        message.body = request.body;
        modePublisher_->publish(message);
    }

    void enterCustom() {
        if(runtime_.mode != Idle) {
            RCLCPP_WARN(get_logger(), "A ignored: Custom is already active");
            return;
        }
        if(!config_.enabled) {
            RCLCPP_WARN(get_logger(), "A ignored: Motion.enabled is false");
            return;
        }
        if(!runtime_.hasLowState) {
            RCLCPP_WARN(get_logger(), "A ignored: no LowState received");
            return;
        }
        publishTarget(defaultPose(), prepareGains());
        changeMode(booster::robot::RobotMode::kCustom);
        runtime_.mode = CustomReady;
        RCLCPP_INFO(get_logger(), "entered Custom; press B for Brain control or X for controller control");
    }

    void selectWalkControl(bool manualControl) {
        if(runtime_.mode == Idle) {
            RCLCPP_WARN(get_logger(), "control switch ignored: press A to enter Custom first");
            return;
        }
        if(runtime_.mode == Running && runtime_.manualControl == manualControl) return;
        for(size_t i = 0; i < walkPolicies_.size(); ++i) walkPolicies_[i].reset();
        turn_.reset();
        target_ = defaultPose();
        runtime_.controlSteps = 0;
        runtime_.manualControl = manualControl;
        runtime_.activeKickSlot = -1;
        runtime_.activeWalkSlot = -3;
        runtime_.mode = Running;
        RCLCPP_INFO(get_logger(), manualControl ? "walk control switched to controller" : "walk control switched to Brain");
    }

    void stop() {
        if(runtime_.mode == Idle) return;
        changeMode(booster::robot::RobotMode::kDamping);
        runtime_.mode = Idle;
        runtime_.manualControl = false;
        RCLCPP_WARN(get_logger(), "Custom controller stopped; robot switched to Damping");
    }

    void exitToWalking() {
        if(runtime_.mode == Idle) {
            RCLCPP_WARN(get_logger(), "Start ignored: Custom controller is not active");
            return;
        }
        changeMode(booster::robot::RobotMode::kWalking);
        runtime_.mode = Idle;
        runtime_.manualControl = false;
        runtime_.activeKickSlot = -1;
        RCLCPP_INFO(get_logger(), "Custom controller stopped; robot switched to Walking");
    }

    void updateTarget() {
        if(!runtime_.manualControl && brain_.kick.active && brain_.kick.model_slot < kickPolicies_.size()
           && kickReady_[brain_.kick.model_slot]) {
            const int slot = brain_.kick.model_slot;
            if(runtime_.activeKickSlot != slot) {
                kickPolicies_[slot].reset();
                runtime_.activeKickSlot = slot;
            }
            runtime_.activeWalkSlot = -3;
            target_ = targetFor(runKick(sharedFrame(kickPolicies_[slot].lastAction), kickPolicies_[slot]));
            return;
        }
        runtime_.activeKickSlot = -1;
        int selectedSlot = 0;
        PolicyContext* policy = selectWalkPolicy(appliedMove_, selectedSlot);
        if(!policy) {
            target_ = defaultPose();
            return;
        }
        if(runtime_.activeWalkSlot != selectedSlot) {
            policy->reset();
            runtime_.activeWalkSlot = selectedSlot;
        }
        target_ = targetFor(runWalk(sharedFrame(policy->lastAction), *policy));
    }

    void tick() {
        if(pending_.damping) {
            pending_.damping = false;
            stop();
        }
        if(pending_.exitToWalking) {
            pending_.exitToWalking = false;
            exitToWalking();
        }
        if(pending_.enterCustom) {
            pending_.enterCustom = false;
            enterCustom();
        }
        if(pending_.selectBrain) {
            pending_.selectBrain = false;
            selectWalkControl(false);
        }
        if(pending_.selectManual) {
            pending_.selectManual = false;
            selectWalkControl(true);
        }
        if(runtime_.mode != Running || !hasWalkModel() || !runtime_.hasLowState) return;
        if(lowState_.motor_state_serial.size() != kJointCount) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                  "serial low state needs %d joints, got %zu", kJointCount,
                                  lowState_.motor_state_serial.size());
            return;
        }
        if(runtime_.controlSteps++ % kPolicyControlSteps == 0) {
            updateAppliedMove();
            updateTarget();
        }
        publishTarget(target_, runningGains());
    }

    vector<bool> kickReady_;
    booster_interface::msg::LowState lowState_;
    vector<bool> walkReady_;
    vector<PolicyContext> walkPolicies_;
    bool turnReady_ = false;
    PolicyContext turn_;
    vector<PolicyContext> kickPolicies_;
    Eigen::VectorXd target_;
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSub_;
    rclcpp::Subscription<motion::msg::MoveRequest>::SharedPtr moveSub_;
    rclcpp::Subscription<motion::msg::HeadRequest>::SharedPtr headSub_;
    rclcpp::Subscription<motion::msg::KickRequest>::SharedPtr kickSub_;
    rclcpp::Subscription<booster_interface::msg::RemoteControllerState>::SharedPtr controllerSub_;
    rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr jointPublisher_;
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr modePublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<MotionNode>());
    rclcpp::shutdown();
    return 0;
}
