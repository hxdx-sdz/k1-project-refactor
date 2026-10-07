#include "motion/policy.hpp"
#include "motion/odom.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <booster_interface/message_utils.hpp>
#include <booster_interface/msg/low_cmd.hpp>
#include <booster_interface/msg/low_state.hpp>
#include <booster_interface/msg/odometer.hpp>
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
const double kPi = 3.14159265358979323846;

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

// 每秒汇总一次，保留平均值和最坏值，避免高频日志影响控制时序。
struct TimingSample {
    unsigned count = 0;
    double total = 0;
    double maximum = 0;

    void add(double value) {
        ++count;
        total += value;
        maximum = max(maximum, value);
    }

    double average() const { return count ? total / count : 0; }
};

struct MotionTiming {
    typedef chrono::steady_clock Clock;
    typedef Clock::time_point Time;

    TimingSample tickGap, tickWork, inference;
    Time lastTick;
    Time lastReport = Clock::now();
    unsigned lateTicks = 0;
    unsigned slowTicks = 0;

    static double elapsed(const Time& start, const Time& end) {
        return chrono::duration<double, micro>(end - start).count();
    }

    static void gap(TimingSample& sample, Time& previous, const Time& now) {
        if(previous != Time()) sample.add(elapsed(previous, now));
        previous = now;
    }

    void clearSamples() {
        tickGap = tickWork = inference = TimingSample();
        lateTicks = slowTicks = 0;
    }
};

}

class MotionNode : public rclcpp::Node {
public:
    MotionNode()
        : Node("motion_node"), target_(defaultPose()) {
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
        config_.kickBallOffset = declare_parameter<vector<double> >("Motion.kick_ball_offset", {0.0, 0.0});
        if(config_.kickBallOffset.size() != 2)
            throw invalid_argument("Motion.kick_ball_offset needs [x, y]");
        config_.odomPaths = declare_parameter<vector<string> >(
            "Motion.odom_models", {"K1_odom_model_20260128_152713", "K1_odom_model_visionkick"});
        config_.walkPaths = declare_parameter<vector<string> >("Motion.walk_models", vector<string>());
        config_.turnPath = declare_parameter<string>("Motion.turn_model", "");
        config_.kickPaths = declare_parameter<vector<string> >("Motion.kick_models", vector<string>());

        for(size_t i = 0; i < config_.odomPaths.size(); ++i)
            odomModels_.push_back(motion::OdomModel(motion::k1Odom20260128Calibration()));
        walkPolicies_.resize(config_.walkPaths.size());
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
        odomPublisher_ = create_publisher<booster_interface::msg::Odometer>("/motion/odometer", 1);
        modePublisher_ = create_publisher<booster_msgs::msg::RpcReqMsg>("LocoApiTopicReq", 1);
        timer_ = create_wall_timer(chrono::milliseconds(kControlIntervalMs), [this]() {
            const MotionTiming::Time start = MotionTiming::Clock::now();
            if(timing_.lastTick != MotionTiming::Time()
               && MotionTiming::elapsed(timing_.lastTick, start) > 3000) ++timing_.lateTicks;
            MotionTiming::gap(timing_.tickGap, timing_.lastTick, start);
            tick();
            const double work = MotionTiming::elapsed(start, MotionTiming::Clock::now());
            timing_.tickWork.add(work);
            if(work > 2000) ++timing_.slowTicks;
            reportTiming();
        });
    }

private:
    enum ControllerMode { Idle, CustomReady, Running };

    MotionTiming timing_;

    void reportTiming() {
        const MotionTiming::Time now = MotionTiming::Clock::now();
        const double seconds = MotionTiming::elapsed(timing_.lastReport, now) / 1000000;
        if(seconds < 1) return;
        if(runtime_.mode == Running)
            RCLCPP_INFO(get_logger(),
                "timing: tick=%.2f/%.2fms late(>3ms)=%u work=%.2f/%.2fms slow(>2ms)=%u infer=%.2f/%.2fms",
                timing_.tickGap.average() / 1000, timing_.tickGap.maximum / 1000,
                timing_.lateTicks, timing_.tickWork.average() / 1000,
                timing_.tickWork.maximum / 1000, timing_.slowTicks,
                timing_.inference.average() / 1000, timing_.inference.maximum / 1000);
        timing_.lastReport = now;
        timing_.clearSamples();
    }

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
        vector<double> kickBallOffset;
        vector<string> odomPaths;
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
        int activeKickSlot = -1;
        unsigned controlSteps = 0;
        uint64_t requestSequence = 0;
    } runtime_;

    // 这是模型实际看到的速度命令。它只负责让输入连续，不承担策略判断。
    motion::msg::MoveRequest appliedMove_;

    // 所有 walk 模型看到同一段运动过程；切换模型不重置历史或上一帧动作。
    struct WalkObservation {
        motion::History history{69};
        Eigen::VectorXd lastAction = Eigen::VectorXd::Zero(kPolicyJointCount);

        void reset() {
            history.clear();
            lastAction.setZero();
        }
    } walkObservation_;

    struct OdomState {
        motion::OdomObservation observation;
        Eigen::Vector2d position = Eigen::Vector2d::Zero();
    } odomState_;


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
        int odomHistorySize = 0;
        for(size_t i = 0; i < odomModels_.size(); ++i) {
            const string path = modelPath(config_.odomPaths[i]);
            odomModels_[i].load(path);
            odomHistorySize = max(odomHistorySize, odomModels_[i].historySize());
            RCLCPP_INFO(get_logger(), "loaded odom slot %zu: %s", i, path.c_str());
        }
        odomState_.observation.init(odomHistorySize);
        walkReady_.assign(walkPolicies_.size(), false);
        for(size_t i = 0; i < walkPolicies_.size(); ++i) {
            try {
                const string path = modelPath(config_.walkPaths[i]);
                walkPolicies_[i].load(path);
                if(walkPolicies_[i].inputSize() != 69 * kHistorySize
                   || walkPolicies_[i].outputSize() != kPolicyJointCount)
                    throw runtime_error("walk policy must be 690 -> 20");
                walkReady_[i] = true;
                RCLCPP_INFO(get_logger(), "loaded walk slot %zu: %s", i, path.c_str());
            } catch(const exception& error) {
                RCLCPP_WARN(get_logger(), "walk slot %zu is unavailable: %s", i, error.what());
            }
        }

        try {
            const string path = modelPath(config_.turnPath);
            turn_.load(path);
            if(turn_.inputSize() != 69 * kHistorySize || turn_.outputSize() != kPolicyJointCount)
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

    static double wrapAngle(double angle) {
        while(angle > kPi) angle -= 2 * kPi;
        while(angle < -kPi) angle += 2 * kPi;
        return angle;
    }

    Eigen::VectorXd odomFrame() const {
        Eigen::VectorXd frame = Eigen::VectorXd::Zero(39);
        for(int i = 0; i < 3; ++i) {
            frame[9 + i] = lowState_.imu_state.gyro[i];
            frame[12 + i] = lowState_.imu_state.acc[i];
        }
        for(int i = 0; i < 12; ++i) {
            frame[15 + i] = lowState_.motor_state_serial[10 + i].q;
            frame[27 + i] = lowState_.motor_state_serial[10 + i].dq;
        }
        return frame;
    }

    void updateOdom() {
        const Eigen::Quaterniond orientation =
            Eigen::AngleAxisd(lowState_.imu_state.rpy[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(lowState_.imu_state.rpy[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(lowState_.imu_state.rpy[0], Eigen::Vector3d::UnitX());
        const Eigen::VectorXd window = odomState_.observation.push(odomFrame(), orientation);
        // VisualKick 与普通走路的接触节奏不同，里程计模型由 Motion 按当前控制模式选择。
        const unsigned slot = !runtime_.manualControl && brain_.kick.active ? 1 : 0;
        const Eigen::Vector2d delta = odomModels_[slot].infer(window);
        const double yaw = odomState_.observation.firstYaw();
        const double c = cos(yaw), s = sin(yaw);
        odomState_.position.x() += c * delta.x() - s * delta.y();
        odomState_.position.y() += s * delta.x() + c * delta.y();
        booster_interface::msg::Odometer message;
        message.x = odomState_.position.x();
        message.y = odomState_.position.y();
        message.theta = odomState_.observation.currentYaw();
        odomPublisher_->publish(message);
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

    motion::Policy* selectWalkPolicy(const motion::msg::MoveRequest& move) {
        const int requestedSlot = move.walk_slot;
        if(requestedSlot == kDefaultWalkSlot && isTurnOnly(move) && turnReady_) {
            return &turn_;
        }

        const int selectedSlot = requestedSlot == kDefaultWalkSlot ? 0 : requestedSlot;
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

    Eigen::VectorXd runWalk(motion::Policy& policy) {
        const Eigen::VectorXd shared = sharedFrame(walkObservation_.lastAction);
        Eigen::VectorXd frame(69);
        // exported_policy_k1 的每一帧顺序固定为：角速度、重力、速度命令、关节状态、上次动作。
        // 速度命令必须放在第 6..8 位，不能简单追加到 66 维共享观测末尾。
        frame.segment(0, 6) = shared.segment(0, 6);
        const motion::msg::MoveRequest& move = appliedMove_;
        frame[6] = move.forward;
        frame[7] = move.left;
        frame[8] = move.turn;
        frame.segment(9, 60) = shared.segment(6, 60);
        frame = walkObservation_.history.push(frame).cwiseMax(-100).cwiseMin(100);
        const MotionTiming::Time start = MotionTiming::Clock::now();
        walkObservation_.lastAction = policy.run(frame).cwiseMax(-100).cwiseMin(100);
        timing_.inference.add(MotionTiming::elapsed(start, MotionTiming::Clock::now()));
        return walkObservation_.lastAction;
    }

    Eigen::VectorXd runKick(const Eigen::VectorXd& shared, PolicyContext& policy) {
        Eigen::VectorXd frame(71);
        frame.segment(0, 66) = shared;
        const double strength = max(0.0, brain_.kick.power);
        // 偏移只校准踢球模型看到的球位，不回写感知数据；在归一化前加上。
        const double ballX = brain_.kick.x + config_.kickBallOffset[0];
        const double ballY = brain_.kick.y + config_.kickBallOffset[1];
        const double distance = hypot(ballX, ballY);
        const double ballScale = 2.0 / max(distance, 2.0);
        frame[66] = cos(brain_.kick.direction) * strength;
        frame[67] = sin(brain_.kick.direction) * strength;
        frame[68] = ballX * ballScale;
        frame[69] = ballY * ballScale;
        // 当前 Kick138 的实机基准没有接入遮挡语义，固定认为球可见。
        frame[70] = 1.0;
        frame = policy.history.push(frame).cwiseMax(-100).cwiseMin(100);
        const MotionTiming::Time start = MotionTiming::Clock::now();
        policy.lastAction = policy.policy.run(frame).cwiseMax(-10).cwiseMin(10);
        timing_.inference.add(MotionTiming::elapsed(start, MotionTiming::Clock::now()));
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
        walkObservation_.reset();
        target_ = defaultPose();
        runtime_.controlSteps = 0;
        runtime_.manualControl = manualControl;
        runtime_.activeKickSlot = -1;
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
            target_ = targetFor(runKick(sharedFrame(kickPolicies_[slot].lastAction), kickPolicies_[slot]));
            return;
        }
        // 踢球与 walk 的观测不同，返回 walk 时重新开始；walk 模型之间直接续用。
        if(runtime_.activeKickSlot >= 0) walkObservation_.reset();
        runtime_.activeKickSlot = -1;
        motion::Policy* policy = selectWalkPolicy(appliedMove_);
        if(!policy) {
            target_ = defaultPose();
            return;
        }
        target_ = targetFor(runWalk(*policy));
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
        if(!runtime_.hasLowState) return;
        if(lowState_.motor_state_serial.size() != kJointCount) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                  "serial low state needs %d joints, got %zu", kJointCount,
                                  lowState_.motor_state_serial.size());
            return;
        }
        if(runtime_.controlSteps++ % kPolicyControlSteps == 0) {
            updateOdom();
        }
        if(runtime_.mode != Running || !hasWalkModel()) return;
        if((runtime_.controlSteps - 1) % kPolicyControlSteps == 0) {
            updateAppliedMove();
            updateTarget();
        }
        publishTarget(target_, runningGains());
    }

    vector<bool> kickReady_;
    booster_interface::msg::LowState lowState_;
    vector<bool> walkReady_;
    vector<motion::Policy> walkPolicies_;
    bool turnReady_ = false;
    motion::Policy turn_;
    vector<motion::OdomModel> odomModels_;
    vector<PolicyContext> kickPolicies_;
    Eigen::VectorXd target_;
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSub_;
    rclcpp::Subscription<motion::msg::MoveRequest>::SharedPtr moveSub_;
    rclcpp::Subscription<motion::msg::HeadRequest>::SharedPtr headSub_;
    rclcpp::Subscription<motion::msg::KickRequest>::SharedPtr kickSub_;
    rclcpp::Subscription<booster_interface::msg::RemoteControllerState>::SharedPtr controllerSub_;
    rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr jointPublisher_;
    rclcpp::Publisher<booster_interface::msg::Odometer>::SharedPtr odomPublisher_;
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr modePublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<MotionNode>());
    rclcpp::shutdown();
    return 0;
}
