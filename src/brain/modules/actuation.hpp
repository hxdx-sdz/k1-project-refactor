#pragma once

#include "modules/skill.hpp"
#include "common/module.h"
#include <motion/msg/head_request.hpp>
#include <motion/msg/kick_request.hpp>
#include <motion/msg/move_request.hpp>
#include <sstream>

namespace brain {

using namespace std;

struct ActuationState {
    bool kickActive = false;
    uint64_t requests = 0;

    string detail() const {
        ostringstream out;
        out << "kick_active=" << kickActive << "\nrequests=" << requests;
        return out.str();
    }
};

// Actuation 是 Brain 和运控包之间唯一的边界。
// 它不再调用高层 Robocup RPC，也不管理动作模式；Skill 的三个请求原样交给 motion。
class ActuationModule : public Module {
public:
    ActuationModule(const HeadRequest& head, const MotionRequest& motion, const KickRequest& kick)
        : head_(head), motion_(motion), kick_(kick) {}

    ActuationState output;

    ModuleInfo info() const override {
        return {"Actuation", {"HeadRequest", "MotionRequest", "KickRequest"}, {"ActuationState"}};
    }

    void configure(rclcpp::Node& node) override {
        headPublisher_ = node.create_publisher<motion::msg::HeadRequest>("/motion/head_request", 1);
        movePublisher_ = node.create_publisher<motion::msg::MoveRequest>("/motion/move_request", 1);
        kickPublisher_ = node.create_publisher<motion::msg::KickRequest>("/motion/kick_request", 1);
    }

    void run() override {
        motion::msg::HeadRequest head;
        head.pitch = head_.pitch;
        head.yaw = head_.yaw;
        headPublisher_->publish(head);

        motion::msg::MoveRequest move;
        move.walk_slot = motion_.walkSlot;
        move.forward = motion_.forward;
        move.left = motion_.left;
        move.turn = motion_.turn;
        movePublisher_->publish(move);

        motion::msg::KickRequest kick;
        kick.active = kick_.active;
        kick.model_slot = kick_.modelSlot;
        kick.x = kick_.x;
        kick.y = kick_.y;
        kick.direction = kick_.direction;
        kick.power = kick_.power;
        kickPublisher_->publish(kick);

        output.kickActive = kick_.active;
        ++output.requests;
    }

private:
    const HeadRequest& head_;
    const MotionRequest& motion_;
    const KickRequest& kick_;
    rclcpp::Publisher<motion::msg::HeadRequest>::SharedPtr headPublisher_;
    rclcpp::Publisher<motion::msg::MoveRequest>::SharedPtr movePublisher_;
    rclcpp::Publisher<motion::msg::KickRequest>::SharedPtr kickPublisher_;
};

}
