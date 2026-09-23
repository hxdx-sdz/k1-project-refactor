#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace brain {

using namespace std;

// 数据名称描述“需要什么、提供什么”，用于启动时排序。
// 实际输入输出仍使用具体类型，不通过字符串查找或转换数据。
struct ModuleInfo {
    string name;
    vector<string> inputs;
    vector<string> outputs;
};

// Module 是围绕一个明确职责组织的计算单元：读取声明的输入，产生声明的输出。
// 它不是 ROS 节点，也不是单独的线程；由 Brain 的 tick 经调度器统一驱动。
//
// 一个具体模块应把以下内容放在一起：
// 输入：构造时绑定需要的数据，只读；时间也是显式输入。
// 配置：模块内定义 Config 和默认值，configure 启动时读取，计算接口显式接收 Config。
// 输出：由本模块保存和更新，下游只读；私有历史与对外输出分开。
// info：描述数据依赖，用于启动排序，必须与实际绑定保持一致。
// run：完成本帧计算并保存输出；具体算法用明确参数的 update 接口表达。
//
// 计算不向 Brain、端口或其他模块索取数据，也不负责日志发送和动作下发。
// 小模块不必另拆“算法类”和“适配类”；无状态算法可用 static update，
// 有历史的算法用普通成员函数。抽象只用于简化表达，不增加无用层次。
class Module {
public:
    virtual ~Module() {}
    virtual ModuleInfo info() const = 0;
    virtual void configure(rclcpp::Node&) {}
    virtual void run() = 0;
};

}
