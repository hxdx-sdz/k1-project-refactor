#pragma once

#include "common/input_port.h"
#include "common/geometry.hpp"
#include <array>
#include <string>
#include <vector>

namespace brain {

using namespace std;

// 系统入口共用的数据；各模块的输出类型留在对应模块文件中。
struct Odometry {
    double x = 0, y = 0, yaw = 0;
};
// 一项电机反馈保留位置、运动状态及诊断信息，具体用途由下游算法决定。
struct JointState {
    int8_t mode = 0;
    float q = 0, dq = 0, ddq = 0, tauEst = 0;
    int8_t temperature = 0;
    uint32_t lost = 0;
    array<uint32_t, 2> reserve = {{0, 0}};
};

// 保留 LowState 的两组原始顺序，不假定关节数量，也不把索引解释成关节名称。
// serial 与 parallel 是消息中的两种状态表示，不能直接拼成一组独立关节。
struct JointData {
    vector<JointState> serial;
    vector<JointState> parallel;
};
struct GameState {
    int state = 0, secondaryState = 0, team = 0;
    int ownScore = 0, opponentScore = 0, penalty = 0;
};
// ROS 入口只做字段转换；是否采用某个观测由观测模块决定。
struct Detection {
    string label, color;
    double confidence = 0;
    array<double, 4> box = {{0, 0, 0, 0}}; // xmin, ymin, xmax, ymax
    vector<float> projection, pixel;
};
struct VisionState {
    unsigned objects = 0, balls = 0, robots = 0;
    double sourceTime = 0; // 消息时间，仅用于追溯，不与接收端单调时间混算
    vector<Detection> detections;
};
struct FieldLines {
    double sourceTime = 0;
    vector<float> coordinates, pixels; // 每四个数为一条线的两个端点
};
struct HeadPose {
    array<double, 3> position = {{0, 0, 0}};
    array<double, 4> rotation = {{0, 0, 0, 1}}; // x, y, z, w
};
struct RecoveryState {
    int state = 0; // Demo 协议：ready / falling / fallen / getting-up = 0 / 1 / 2 / 3
    bool available = false;
    int planner = 0;
};

// Brain 每帧读取的外部输入，供模块按需绑定；这不是所有模块结果的总仓库。
// 各路仍是独立接收的最新值，不代表采集时间对齐。now 使用接收端单调时钟。
struct InputFrame {
    double now = 0;
    Sample<Odometry> odometry;
    Sample<JointData> joint;
    Sample<GameState> game;
    Sample<VisionState> vision;
    Sample<FieldLines> fieldLines;
    Sample<HeadPose> headPose;
    Sample<RecoveryState> recovery;
};


}
