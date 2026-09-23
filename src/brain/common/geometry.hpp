#pragma once

#include <cmath>

namespace brain {

using namespace std;

// 平面坐标统一使用米、弧度，x 向前、y 向左。名称注明参考坐标系。
struct Pose2D {
    double x, y, yaw;
    Pose2D(double x = 0, double y = 0, double yaw = 0) : x(x), y(y), yaw(yaw) {}
};
inline double wrapAngle(double angle) { return atan2(sin(angle), cos(angle)); }
inline Pose2D compose(const Pose2D& base, const Pose2D& local) {
    return {base.x + cos(base.yaw) * local.x - sin(base.yaw) * local.y,
            base.y + sin(base.yaw) * local.x + cos(base.yaw) * local.y,
            wrapAngle(base.yaw + local.yaw)};
}
inline Pose2D inverse(const Pose2D& pose) {
    return {-cos(pose.yaw) * pose.x - sin(pose.yaw) * pose.y,
             sin(pose.yaw) * pose.x - cos(pose.yaw) * pose.y, -pose.yaw};
}

}
