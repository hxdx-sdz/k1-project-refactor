#pragma once

#include "common/module.h"
#include <iomanip>
#include <sstream>
#include <vector>

namespace brain {

using namespace std;

struct FieldDimensions {
    double length, width, penaltyDist, goalWidth, circleRadius;
    double penaltyAreaLength, penaltyAreaWidth, goalAreaLength, goalAreaWidth;
};

struct FieldLine {
    double x0, y0, x1, y1;
};

struct FieldModel {
    // 临时 14x11 场地；其他尺寸沿用原 Demo adult_size 的定义量级。
    FieldDimensions dimensions = {14, 11, 2.242, 2.6, 1.54, 3.24, 6.192, 1.345, 4};
    vector<FieldLine> lines;

    string detail() const {
        ostringstream out;
        out << fixed << setprecision(1) << "size=" << dimensions.length << 'x'
            << dimensions.width << "m\nlines=" << lines.size();
        return out.str();
    }
};

// FieldProvider 保存比赛场地的静态模型。它不是定位算法的一部分，
// 只是把“这场比赛在哪块场地上踢”变成一份明确的数据输出。
class FieldProviderModule : public Module {
public:
    FieldModel output;

    ModuleInfo info() const override {
        return {"FieldProvider", {}, {"FieldModel"}};
    }

    void configure(rclcpp::Node& node) override {
        FieldDimensions& f = output.dimensions;
        f.length = node.declare_parameter<double>("FieldProvider.field.length", f.length);
        f.width = node.declare_parameter<double>("FieldProvider.field.width", f.width);
        f.penaltyDist = node.declare_parameter<double>("FieldProvider.field.penalty_distance", f.penaltyDist);
        f.goalWidth = node.declare_parameter<double>("FieldProvider.field.goal_width", f.goalWidth);
        f.circleRadius = node.declare_parameter<double>("FieldProvider.field.circle_radius", f.circleRadius);
        f.penaltyAreaLength = node.declare_parameter<double>("FieldProvider.field.penalty_area_length", f.penaltyAreaLength);
        f.penaltyAreaWidth = node.declare_parameter<double>("FieldProvider.field.penalty_area_width", f.penaltyAreaWidth);
        f.goalAreaLength = node.declare_parameter<double>("FieldProvider.field.goal_area_length", f.goalAreaLength);
        f.goalAreaWidth = node.declare_parameter<double>("FieldProvider.field.goal_area_width", f.goalAreaWidth);

        makeLines();
    }

    void run() override {}

private:
    void makeLines() {
        const FieldDimensions& f = output.dimensions;
        output.lines.clear();
        addLine( f.length / 2, -f.width / 2,  f.length / 2,  f.width / 2);
        addLine(-f.length / 2, -f.width / 2, -f.length / 2,  f.width / 2);
        addLine(-f.length / 2,  f.width / 2,  f.length / 2,  f.width / 2);
        addLine(-f.length / 2, -f.width / 2,  f.length / 2, -f.width / 2);
        addLine(0, -f.width / 2, 0, f.width / 2);

        addBox( f.length / 2, -1, f.goalAreaLength, f.goalAreaWidth);
        addBox( f.length / 2, -1, f.penaltyAreaLength, f.penaltyAreaWidth);
        addBox(-f.length / 2,  1, f.goalAreaLength, f.goalAreaWidth);
        addBox(-f.length / 2,  1, f.penaltyAreaLength, f.penaltyAreaWidth);
    }

    void addLine(double x0, double y0, double x1, double y1) {
        output.lines.push_back({x0, y0, x1, y1});
    }

    void addBox(double goalX, int dir, double length, double width) {
        const double innerX = goalX + dir * length;
        addLine(goalX, -width / 2, innerX, -width / 2);
        addLine(goalX,  width / 2, innerX,  width / 2);
        addLine(innerX, -width / 2, innerX,  width / 2);
    }
};

}
