#pragma once

#include "common/module.h"
#include <iomanip>
#include <sstream>
#include <stdexcept>
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
    FieldDimensions dimensions = {9, 6, 1.5, 2.6, 0.75, 2, 4, 1, 3};
    vector<FieldLine> lines;

    string detail() const {
        ostringstream out;
        out << fixed << setprecision(1) << "size=" << dimensions.length << 'x'
            << dimensions.width << "m";
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
        string size = node.declare_parameter<string>("FieldProvider.field_size", "kid_size");
        output.dimensions = dimensionsFor(size);
        makeLines();
    }

    void run() override {}

private:
    FieldDimensions dimensionsFor(const string& size) {
        if(size == "kid_size")
            return {9, 6, 1.5, 2.6, 0.75, 2, 4, 1, 3};
        if(size == "adult_size")
            return {14, 9, 2, 2.6, 1.5, 3, 6, 1, 4};
        if(size == "robo_league")
            return {22, 14, 2.5, 2.6, 2, 3.5, 7, 1, 5};
        throw invalid_argument("FieldProvider.field_size must be kid_size, adult_size, or robo_league");
    }

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
