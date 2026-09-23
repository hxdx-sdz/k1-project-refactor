#pragma once

#include "common/input_types.hpp"
#include "common/module.h"
#include <sstream>

namespace brain {

using namespace std;

struct ObservedObject {
    string label, color;
    array<double, 4> box;
    vector<float> pixel;
    Pose2D relative;
    double confidence, range, yaw, pitch;
};
struct FieldMarker { char type; double x, y, confidence; };
struct ObservedLine {
    array<double, 4> relative, pixel;
    bool hasPixel = false;
};
struct Observations {
    vector<ObservedObject> balls, robots, goalposts, markings;
    vector<FieldMarker> markers;
    vector<ObservedLine> lines;
    uint64_t detectionRevision = 0, lineRevision = 0;
    double receivedAt = 0, sourceTime = 0;
    double linesReceivedAt = 0, linesSourceTime = 0;

    string detail() const {
        ostringstream out;
        out << "markers=" << markers.size() << " balls=" << balls.size() << "\n"
            << "robots=" << robots.size() << " lines=" << lines.size() << "\n"
            << "detection_rev=" << detectionRevision << "\n"
            << "line_rev=" << lineRevision;
        return out.str();
    }
};

// 只整理本帧观测，不提前用未知的场地位姿给物体命名或设置场地坐标
class ObservationModule : public Module {
public:
    struct Config { double robotHeight = 0.9; }; // 米，用于 pitch 近似

    ObservationModule(const Sample<VisionState>& vision, const Sample<FieldLines>& lines)
        : vision_(vision), lines_(lines) {}
    Observations output;
    ModuleInfo info() const override {
        return {"Observations", {"VisionState", "FieldLines"}, {"Observations"}};
    }
    void configure(rclcpp::Node& node) override {
        config_.robotHeight = node.declare_parameter<double>("Observations.robot_height", config_.robotHeight);
    }
    void run() override {
        if(output.detectionRevision != vision_.revision || output.lineRevision != lines_.revision)
            output = update(vision_, lines_, config_);
    }
    static Observations update(const Sample<VisionState>& vision, const Sample<FieldLines>& lines,
                               const Config& config) {
        Observations result;
        result.detectionRevision = vision.revision;
        result.lineRevision = lines.revision;
        result.receivedAt = vision.updatedAt;
        result.sourceTime = vision.value.sourceTime;
        result.linesReceivedAt = lines.updatedAt;
        result.linesSourceTime = lines.value.sourceTime;
        if(vision.valid()) for(const Detection& detection : vision.value.detections) {
            // ROS 数组是变长的，缺坐标或非有限数的观测不能送进定位器。
            if(detection.projection.size() < 2) continue;
            const double x = detection.projection[0], y = detection.projection[1];
            if(!isfinite(x) || !isfinite(y) || !isfinite(detection.confidence)) continue;
            ObservedObject object;
            object.label = detection.label; object.color = detection.color;
            object.box = detection.box; object.pixel = detection.pixel;
            object.relative = {x, y, 0}; object.confidence = detection.confidence;
            object.range = hypot(x, y); object.yaw = atan2(y, x);
            object.pitch = atan2(config.robotHeight, object.range);
            if(object.label == "Ball") result.balls.push_back(object);
            else if(object.label == "Opponent") result.robots.push_back(object);
            else if(object.label == "Goalpost" && object.confidence >= 50)
                result.goalposts.push_back(object);
            char type = 0;
            if(object.label == "LCross") type = 'L';
            else if(object.label == "TCross") type = 'T';
            else if(object.label == "XCross") type = 'X';
            else if(object.label == "PenaltyPoint") type = 'P';
            if(type && object.confidence >= 50 && x >= -0.5 && x <= 15) {
                result.markings.push_back(object);
                result.markers.push_back({type, x, y, object.confidence});
            }
        }
        if(lines.valid()) for(size_t i = 0; i + 3 < lines.value.coordinates.size(); i += 4) {
            ObservedLine line;
            bool valid = true;
            line.hasPixel = i + 3 < lines.value.pixels.size();
            for(size_t j = 0; j < 4; ++j) {
                line.relative[j] = lines.value.coordinates[i + j];
                line.pixel[j] = line.hasPixel ? lines.value.pixels[i + j] : 0;
                valid = valid && isfinite(line.relative[j]);
            }
            if(valid) result.lines.push_back(line);
        }
        return result;
    }
private:
    const Sample<VisionState>& vision_;
    const Sample<FieldLines>& lines_;
    Config config_;
};

}
