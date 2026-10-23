#pragma once

#include "common/module.h"
#include "modules/field_provider.hpp"
#include "modules/localization.hpp"
#include "modules/ball_state.hpp"
#include "modules/field_potential.hpp"
#include "modules/strategy.hpp"
#include "modules/skill.hpp"
#include "modules/actuation.hpp"
#include <rerun.hpp>
#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace brain {

using namespace std;

// ===================== 数据流图排版（纯函数，仅本文件使用） =====================
// 图中只保留数据类型节点。节点按数据依赖分层，
// 相邻层之间画直线箭头；跨层依赖留在启动日志里，避免长线横穿主图。

struct GraphNode {
    string name;
    int column;
    double x, y;
    vector<GraphNode*> up;
    int colorIndex;
    bool external;
};
struct GraphEdge {
    GraphNode* from;
    GraphNode* to;
    int colorIndex;
};
struct GraphDescription {
    vector<string> names;
    double x, y;
};
struct GraphLayout {
    vector<GraphNode> nodes;
    vector<GraphEdge> edges;
    vector<GraphDescription> descriptions;
};

inline GraphNode* dataNode(map<string, GraphNode*>& nodes, const string& name) {
    if(!nodes.count(name)) return nullptr;
    return nodes[name];
}

inline rerun::Color graphColor(int index) {
    const unsigned int colors[] = {
        0xE69F00FF, 0x56B4E9FF, 0x009E73FF, 0xCC79A7FF, 0xD55E00FF, 0x0072B2FF
    };
    return rerun::Color(colors[index % 6]);
}

// 图只负责为每层输出预留动态详情的位置；详情文本留在数据类型自身。
// 外部输入不放详情，避免第一层重新变成说明墙。
inline GraphLayout layoutGraph(const vector<string>& externalInputs,
                               const vector<ModuleInfo>& modules,
                               double columnGap = 24, double rowGap = 6) {
    GraphLayout layout;
    size_t total = externalInputs.size();
    for(const ModuleInfo& info : modules) total += info.outputs.size();
    layout.nodes.reserve(total);

    map<string, GraphNode*> nodes;
    map<string, int> columnOf;
    for(const string& name : externalInputs) {
        layout.nodes.push_back({name, 0, 0.0, 0.0, {}, 0, true});
        nodes[name] = &layout.nodes.back();
        columnOf[name] = 0;
    }

    int nextColor = 0;
    for(const ModuleInfo& info : modules) {
        int column = 1;
        vector<GraphNode*> inputs;
        for(const string& input : info.inputs) {
            GraphNode* node = dataNode(nodes, input);
            if(node) {
                inputs.push_back(node);
                column = max(column, columnOf[input] + 1);
            }
        }

        vector<GraphNode*> outputs;
        for(const string& output : info.outputs) {
            layout.nodes.push_back({output, column, column * columnGap, 0.0,
                                    inputs, nextColor, false});
            GraphNode* node = &layout.nodes.back();
            nodes[output] = node;
            columnOf[output] = column;
            outputs.push_back(node);
        }

        for(GraphNode* output : outputs) {
            for(GraphNode* input : inputs) {
                if(output->column == input->column + 1)
                    layout.edges.push_back({input, output, nextColor});
            }
        }
        ++nextColor;
    }

    int maxColumn = 0;
    for(const GraphNode& node : layout.nodes) maxColumn = max(maxColumn, node.column);
    for(int column = 0; column <= maxColumn; ++column) {
        vector<pair<double, size_t>> order;
        for(size_t i = 0; i < layout.nodes.size(); ++i) {
            const GraphNode& node = layout.nodes[i];
            if(node.column != column) continue;
            double sum = 0;
            for(const GraphNode* upstream : node.up) sum += upstream->y;
            order.push_back({node.up.empty() ? static_cast<double>(i)
                                             : sum / node.up.size(), i});
        }
        stable_sort(order.begin(), order.end());
        for(size_t k = 0; k < order.size(); ++k)
            layout.nodes[order[k].second].y = k * rowGap - (order.size() - 1) * rowGap / 2;
    }

    // 每层仍保留自己的详情块；只把它们的基线统一移到整张图下方。
    // 这样同层信息不会被合成一块，也不会再插进依赖图内部。
    double graphBottom = 0;
    bool hasNode = false;
    for(const GraphNode& node : layout.nodes) {
        graphBottom = hasNode ? max(graphBottom, node.y) : node.y;
        hasNode = true;
    }
    for(int column = 0; column <= maxColumn; ++column) {
        vector<string> names;
        for(const GraphNode& node : layout.nodes)
            if(node.column == column && !node.external) names.push_back(node.name);
        if(!names.empty())
            layout.descriptions.push_back({names, column * columnGap,
                                           graphBottom + rowGap * 8});
    }
    return layout;
}

// ===================== Rerun 观察模块 =====================

// Rerun 是数据流末端的观察模块：当前阶段只在启动时画出静态依赖图。
// 它不向其他模块提供业务数据，也不改变任何算法状态。
class RerunModule : public Module {
public:
    struct Config {
        bool enableTcp = false;
        string serverIp = "127.0.0.1:9876";
    };

    RerunModule(const FieldModel& field, const SelfState& self, const Observations& observations,
                const LocalizationState& localization, const BallState& ball, const FieldPotential& fieldPotential,
                const Strategy& strategy,
                const HeadRequest& head, const MotionRequest& motion,
                const KickRequest& kick, const ActuationState& actuation, const double& now)
        : field_(field), self_(self), observations_(observations), localization_(localization),
          ball_(ball), fieldPotential_(fieldPotential), strategy_(strategy), head_(head), motion_(motion), kick_(kick),
          actuation_(actuation), now_(now), stream_("k1_brain") {}

    ModuleInfo info() const override {
        return {"Rerun", {"FieldModel", "SelfState", "Observations", "LocalizationState",
                            "BallState", "FieldPotential", "Strategy", "HeadRequest", "MotionRequest",
                            "KickRequest", "ActuationState", "FrameTime"}, {}};
    }

    void configure(rclcpp::Node& node) override {
        config_.enableTcp = node.declare_parameter<bool>("Rerun.enable_tcp", config_.enableTcp);
        config_.serverIp = node.declare_parameter<string>("Rerun.server_ip", config_.serverIp);
        if(config_.enableTcp) {
            rerun::Error error = stream_.connect(config_.serverIp);
            if(error.is_err()) cerr << "[rerun] connect failed: " << error.description << endl;
        }
    }

    bool enabled() const { return config_.enableTcp; }
    const string& serverIp() const { return config_.serverIp; }

    // 调度器在启动完成后给出一次依赖图。图本身延后到首帧写入，
    // 这样它与场地、定位使用同一条时间轴。
    void setSchedule(const string& schedule, const vector<ModuleInfo>& modules,
                     const vector<string>& externalInputs) {
        if(!config_.enableTcp) return;
        schedule_ = schedule;
        layout_ = layoutGraph(externalInputs, modules);
    }

    void run() override {
        if(!config_.enableTcp) return;
        stream_.set_time_seconds("time", now_);
        if(!graphLogged_) {
            logGraph();
            graphLogged_ = true;
        }
        if(!fieldLogged_) {
            logField();
            fieldLogged_ = true;
        }
        logGraphDetails();
        logLocalization();
        logFieldPotential();
    }

private:
    void logGraph() {
        stream_.log("brain/graph", rerun::TextLog(schedule_));
        vector<rerun::Position2D> nodePoints;
        vector<string> nodeLabels;
        vector<float> nodeRadii;
        vector<rerun::Color> nodeColors;
        for(const GraphNode& node : layout_.nodes) {
            nodePoints.push_back(rerun::Position2D{static_cast<float>(node.x),
                                                   static_cast<float>(node.y)});
            nodeLabels.push_back(node.name);
            nodeRadii.push_back(1.4f);
            nodeColors.push_back(node.external ? rerun::Color(0xAAAAAAFF)
                                               : graphColor(node.colorIndex));
        }
        vector<rerun::LineStrip2D> strips;
        vector<rerun::Color> stripColors;
        for(const GraphEdge& edge : layout_.edges) {
            const double dx = edge.to->x - edge.from->x;
            const double dy = edge.to->y - edge.from->y;
            const double length = sqrt(dx * dx + dy * dy);
            if(length <= 0.01) continue;
            const double ux = dx / length;
            const double uy = dy / length;
            const double startX = edge.from->x + ux * 1.4;
            const double startY = edge.from->y + uy * 1.4;
            const double endX = edge.to->x - ux * 1.4;
            const double endY = edge.to->y - uy * 1.4;
            const double backX = endX - ux * 0.8;
            const double backY = endY - uy * 0.8;
            const double sideX = -uy * 0.3;
            const double sideY = ux * 0.3;
            vector<rerun::Position2D> points = {
                {static_cast<float>(startX), static_cast<float>(startY)},
                {static_cast<float>(endX), static_cast<float>(endY)},
                {static_cast<float>(backX + sideX), static_cast<float>(backY + sideY)},
                {static_cast<float>(endX), static_cast<float>(endY)},
                {static_cast<float>(backX - sideX), static_cast<float>(backY - sideY)}
            };
            strips.push_back(rerun::LineStrip2D(points));
            stripColors.push_back(graphColor(edge.colorIndex));
        }
        if(!strips.empty())
            stream_.log("brain/graph/edges",
                        rerun::LineStrips2D(strips).with_colors(stripColors));
        if(!nodePoints.empty())
            stream_.log("brain/graph/nodes",
                        rerun::Points2D(nodePoints).with_labels(nodeLabels)
                            .with_radii(nodeRadii).with_colors(nodeColors));

    }

    // 图位置是静态的，但输出详情会随每帧数据变化。
    // 这里的名字只是图的运行时标识；真正的文本只来自对应输出的 detail()。
    string graphDetail(const string& name) const {
        if(name == "FieldModel") return field_.detail();
        if(name == "SelfState") return self_.detail();
        if(name == "Observations") return observations_.detail();
        if(name == "LocalizationState") return localization_.detail();
        if(name == "BallState") return ball_.detail();
        if(name == "FieldPotential") return fieldPotential_.detail();
        if(name == "Strategy") return strategy_.detail();
        if(name == "HeadRequest") return head_.detail();
        if(name == "MotionRequest") return motion_.detail();
        if(name == "KickRequest") return kick_.detail();
        if(name == "ActuationState") return actuation_.detail();
        return "";
    }

    void logGraphDetails() {
        if(!layout_.descriptions.empty()) {
            vector<rerun::Position2D> descriptionPoints;
            vector<string> descriptionLabels;
            vector<float> descriptionRadii;
            vector<rerun::Color> descriptionColors;
            for(const GraphDescription& description : layout_.descriptions) {
                string text;
                for(const string& name : description.names) {
                    const string detail = graphDetail(name);
                    if(detail.empty()) continue;
                    if(!text.empty()) text += "\n\n";
                    text += name + "\n" + detail;
                }
                if(text.empty()) continue;
                descriptionPoints.push_back({static_cast<float>(description.x),
                                             static_cast<float>(description.y)});
                descriptionLabels.push_back(text);
                descriptionRadii.push_back(0.0f);
                descriptionColors.push_back(rerun::Color(0x888888FF));
            }
            if(!descriptionPoints.empty())
                stream_.log("brain/graph/descriptions",
                            rerun::Points2D(descriptionPoints)
                                .with_labels(descriptionLabels)
                                .with_radii(descriptionRadii)
                                .with_colors(descriptionColors));
        }
    }
    static rerun::LineStrip2D circle(double x, double y, double r, int segments = 72) {
        vector<rerun::Position2D> points;
        for(int i = 0; i <= segments; ++i) {
            const double theta = i * 2 * 3.141592653589793 / segments;
            points.push_back({static_cast<float>(x + r * cos(theta)),
                              static_cast<float>(y + r * sin(theta))});
        }
        return rerun::LineStrip2D(points);
    }

    void logField() {
        const FieldDimensions& f = field_.dimensions;
        vector<rerun::LineStrip2D> lines;
        for(const FieldLine& line : field_.lines) {
            lines.push_back(rerun::LineStrip2D({
                {static_cast<float>(line.x0), static_cast<float>(-line.y0)},
                {static_cast<float>(line.x1), static_cast<float>(-line.y1)}
            }));
        }
        lines.push_back(circle(0, 0, f.circleRadius));
        lines.push_back(rerun::LineStrip2D({
            {static_cast<float>( f.length / 2 - f.penaltyDist - 0.2), 0},
            {static_cast<float>( f.length / 2 - f.penaltyDist + 0.2), 0}
        }));
        lines.push_back(rerun::LineStrip2D({
            {static_cast<float>( f.length / 2 - f.penaltyDist), -0.2f},
            {static_cast<float>( f.length / 2 - f.penaltyDist),  0.2f}
        }));
        lines.push_back(rerun::LineStrip2D({
            {static_cast<float>(-f.length / 2 + f.penaltyDist - 0.2), 0},
            {static_cast<float>(-f.length / 2 + f.penaltyDist + 0.2), 0}
        }));
        lines.push_back(rerun::LineStrip2D({
            {static_cast<float>(-f.length / 2 + f.penaltyDist), -0.2f},
            {static_cast<float>(-f.length / 2 + f.penaltyDist),  0.2f}
        }));
        lines.push_back(rerun::LineStrip2D({
            {static_cast<float>( f.length / 2), static_cast<float>( f.goalWidth / 2)},
            {static_cast<float>( f.length / 2 + 1.0), static_cast<float>( f.goalWidth / 2)},
            {static_cast<float>( f.length / 2 + 1.0), static_cast<float>(-f.goalWidth / 2)},
            {static_cast<float>( f.length / 2), static_cast<float>(-f.goalWidth / 2)}
        }));
        lines.push_back(rerun::LineStrip2D({
            {static_cast<float>(-f.length / 2), static_cast<float>( f.goalWidth / 2)},
            {static_cast<float>(-f.length / 2 - 1.0), static_cast<float>( f.goalWidth / 2)},
            {static_cast<float>(-f.length / 2 - 1.0), static_cast<float>(-f.goalWidth / 2)},
            {static_cast<float>(-f.length / 2), static_cast<float>(-f.goalWidth / 2)}
        }));
        stream_.log("field/map",
                    rerun::LineStrips2D(lines)
                        .with_colors({rerun::Color(0xFFFFFFFF)})
                        .with_radii({0.03f})
                        .with_draw_order(0.0));
    }

    void logLocalization() {
        if(!localization_.valid) {
            stream_.log("field/localization", rerun::Clear::FLAT);
            return;
        }
        const Pose2D& pose = localization_.poseToField;
        vector<rerun::LineStrip2D> lines;
        lines.push_back(circle(pose.x, -pose.y, 0.3, 36));
        lines.push_back(rerun::LineStrip2D({
            {static_cast<float>(pose.x), static_cast<float>(-pose.y)},
            {static_cast<float>(pose.x + 0.6 * cos(-pose.yaw)),
             static_cast<float>(-pose.y + 0.6 * sin(-pose.yaw))}
        }));
        stream_.log("field/localization",
                    rerun::LineStrips2D(lines)
                        .with_radii({0.02f})
                        .with_colors({rerun::Color(0x00FF66FF)}));
    }

    // 场地图沿用 Rerun 的 y 轴翻转；球和箭头都来自 FieldPotential 的全局输出。
    void logFieldPotential() {
        if(!fieldPotential_.valid) {
            stream_.log("field/ball", rerun::Clear::FLAT);
            stream_.log("field/kick_dir", rerun::Clear::FLAT);
            return;
        }
        const Pose2D& ball = fieldPotential_.ballToField;
        stream_.log("field/ball",
                    rerun::Points2D({{static_cast<float>(ball.x), static_cast<float>(-ball.y)}})
                        .with_radii({0.12f})
                        .with_colors({rerun::Color(0xFFB000FF)}));

        const double length = 1.2;
        const double endX = ball.x + length * cos(fieldPotential_.directionToField);
        const double endY = -ball.y - length * sin(fieldPotential_.directionToField);
        const double backX = endX - 0.22 * cos(fieldPotential_.directionToField);
        const double backY = endY + 0.22 * sin(fieldPotential_.directionToField);
        const double sideX = 0.09 * sin(fieldPotential_.directionToField);
        const double sideY = 0.09 * cos(fieldPotential_.directionToField);
        vector<rerun::LineStrip2D> lines;
        lines.push_back(rerun::LineStrip2D({
            {static_cast<float>(ball.x), static_cast<float>(-ball.y)},
            {static_cast<float>(endX), static_cast<float>(endY)}
        }));
        lines.push_back(rerun::LineStrip2D({
            {static_cast<float>(backX + sideX), static_cast<float>(backY + sideY)},
            {static_cast<float>(endX), static_cast<float>(endY)},
            {static_cast<float>(backX - sideX), static_cast<float>(backY - sideY)}
        }));
        stream_.log("field/kick_dir",
                    rerun::LineStrips2D(lines)
                        .with_radii({0.04f})
                        .with_colors({rerun::Color(0x00D5FFFF)}));
    }

    const FieldModel& field_;
    const SelfState& self_;
    const Observations& observations_;
    const LocalizationState& localization_;
    const BallState& ball_;
    const FieldPotential& fieldPotential_;
    const Strategy& strategy_;
    const HeadRequest& head_;
    const MotionRequest& motion_;
    const KickRequest& kick_;
    const ActuationState& actuation_;
    const double& now_;
    Config config_;
    GraphLayout layout_;
    string schedule_;
    bool graphLogged_ = false;
    bool fieldLogged_ = false;
    rerun::RecordingStream stream_;
};

}
