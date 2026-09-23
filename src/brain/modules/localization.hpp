#pragma once

#include "modules/field_provider.hpp"
#include "modules/self_state.hpp"
#include "modules/observations.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>

namespace brain {

using namespace std;

struct LocateResult {
    // 保留 Demo 返回码：0 成功，1 无粒子，2 残差过大，3 未收敛，4 点不足，5 概率退化。
    int code = -1;
    double residual = 1e6;
    Pose2D pose;
    int candidates = 0;
};
struct LocalizationState {
    bool valid = false;
    Pose2D poseToField, odomToField;
    string status = "waiting_odometry";
    LocateResult lastAttempt;
    uint64_t attempts = 0, corrections = 0, lineCorrections = 0;
    double lastCorrection = 0;
    double lineResidual = -1;
    int lineMatches = 0;

    string detail() const {
        ostringstream out;
        out << fixed << setprecision(2)
            << "status=" << status
            << "\nvalid=" << valid
            << "\npose=(" << poseToField.x << ',' << poseToField.y << ','
            << poseToField.yaw << ')'
            << "\nresidual=" << ((lastAttempt.residual > 100) ? -1 : lastAttempt.residual)
            << "\nattempts=" << attempts
            << "\ncorrections=" << corrections << " line_corrections=" << lineCorrections
            << "\nline_residual=" << lineResidual << " line_matches=" << lineMatches;
        return out.str();
    }
};

// 算法集中在本文件
// 只读取上游输出，不调用上游模块

class LocalizationModule : public Module {
public:
    struct Config {
        Pose2D searchRange = {1.0, 1.0, 0.7853981633974483};
        int minMarkers = 4;
        double residualTolerance = 0.4;
        double inputTimeout = 1.0, correctionInterval = 0.5;
        double lineMinLength = 0.2;
        // 全局候选的单点平均残差相差不大时，优先沿用已有位姿所在的一侧。
        double globalCandidateMargin = 0.1;
    };

    void configure(rclcpp::Node& node) override {
        config_.searchRange.x = node.declare_parameter<double>("Localization.search_range.x", config_.searchRange.x);
        config_.searchRange.y = node.declare_parameter<double>("Localization.search_range.y", config_.searchRange.y);
        config_.searchRange.yaw = node.declare_parameter<double>("Localization.search_range.yaw", config_.searchRange.yaw);
        config_.minMarkers = node.declare_parameter<int>("Localization.min_markers", config_.minMarkers);
        config_.residualTolerance = node.declare_parameter<double>("Localization.residual_tolerance", config_.residualTolerance);
        config_.inputTimeout = node.declare_parameter<double>("Localization.input_timeout", config_.inputTimeout);
        config_.correctionInterval = node.declare_parameter<double>("Localization.correction_interval", config_.correctionInterval);
        config_.lineMinLength = node.declare_parameter<double>(
            "Localization.line_min_length", config_.lineMinLength);
        config_.globalCandidateMargin = node.declare_parameter<double>(
            "Localization.global_candidate_margin", config_.globalCandidateMargin);
    }

    LocalizationModule(const FieldModel& field, const SelfState& self, const Observations& observations,
                       const double& now)
        : field_(field), self_(self), observations_(observations), now_(now), random_(1) {}
    LocalizationState output;
    ModuleInfo info() const override {
        return {"Localization", {"FieldModel", "SelfState", "Observations", "FrameTime"}, {"LocalizationState"}};
    }
    void run() override { update(field_, self_, observations_, now_, config_); }
    void update(const FieldModel& field, const SelfState& self, const Observations& observations,
                double now, const Config& config) {
        if(!self.odometry.valid()) { output.status = "waiting_odometry"; output.valid = false; return; }
        const Pose2D& odom = self.odometry.value;
        // 即使暂时看不到标志点，已校准的场地位姿仍随里程计前进；不伪造视觉更新。
        if(output.valid) output.poseToField = compose(output.odomToField, odom);
        if(now - self.odometry.updatedAt > config.inputTimeout) {
            output.status = "stale_odometry"; output.valid = false; return;
        }
        if(self.recovery.valid() && self.recovery.value.state != 0) {
            output.status = "recovering"; output.valid = false;
            // 恢复后必须有新观测重新校正，不能沿用倒地前的有效标记。
            usedDetection_ = observations.detectionRevision;
            return;
        }
        const bool markersFresh = observations.detectionRevision
            && now - observations.receivedAt <= config.inputTimeout;
        const bool linesFresh = observations.lineRevision
            && now - observations.linesReceivedAt <= config.inputTimeout;
        if(markersFresh && observations.detectionRevision != usedDetection_ && now >= nextAttempt_) {
            usedDetection_ = observations.detectionRevision;
            nextAttempt_ = now + config.correctionInterval;
            ++output.attempts;
            // 初次没有可用位姿、连续局部校正失败，或当前位姿已明显跑出场地时，
            // 都从标志点几何关系重新给出全局候选，而不是围绕旧位姿继续猜。
            const FieldDimensions& dimensions = field.dimensions;
            const bool global = !output.valid || failures_ >= 3 || outsideField(output.poseToField, dimensions);
            output.lastAttempt = global ? globalLocate(observations.markers, dimensions, config,
                                                        output.poseToField, output.valid)
                                         : locate(observations.markers, dimensions, config, output.poseToField);
            if(output.lastAttempt.code == 0) {
                // Demo calibrateOdom：field←odom = field←robot × robot←odom。
                output.odomToField = compose(output.lastAttempt.pose, inverse(odom));
                output.poseToField = output.lastAttempt.pose;
                output.valid = true;
                output.lastCorrection = now;
                ++output.corrections;
                failures_ = 0;
                output.status = global ? "global_corrected" : "corrected";
            } else {
                ++failures_;
                output.status = output.lastAttempt.code == 4 ? "not_enough_markers" :
                                (global ? "global_search_failed" : "correction_failed");
            }
        }
        if(linesFresh) correctWithLines(field, observations.lines, odom, config);
        if(!markersFresh && !linesFresh)
            output.status = output.valid ? "stale_observations" : "waiting_observations";
    }
    static vector<FieldMarker> fieldMarkers(const FieldDimensions& f) {
        vector<FieldMarker> result;
        for(int sign : {-1, 1}) {
            result.push_back({'X', 0, sign * f.circleRadius, 100});
            result.push_back({'P', sign * (f.length / 2 - f.penaltyDist), 0, 100});
            result.push_back({'T', 0, sign * f.width / 2, 100});
        }
        for(int x : {-1, 1}) for(int y : {-1, 1}) {
            result.push_back({'L', x * (f.length / 2 - f.penaltyAreaLength), y * f.penaltyAreaWidth / 2, 100});
            result.push_back({'T', x * f.length / 2, y * f.penaltyAreaWidth / 2, 100});
            result.push_back({'L', x * (f.length / 2 - f.goalAreaLength), y * f.goalAreaWidth / 2, 100});
            result.push_back({'T', x * f.length / 2, y * f.goalAreaWidth / 2, 100});
            result.push_back({'L', x * f.length / 2, y * f.width / 2, 100});
        }
        return result;
    }
    static double residual(const vector<FieldMarker>& observed,
                           const vector<FieldMarker>& field, const Pose2D& pose) {
        return markerScore(observed, field, pose);
    }
    static double markerScore(const vector<FieldMarker>& observed,
                              const vector<FieldMarker>& field, const Pose2D& pose) {
        vector<bool> usedObserved(observed.size(), false), usedField(field.size(), false);
        double sum = 0;
        for(size_t count = 0; count < observed.size(); ++count) {
            double nearest = numeric_limits<double>::infinity();
            size_t bestObserved = observed.size(), bestField = field.size();
            for(size_t i = 0; i < observed.size(); ++i) {
                if(usedObserved[i]) continue;
                const Pose2D point = compose(pose, {observed[i].x, observed[i].y, 0});
                for(size_t j = 0; j < field.size(); ++j) {
                    if(usedField[j] || observed[i].type != field[j].type) continue;
                    const double distance = hypot(point.x - field[j].x, point.y - field[j].y);
                    if(distance < nearest) {
                        nearest = distance;
                        bestObserved = i;
                        bestField = j;
                    }
                }
            }
            if(bestObserved == observed.size()) break;
            usedObserved[bestObserved] = true;
            usedField[bestField] = true;
            sum += nearest / max(hypot(observed[bestObserved].x, observed[bestObserved].y), 0.1) * 3;
        }
        return sum;
    }
    static bool outsideField(const Pose2D& pose, const FieldDimensions& field) {
        return abs(pose.x) > field.length / 2 + 1 || abs(pose.y) > field.width / 2 + 1;
    }

    struct LineScore {
        double residual = numeric_limits<double>::infinity();
        int matches = 0;
    };

    struct LineCandidate {
        Pose2D pose;
        LineScore score;
    };

    static double pointToSegment(double x, double y, const FieldLine& line) {
        const double dx = line.x1 - line.x0, dy = line.y1 - line.y0;
        const double length2 = dx * dx + dy * dy;
        if(length2 == 0) return hypot(x - line.x0, y - line.y0);
        const double projection = ((x - line.x0) * dx + (y - line.y0) * dy) / length2;
        const double t = max(0.0, min(1.0, projection));
        return hypot(x - (line.x0 + t * dx), y - (line.y0 + t * dy));
    }

    static bool linesMatch(const ObservedLine& observed, const FieldLine& field,
                           const Pose2D& pose) {
        const Pose2D p0 = compose(pose, {observed.relative[0], observed.relative[1], 0});
        const Pose2D p1 = compose(pose, {observed.relative[2], observed.relative[3], 0});
        const double angle = atan2(p1.y - p0.y, p1.x - p0.x);
        const double fieldAngle = atan2(field.y1 - field.y0, field.x1 - field.x0);
        if(abs(sin(angle - fieldAngle)) > sin(25.0 * 3.141592653589793 / 180.0)) return false;
        const double distance = (pointToSegment(p0.x, p0.y, field)
                               + pointToSegment(p1.x, p1.y, field)) / 2;
        const double length = hypot(observed.relative[2] - observed.relative[0],
                                    observed.relative[3] - observed.relative[1]);
        return distance < (length >= 2.5 ? 0.5 : 0.2);
    }

    static bool lineIntersection(const ObservedLine& first, const ObservedLine& second,
                                 double& x, double& y) {
        const double ax = first.relative[0], ay = first.relative[1];
        const double bx = first.relative[2], by = first.relative[3];
        const double cx = second.relative[0], cy = second.relative[1];
        const double dx = second.relative[2], dy = second.relative[3];
        const double denominator = (ax - bx) * (cy - dy) - (ay - by) * (cx - dx);
        if(abs(denominator) < 1e-6) return false;
        const double cross1 = ax * by - ay * bx;
        const double cross2 = cx * dy - cy * dx;
        x = (cross1 * (cx - dx) - (ax - bx) * cross2) / denominator;
        y = (cross1 * (cy - dy) - (ay - by) * cross2) / denominator;
        return true;
    }

    static bool lineIntersection(const FieldLine& first, const FieldLine& second,
                                 double& x, double& y) {
        ObservedLine a, b;
        a.relative = {first.x0, first.y0, first.x1, first.y1};
        b.relative = {second.x0, second.y0, second.x1, second.y1};
        return lineIntersection(a, b, x, y);
    }

    static LineScore lineScore(const vector<ObservedLine>& observed,
                               const vector<FieldLine>& field, const Pose2D& pose,
                               const Config& config) {
        LineScore result;
        double sum = 0;
        for(const ObservedLine& line : observed) {
            const double localDx = line.relative[2] - line.relative[0];
            const double localDy = line.relative[3] - line.relative[1];
            if(hypot(localDx, localDy) < config.lineMinLength) continue;
            const Pose2D p0 = compose(pose, {line.relative[0], line.relative[1], 0});
            const Pose2D p1 = compose(pose, {line.relative[2], line.relative[3], 0});
            double nearest = numeric_limits<double>::infinity();
            for(const FieldLine& mapLine : field) {
                if(!linesMatch(line, mapLine, pose)) continue;
                const double distance = (pointToSegment(p0.x, p0.y, mapLine)
                                       + pointToSegment(p1.x, p1.y, mapLine)) / 2;
                nearest = min(nearest, distance);
            }
            if(isfinite(nearest)) {
                sum += nearest;
                ++result.matches;
            }
        }
        if(result.matches) result.residual = sum / result.matches;
        return result;
    }

    static bool betterLineCandidate(const LineCandidate& candidate,
                                    const LineCandidate& selected,
                                    const Pose2D& prediction, bool hasPrediction) {
        if(candidate.score.matches != selected.score.matches)
            return candidate.score.matches > selected.score.matches;
        if(abs(candidate.score.residual - selected.score.residual) > 0.01)
            return candidate.score.residual < selected.score.residual;
        return hasPrediction && closerToPrediction(candidate.pose, selected.pose, prediction);
    }

    vector<LineCandidate> lineCandidates(const FieldModel& field,
                                         const vector<ObservedLine>& observed,
                                         const Config& config) const {
        vector<ObservedLine> usable;
        for(const ObservedLine& line : observed) {
            const double length = hypot(line.relative[2] - line.relative[0],
                                        line.relative[3] - line.relative[1]);
            if(length >= config.lineMinLength) usable.push_back(line);
        }
        if(usable.size() < 2) return {};

        vector<ObservedLine> parallel, perpendicular;
        const double firstAngle = atan2(usable[0].relative[3] - usable[0].relative[1],
                                        usable[0].relative[2] - usable[0].relative[0]);
        for(const ObservedLine& line : usable) {
            const double angle = atan2(line.relative[3] - line.relative[1],
                                       line.relative[2] - line.relative[0]);
            const double difference = abs(wrapAngle(angle - firstAngle));
            if(difference < 20.0 * 3.141592653589793 / 180.0 ||
               difference > 160.0 * 3.141592653589793 / 180.0)
                parallel.push_back(line);
            else if(difference > 70.0 * 3.141592653589793 / 180.0 &&
                    difference < 110.0 * 3.141592653589793 / 180.0)
                perpendicular.push_back(line);
        }
        if(parallel.empty() || perpendicular.empty() || parallel.size() > 8 || perpendicular.size() > 8)
            return {};

        vector<LineCandidate> result;
        for(const ObservedLine& first : parallel) for(const ObservedLine& second : perpendicular) {
            double localX, localY;
            if(!lineIntersection(first, second, localX, localY)) continue;
            const double localAngle = atan2(first.relative[3] - first.relative[1],
                                            first.relative[2] - first.relative[0]);
            for(size_t a = 0; a < field.lines.size(); ++a) for(size_t b = 0; b < field.lines.size(); ++b) {
                if(a == b) continue;
                double fieldX, fieldY;
                if(!lineIntersection(field.lines[a], field.lines[b], fieldX, fieldY)) continue;
                const double fieldAngle = atan2(field.lines[a].y1 - field.lines[a].y0,
                                                field.lines[a].x1 - field.lines[a].x0);
                for(int quarter = 0; quarter < 4; ++quarter) {
                    const double yaw = wrapAngle(fieldAngle - localAngle + quarter * 3.141592653589793 / 2);
                    const Pose2D pose = {fieldX - cos(yaw) * localX + sin(yaw) * localY,
                                         fieldY - sin(yaw) * localX - cos(yaw) * localY, yaw};
                    if(abs(pose.x) > field.dimensions.length / 2 + 1 ||
                       abs(pose.y) > field.dimensions.width / 2 + 1) continue;
                    if(!linesMatch(first, field.lines[a], pose) ||
                       !linesMatch(second, field.lines[b], pose)) continue;
                    const LineScore score = lineScore(usable, field.lines, pose, config);
                    if(score.matches < 2) continue;
                    result.push_back({pose, score});
                }
            }
        }
        return result;
    }

    void correctWithLines(const FieldModel& field, const vector<ObservedLine>& lines,
                          const Pose2D& odom, const Config& config) {
        if(observations_.lineRevision == usedLines_) return;
        usedLines_ = observations_.lineRevision;
        const vector<LineCandidate> candidates = lineCandidates(field, lines, config);
        if(candidates.empty()) return;

        const bool initial = !output.valid;
        bool selected = false;
        LineCandidate best;
        for(const LineCandidate& candidate : candidates) {
            if(initial && candidate.pose.x >= 0) continue;
            if(!selected || betterLineCandidate(candidate, best, output.poseToField, output.valid)) {
                best = candidate;
                selected = true;
            }
        }
        if(!selected || (initial && best.score.matches < 3)) return;

        output.lineResidual = best.score.residual;
        output.lineMatches = best.score.matches;
        if(initial) {
            output.poseToField = best.pose;
            output.status = "line_initialized";
            output.valid = true;
        } else {
            const double dx = max(-0.5, min(0.5, best.pose.x - output.poseToField.x));
            const double dy = max(-0.5, min(0.5, best.pose.y - output.poseToField.y));
            const double da = max(-0.5, min(0.5, wrapAngle(best.pose.yaw - output.poseToField.yaw)));
            output.poseToField.x += dx * 0.1;
            output.poseToField.y += dy * 0.1;
            output.poseToField.yaw = wrapAngle(output.poseToField.yaw + da * 0.15);
            output.status = "line_corrected";
        }
        output.odomToField = compose(output.poseToField, inverse(odom));
        output.lastCorrection = now_;
        ++output.lineCorrections;
    }

    // 全局搜索常会产生左右镜像或整场旋转的近似等价解。
    // 它们的几何残差接近时，位置连续性优先；位置同样接近时再比较朝向连续性。
    static bool closerToPrediction(const Pose2D& candidate, const Pose2D& selected,
                                   const Pose2D& prediction) {
        const double candidateDistance = hypot(candidate.x - prediction.x, candidate.y - prediction.y);
        const double selectedDistance = hypot(selected.x - prediction.x, selected.y - prediction.y);
        if(abs(candidateDistance - selectedDistance) > 0.05)
            return candidateDistance < selectedDistance;
        return abs(wrapAngle(candidate.yaw - prediction.yaw)) <
               abs(wrapAngle(selected.yaw - prediction.yaw));
    }

    LocateResult globalLocate(const vector<FieldMarker>& observed, const FieldDimensions& dimensions,
                              const Config& config, const Pose2D& prediction,
                              bool hasPrediction) {
        LocateResult result;
        if(observed.size() < static_cast<size_t>(config.minMarkers)) { result.code = 4; return result; }
        const vector<FieldMarker> field = fieldMarkers(dimensions);
        struct Seed { Pose2D pose; double score; };
        vector<Seed> seeds;
        double best = numeric_limits<double>::infinity();
        Pose2D bestSeed;
        int candidates = 0;
        for(size_t i = 0; i < observed.size(); ++i) for(size_t j = i + 1; j < observed.size(); ++j) {
            const double observedDistance = hypot(observed[j].x - observed[i].x, observed[j].y - observed[i].y);
            if(observedDistance < 0.1) continue;
            for(size_t a = 0; a < field.size(); ++a) for(size_t b = 0; b < field.size(); ++b) {
                if(a == b || observed[i].type != field[a].type || observed[j].type != field[b].type) continue;
                const double fieldDistance = hypot(field[b].x - field[a].x, field[b].y - field[a].y);
                if(abs(fieldDistance - observedDistance) > max(0.3, fieldDistance * 0.15)) continue;
                ++candidates;
                const double yaw = atan2(field[b].y - field[a].y, field[b].x - field[a].x) -
                                   atan2(observed[j].y - observed[i].y, observed[j].x - observed[i].x);
                const double x = field[a].x - cos(yaw) * observed[i].x + sin(yaw) * observed[i].y;
                const double y = field[a].y - sin(yaw) * observed[i].x - cos(yaw) * observed[i].y;
                const Pose2D candidate = {x, y, wrapAngle(yaw)};
                // 首次全局定位固定“初始在己方半场”。场地几何本身左右对称，
                // 不给这一条比赛语义，视觉无法区分朝向相反的两个等价答案。
                if(!hasPrediction && candidate.x >= 0) continue;
                const double score = markerScore(observed, field, candidate);
                seeds.push_back({candidate, score});
                if(score < best) { best = score; bestSeed = candidate; }
            }
        }
        if(!isfinite(best)) { result.code = 4; return result; }
        Pose2D seed = bestSeed;
        if(hasPrediction) {
            const double scoreMargin = config.globalCandidateMargin * observed.size();
            for(const Seed& candidate : seeds)
                if(candidate.score <= best + scoreMargin &&
                   closerToPrediction(candidate.pose, seed, prediction))
                    seed = candidate.pose;
        }
        result = locate(observed, dimensions, config, seed);
        if(!hasPrediction && result.code == 0 && result.pose.x >= 0)
            result.code = 2;
        result.candidates = candidates;
        return result;
    }
    LocateResult locate(const vector<FieldMarker>& observed,
                        const FieldDimensions& dimensions, const Config& config, const Pose2D& center) {
        LocateResult result;
        if(observed.size() < static_cast<size_t>(config.minMarkers)) { result.code = 4; return result; }
        const vector<FieldMarker> field = fieldMarkers(dimensions);
        struct Particle { Pose2D pose; double residual, cumulative; };
        vector<Particle> particles(200);
        uniform_real_distribution<double> unit(0, 1);
        const Pose2D& range = config.searchRange;
        for(size_t i = 0; i < particles.size(); ++i) {
            Particle& p = particles[i];
            p.pose = i == 0 ? center : Pose2D{center.x + (2 * unit(random_) - 1) * range.x,
                                               center.y + (2 * unit(random_) - 1) * range.y,
                                               center.yaw + (2 * unit(random_) - 1) * range.yaw};
        }
        double best = numeric_limits<double>::infinity();
        Pose2D offset(2, 2, 0.7853981633974483);
        // 与 Demo 相同：初始评分后最多重采样 20 次。
        for(int iteration = 0; iteration <= 20; ++iteration) {
            double mean = 0, minimum = numeric_limits<double>::infinity();
            Pose2D low = particles.front().pose, high = low;
            for(Particle& p : particles) {
                p.residual = residual(observed, field, p.pose);
                mean += p.residual;
                minimum = min(minimum, p.residual);
                if(p.residual < best) { best = p.residual; result.pose = p.pose; }
                low.x = min(low.x, p.pose.x); high.x = max(high.x, p.pose.x);
                low.y = min(low.y, p.pose.y); high.y = max(high.y, p.pose.y);
                low.yaw = min(low.yaw, p.pose.yaw); high.yaw = max(high.yaw, p.pose.yaw);
            }
            result.residual = best / observed.size();
            // 先判收敛，使相同粒子/固定先验不会因标准差为零被当成概率失败。
            if(high.x - low.x < 0.2 && high.y - low.y < 0.2 && high.yaw - low.yaw < 0.2) {
                result.code = result.residual <= config.residualTolerance ? 0 : 2;
                result.pose.yaw = wrapAngle(result.pose.yaw);
                return result;
            }
            if(iteration == 20) break;
            mean /= particles.size();
            double variance = 0;
            for(const Particle& p : particles) variance += (p.residual - mean) * (p.residual - mean);
            const double sigma = sqrt(variance / (particles.size() - 1));
            if(!isfinite(sigma) || sigma < 1e-5) { result.code = 5; return result; }
            const double mu = minimum - 2 * sigma;
            double sum = 0;
            for(Particle& p : particles) {
                const double z = (p.residual - mu) / sigma;
                sum += exp(-0.5 * z * z) / (sqrt(2 * 3.141592653589793) * sigma);
                p.cumulative = sum;
            }
            if(sum < 1e-5 || !isfinite(sum)) { result.code = 5; return result; }
            for(Particle& p : particles) p.cumulative /= sum;
            particles.back().cumulative = 1; // 避免浮点累计误差造成重采样越界
            vector<Particle> next(static_cast<size_t>(particles.size() * 0.85));
            if(next.empty()) { result.code = 1; return result; }
            offset.x *= 0.8; offset.y *= 0.8; offset.yaw *= 0.8;
            for(Particle& p : next) {
                const double draw = unit(random_);
                size_t index = 0;
                while(particles[index].cumulative < draw) ++index;
                const Pose2D& parent = particles[index].pose;
                p.pose.x = max(center.x - range.x, min(center.x + range.x, parent.x + (2 * unit(random_) - 1) * offset.x));
                p.pose.y = max(center.y - range.y, min(center.y + range.y, parent.y + (2 * unit(random_) - 1) * offset.y));
                p.pose.yaw = max(center.yaw - range.yaw, min(center.yaw + range.yaw, parent.yaw + (2 * unit(random_) - 1) * offset.yaw));
            }
            particles.swap(next);
        }
        result.code = 3;
        return result;
    }
private:
    const FieldModel& field_;
    const SelfState& self_;
    const Observations& observations_;
    const double& now_;
    Config config_;
    mt19937 random_; // 实例内随机状态，重放可重复，不修改全局 rand
    uint64_t usedDetection_ = 0;
    uint64_t usedLines_ = 0;
    double nextAttempt_ = 0;
    int failures_ = 0;
};

}
