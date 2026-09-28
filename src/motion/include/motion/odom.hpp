#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace motion {

// Odom 模型的归一化参数随训练版本变化，不能和权重文件拆开使用。
struct OdomCalibration {
    std::vector<double> inputMean;
    std::vector<double> inputStd;
    std::vector<double> outputMean;
    std::vector<double> outputStd;
    std::vector<double> outputDeadZone;
};

// K1 Odom 模型：每帧 39 维本体感觉，输出机器人坐标系下的平面位移增量。
// 权重文件内只有网络和噪声参数，输入输出的归一化参数由配套配置提供。
class OdomModel {
public:
    explicit OdomModel(const OdomCalibration& calibration) : calibration_(calibration) {
        if(calibration_.inputMean.size() != kFrameSize || calibration_.inputStd.size() != kFrameSize)
            throw std::invalid_argument("odom calibration needs 39 input values");
        if(calibration_.outputMean.size() != kOutputSize || calibration_.outputStd.size() != kOutputSize
           || calibration_.outputDeadZone.size() != kOutputSize)
            throw std::invalid_argument("odom calibration needs 2 output values");
        for(int i = 0; i < kFrameSize; ++i)
            if(calibration_.inputStd[i] == 0) throw std::invalid_argument("odom input std cannot be zero");
    }

    void load(const std::string& path) {
        std::ifstream file(path.c_str(), std::ios::binary);
        if(!file) throw std::runtime_error("cannot open odom model: " + path);

        std::vector<Eigen::MatrixXd> weights(4);
        std::vector<Eigen::VectorXd> biases(4);
        bool hasWeight[4] = {false, false, false, false};
        bool hasBias[4] = {false, false, false, false};

        while(file.peek() != EOF) {
            const std::string name = readName(file);
            const int rows = readInt(file);
            const int columns = readInt(file);
            if(rows <= 0 || columns <= 0) throw std::runtime_error("invalid odom tensor shape: " + path);

            std::vector<double> raw(rows * columns);
            file.read(reinterpret_cast<char*>(&raw[0]), sizeof(double) * raw.size());
            if(!file) throw std::runtime_error("truncated odom model: " + path);

            const int layer = networkLayer(name);
            if(layer < 0) continue;

            if(name.find(".weight") != std::string::npos) {
                weights[layer].resize(rows, columns);
                for(int row = 0; row < rows; ++row)
                    for(int column = 0; column < columns; ++column)
                        weights[layer](row, column) = raw[row * columns + column];
                hasWeight[layer] = true;
            }
            if(name.find(".bias") != std::string::npos) {
                biases[layer].resize(rows);
                for(int row = 0; row < rows; ++row) biases[layer][row] = raw[row];
                hasBias[layer] = true;
            }
        }

        for(int i = 0; i < 4; ++i)
            if(!hasWeight[i] || !hasBias[i]) throw std::runtime_error("unsupported odom model layout: " + path);
        if(weights[0].cols() % kFrameSize != 0 || weights[3].rows() != kOutputSize)
            throw std::runtime_error("unsupported odom model dimensions: " + path);
        for(int i = 0; i < 4; ++i) {
            if(weights[i].rows() != biases[i].size()) throw std::runtime_error("odom weight/bias mismatch: " + path);
            if(i && weights[i].cols() != weights[i - 1].rows())
                throw std::runtime_error("odom layer mismatch: " + path);
        }

        weights_ = weights;
        biases_ = biases;
        historySize_ = weights_[0].cols() / kFrameSize;
        reset();
    }

    bool loaded() const { return weights_.size() == 4; }
    int historySize() const { return historySize_; }

    Eigen::Vector2d update(const Eigen::VectorXd& rawFrame) {
        if(!loaded()) throw std::runtime_error("odom model is not loaded");
        if(rawFrame.size() != kFrameSize) throw std::invalid_argument("odom frame needs 39 values");

        Eigen::VectorXd frame(kFrameSize);
        for(int i = 0; i < kFrameSize; ++i)
            frame[i] = (rawFrame[i] - calibration_.inputMean[i]) / calibration_.inputStd[i];

        if(history_.empty()) {
            history_.assign(historySize_, frame);
        } else {
            for(int i = 0; i + 1 < historySize_; ++i) history_[i] = history_[i + 1];
            history_[historySize_ - 1] = frame;
        }

        Eigen::VectorXd value(kFrameSize * historySize_);
        for(int i = 0; i < historySize_; ++i) value.segment(i * kFrameSize, kFrameSize) = history_[i];
        for(int layer = 0; layer < 4; ++layer) {
            value = weights_[layer] * value + biases_[layer];
            if(layer < 3)
                for(int i = 0; i < value.size(); ++i)
                    value[i] = value[i] < 0 ? std::exp(value[i]) - 1 : value[i];
        }

        Eigen::Vector2d delta;
        for(int i = 0; i < kOutputSize; ++i) {
            delta[i] = value[i] * calibration_.outputStd[i] + calibration_.outputMean[i];
            if(std::abs(delta[i]) < calibration_.outputDeadZone[i]) delta[i] = 0;
        }
        return delta;
    }

    void reset() { history_.clear(); }

private:
    static const int kFrameSize = 39;
    static const int kOutputSize = 2;

    static int readInt(std::ifstream& file) {
        int value = 0;
        file.read(reinterpret_cast<char*>(&value), sizeof(value));
        if(!file) throw std::runtime_error("truncated odom model integer");
        return value;
    }

    static std::string readName(std::ifstream& file) {
        const int size = readInt(file);
        if(size <= 0 || size > 4096) throw std::runtime_error("invalid odom tensor name");
        std::string name(size, '\0');
        file.read(&name[0], size);
        if(!file) throw std::runtime_error("truncated odom tensor name");
        return name;
    }

    static int networkLayer(const std::string& name) {
        if(name.find("net.0.") == 0) return 0;
        if(name.find("net.2.") == 0) return 1;
        if(name.find("net.4.") == 0) return 2;
        if(name.find("net.6.") == 0) return 3;
        return -1;
    }

    OdomCalibration calibration_;
    std::vector<Eigen::MatrixXd> weights_;
    std::vector<Eigen::VectorXd> biases_;
    std::vector<Eigen::VectorXd> history_;
    int historySize_ = 0;
};

}
