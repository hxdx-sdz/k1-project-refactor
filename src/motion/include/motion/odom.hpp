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

// 与 K1_odom_model_20260128_152713 成对发布的官方标定参数。
inline OdomCalibration k1Odom20260128Calibration() {
    return {
        {0.98197, -0.0046688, 0.00011839, 0.0046722, 0.98197, -0.000076649,
         -0.000066154, 0.00010093, 0.99953, 0.00036994, 0.0011922, 0.0095861,
         -0.10216, 0.051952, 9.7849, -0.28482, 0.0070854, 0.013864, 0.39954,
         -0.16429, 0.018119, -0.27976, -0.0028276, -0.017625, 0.40313, -0.14074,
         -0.014925, -0.0046144, -0.0053214, 0.0045190, -0.0045163, 0.0021489,
         -0.0046973, 0.0045580, -0.0036131, 0.0038598, 0.0039056, 0.0029058, 0.021452},
        {0.060042, 0.17787, 0.021745, 0.17788, 0.060029, 0.021570,
         0.021687, 0.021628, 0.00084860, 0.32136, 0.27714, 0.67120,
         2.6908, 2.6318, 4.9752, 0.24326, 0.069868, 0.085098, 0.33803,
         0.13636, 0.068686, 0.23849, 0.072647, 0.084776, 0.33425, 0.14620,
         0.066263, 1.6190, 0.70481, 0.72105, 3.0032, 1.4236, 1.2506,
         1.5869, 0.75354, 0.71201, 2.9700, 1.5010, 1.2543},
        {0.0013219, 0.00023894}, {0.0070, 0.0052}, {0.0020, 0.0006}
    };
}

// 走路和踢球共用连续历史。整个窗口以首帧姿态为参考，不能逐帧去掉 yaw。
class OdomObservation {
public:
    void init(int size) {
        frames_.resize(39, size);
        orientations_.resize(4, size);
        started_ = false;
    }

    Eigen::VectorXd push(const Eigen::VectorXd& frame, const Eigen::Quaterniond& orientation) {
        const int size = frames_.cols();
        if(!started_) {
            const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
            start_ = Eigen::AngleAxisd(atan2(rotation(1, 0), rotation(0, 0)), Eigen::Vector3d::UnitZ());
            Eigen::VectorXd initial = frame;
            initial.segment(9, 6).setZero();
            initial[14] = 9.81;
            initial.segment(27, 12).setZero();
            const Eigen::Quaterniond initialOrientation = start_.conjugate() * orientation;
            for(int i = 0; i < size; ++i) {
                frames_.col(i) = initial;
                orientations_.col(i) = initialOrientation.coeffs();
            }
            started_ = true;
        } else {
            for(int i = 0; i + 1 < size; ++i) {
                frames_.col(i) = frames_.col(i + 1);
                orientations_.col(i) = orientations_.col(i + 1);
            }
        }
        frames_.col(size - 1) = frame;
        const Eigen::Quaterniond current = start_.conjugate() * orientation;
        orientations_.col(size - 1) = current.coeffs();

        const Eigen::Quaterniond first(orientations_.col(0));
        Eigen::VectorXd window(39 * size);
        for(int i = 0; i < size; ++i) {
            const Eigen::Quaterniond sample(orientations_.col(i));
            const Eigen::Matrix3d rotation = (first.conjugate() * sample).toRotationMatrix();
            window.segment(39 * i, 39) = frames_.col(i);
            for(int row = 0; row < 3; ++row)
                for(int column = 0; column < 3; ++column)
                    window[39 * i + row * 3 + column] = rotation(row, column);
        }
        return window;
    }

    // 模型位移属于窗口首帧坐标系；发布的朝向则属于当前帧。
    double firstYaw() const { return yaw(0); }
    double currentYaw() const { return yaw(orientations_.cols() - 1); }

private:
    double yaw(int index) const {
        const Eigen::Quaterniond orientation(orientations_.col(index));
        const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
        return atan2(rotation(1, 0), rotation(0, 0));
    }

    Eigen::MatrixXd frames_;
    Eigen::Matrix4Xd orientations_;
    Eigen::Quaterniond start_ = Eigen::Quaterniond::Identity();
    bool started_ = false;
};

// K1 Odom 模型：每帧 39 维本体感觉，输出窗口首帧坐标系下的平面位移增量。
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
    }

    bool loaded() const { return weights_.size() == 4; }
    int historySize() const { return historySize_; }

    Eigen::Vector2d infer(const Eigen::VectorXd& window) const {
        if(!loaded()) throw std::runtime_error("odom model is not loaded");
        if(window.size() < kFrameSize * historySize_)
            throw std::invalid_argument("odom history does not match model input");
        // 当前官方实现从共享窗口开头读取网络需要的长度：RVK 为 30 帧，走路为 50 帧。
        Eigen::VectorXd value(kFrameSize * historySize_);
        for(int i = 0; i < value.size(); ++i) {
            const int component = i % kFrameSize;
            value[i] = (window[i] - calibration_.inputMean[component]) / calibration_.inputStd[component];
        }
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
    int historySize_ = 0;
};

}
