#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace motion {

// Booster .bin 是四层 ELU MLP。这里直接读取其公开的 Eigen tensor 流，
// 不引入 Torch、Python 或第二套运行时。
class Policy {
public:
    void load(const std::string& path) {
        std::ifstream file(path.c_str(), std::ios::binary);
        if(!file) throw std::runtime_error("cannot open policy: " + path);

        std::vector<Eigen::MatrixXd> weights(4);
        std::vector<Eigen::VectorXd> biases(4);
        bool hasWeight[4] = {false, false, false, false};
        bool hasBias[4] = {false, false, false, false};
        while(file.peek() != EOF) {
            const std::string name = readName(file);
            const int rows = readInt(file);
            const int columns = readInt(file);
            if(rows <= 0 || columns <= 0) throw std::runtime_error("invalid tensor shape in " + path);

            std::vector<double> raw(rows * columns);
            file.read(reinterpret_cast<char*>(&raw[0]), sizeof(double) * raw.size());
            if(!file) throw std::runtime_error("truncated tensor in " + path);
            Eigen::MatrixXd tensor(rows, columns);
            for(int row = 0; row < rows; ++row)
                for(int column = 0; column < columns; ++column)
                    tensor(row, column) = raw[row * columns + column];
            const int layer = actorLayer(name);
            if(layer >= 0 && name.find(".weight") != std::string::npos) {
                weights[layer] = tensor;
                hasWeight[layer] = true;
            }
            if(layer >= 0 && name.find(".bias") != std::string::npos) {
                Eigen::VectorXd bias(tensor.size());
                for(int i = 0; i < tensor.size(); ++i) bias[i] = raw[i];
                biases[layer] = bias;
                hasBias[layer] = true;
            }
        }
        for(int i = 0; i < 4; ++i)
            if(!hasWeight[i] || !hasBias[i]) throw std::runtime_error("unsupported policy layout: " + path);
        for(size_t i = 0; i < weights.size(); ++i) {
            if(weights[i].rows() != biases[i].size()) throw std::runtime_error("policy weight/bias mismatch: " + path);
            if(i && weights[i].cols() != weights[i - 1].rows()) throw std::runtime_error("policy layer mismatch: " + path);
        }
        weights_ = weights;
        biases_ = biases;
    }

    bool loaded() const { return weights_.size() == 4; }
    int inputSize() const { return loaded() ? weights_[0].cols() : 0; }
    int outputSize() const { return loaded() ? weights_[3].rows() : 0; }

    Eigen::VectorXd run(const Eigen::VectorXd& input) const {
        if(input.size() != inputSize()) throw std::runtime_error("policy input size mismatch");
        Eigen::VectorXd value = input;
        for(int i = 0; i < 4; ++i) {
            value = weights_[i] * value + biases_[i];
            if(i < 3) {
                for(int j = 0; j < value.size(); ++j)
                    value[j] = value[j] < 0 ? std::exp(value[j]) - 1 : value[j];
            }
        }
        return value;
    }

private:
    static int actorLayer(const std::string& name) {
        if(name.find("actor.0.") == 0) return 0;
        if(name.find("actor.2.") == 0) return 1;
        if(name.find("actor.4.") == 0) return 2;
        if(name.find("actor.6.") == 0) return 3;
        return -1;
    }

    static int readInt(std::ifstream& file) {
        int value = 0;
        file.read(reinterpret_cast<char*>(&value), sizeof(value));
        if(!file) throw std::runtime_error("truncated policy integer");
        return value;
    }

    static std::string readName(std::ifstream& file) {
        const int size = readInt(file);
        if(size <= 0 || size > 4096) throw std::runtime_error("invalid policy tensor name");
        std::string name(size, '\0');
        file.read(&name[0], size);
        if(!file) throw std::runtime_error("truncated policy tensor name");
        return name;
    }

    std::vector<Eigen::MatrixXd> weights_;
    std::vector<Eigen::VectorXd> biases_;
};

class History {
public:
    explicit History(int frameSize = 0) : frameSize_(frameSize) {}

    Eigen::VectorXd push(const Eigen::VectorXd& frame) {
        if(frame.size() != frameSize_) throw std::runtime_error("history frame size mismatch");
        if(values_.empty()) {
            values_.assign(10, frame);
        } else {
            for(int i = 0; i < 9; ++i) values_[i] = values_[i + 1];
            values_[9] = frame;
        }
        Eigen::VectorXd result(frameSize_ * 10);
        for(int i = 0; i < 10; ++i) result.segment(i * frameSize_, frameSize_) = values_[i];
        return result;
    }

    void clear() { values_.clear(); }

private:
    int frameSize_;
    std::vector<Eigen::VectorXd> values_;
};

}
