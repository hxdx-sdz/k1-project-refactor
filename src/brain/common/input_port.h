#pragma once

#include <cstdint>
#include <mutex>

namespace brain {

using namespace std;

// 一份输入除了“内容是什么”，还需要说明“有没有收到、多久没更新”。
// revision 为 0 表示从未收到，之后每次接收加一；updatedAt 是接收端
// 单调时钟的秒数，用于判断数据新旧，不代表传感器的采样时刻。
template<class T>
struct Sample {
    T value;
    double updatedAt;
    uint64_t revision;
    Sample() : value(), updatedAt(0), revision(0) {}
    bool valid() const { return revision != 0; }
};

// 输入端口把消息接收与 Brain 的执行节奏分开：消息到达就覆盖最新值，
// Brain 到自己的 Tick 再取用。两次 Tick 之间的中间消息不会保留。
//
// 多个消费者可以分别比较 revision 判断是否更新，互不清除对方的事件。
// read 返回独立副本，后续消息不会改变已经读到的数据；各端口之间不做时间对齐。
template<class T>
class InputPort {
public:
    void write(const T& value, double now) {
        lock_guard<mutex> lock(mutex_);
        latest_.value = value;
        latest_.updatedAt = now;
        ++latest_.revision;
    }

    Sample<T> read() const {
        lock_guard<mutex> lock(mutex_);
        return latest_;
    }

private:
    mutable mutex mutex_;
    Sample<T> latest_;
};

}
