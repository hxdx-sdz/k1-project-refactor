#pragma once

#include "common/module.h"
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace brain {

using namespace std;

// 调度器只管理依赖和调用顺序，不拥有模块，也不保存算法数据。
// Brain 注册模块；prepare 把声明转换成固定执行表，
// 之后 run 只遍历该表。模块必须比调度器活得更久，注册后不能移动。
class Scheduler {
public:
    Scheduler() : prepared_(false) {}

    void add(Module& module) {
        if(prepared_) throw logic_error("cannot register after prepare");
        entries_.push_back(Entry{module.info(), &module});
    }

    void configure(rclcpp::Node& node) {
        for(const Entry& entry : entries_) entry.module->configure(node);
    }

    void prepare(const vector<string>& externalInputs) {
        prepared_ = false;
        plan_.clear();
        externalInputs_ = externalInputs;
        set<string> available(externalInputs.begin(), externalInputs.end());
        map<string, string> producers;
        set<string> names;
        for(const Entry& entry : entries_) {
            if(!names.insert(entry.info.name).second)
                throw runtime_error("duplicate module: " + entry.info.name);
            for(const string& output : entry.info.outputs) {
                if(available.count(output) || !producers.insert(
                       make_pair(output, entry.info.name)).second)
                    throw runtime_error("duplicate output: " + output);
            }
        }
        for(const Entry& entry : entries_)
            for(const string& input : entry.info.inputs)
                if(!available.count(input) && !producers.count(input))
                    throw runtime_error("missing input: " + entry.info.name + " <- " + input);

        // 拓扑排序：反复选出输入已经有来源的模块。无依赖的模块按注册顺序选择，
        // 保证相同配置得到相同计划；这段搜索只在初始化执行。
        vector<bool> added(entries_.size(), false);
        while(plan_.size() < entries_.size()) {
            size_t next = entries_.size();
            for(size_t i = 0; i < entries_.size(); ++i) {
                if(added[i]) continue;
                bool ready = true;
                for(const string& input : entries_[i].info.inputs)
                    if(!available.count(input)) ready = false;
                if(ready) { next = i; break; }
            }
            if(next == entries_.size()) {
                string blocked;
                for(size_t i = 0; i < entries_.size(); ++i)
                    if(!added[i]) blocked += " " + entries_[i].info.name;
                plan_.clear();
                throw runtime_error("cyclic dependency; blocked modules:" + blocked);
            }
            added[next] = true;
            plan_.push_back(next);
            for(const string& output : entries_[next].info.outputs)
                available.insert(output);
        }
        prepared_ = true;
    }

    void run() const {
        if(!prepared_) throw logic_error("scheduler must be prepared before run");
        for(size_t index : plan_) entries_[index].module->run();
    }

    // 依赖图的结构化出口：执行顺序的模块声明快照与外部输入名。
    // 文本版 describe 仍保留，两者来自同一份声明。
    vector<ModuleInfo> planInfos() const {
        vector<ModuleInfo> result;
        for(size_t index : plan_) result.push_back(entries_[index].info);
        return result;
    }
    const vector<string>& externalInputs() const { return externalInputs_; }

    // 每个模块只展示一行数据依赖，不重复列注册顺序和执行顺序。
    string describe() const {
        ostringstream out;
        out << "[schedule] dependencies:";
        for(const Entry& entry : entries_) {
            const ModuleInfo& info = entry.info;
            out << "\n  " << join(info.inputs)
                << " --[" << info.name << "]--> " << join(info.outputs);
        }
        return out.str();
    }

private:
    struct Entry {
        ModuleInfo info;
        Module* module;
    };
    static string join(const vector<string>& values) {
        ostringstream out;
        out << '{';
        for(size_t i = 0; i < values.size(); ++i) {
            if(i) out << ", ";
            out << values[i];
        }
        return out.str() + '}';
    }
    vector<Entry> entries_;
    vector<size_t> plan_;
    vector<string> externalInputs_;
    bool prepared_;
};

}
