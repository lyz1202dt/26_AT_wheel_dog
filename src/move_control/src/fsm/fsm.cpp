#include "fsm/fsm.hpp"
#include <iostream>


FSM::FSM(const std::string &init_state)
{
    first_run = true;
    last_state = init_state;
}

FSM::~FSM()
{
    
}

void  FSM::run()
{
    // 如果没有状态，直接返回
    if (states.empty()) {
        return;
    }

    // 如果是第一次运行，初始化到第一个状态
    if (first_run) {
        if(states.find(last_state) != states.end())
        states[last_state]->enter(this, "");
        first_run = false;
    }

    // 调用当前状态的update获取下一个状态名
    std::string next_state = states[last_state]->update(this);

    // 如果返回的状态名与当前状态不同，则切换状态
    if (next_state != last_state) {
        // 检查下一个状态是否存在
        if (states.find(next_state) != states.end()) {
            std::string prev_state = last_state;
            last_state = next_state;
            // 调用新状态的enter方法
            states[last_state]->enter(this, prev_state);
        } else {
            // 状态跳转失败，调用处理方法
            on_state_transition_failed(last_state, next_state);
        }
    }
}


bool FSM::register_state(const std::unique_ptr<BaseState> state)
{
    if (!state) {
        return false;
    }

    std::string state_name = state->name;
    
    // 检查状态是否已存在
    if (states.find(state_name) != states.end()) {
        return false;
    }

    // 使用move语义将unique_ptr移动到map中
    states[state_name] = std::move(const_cast<std::unique_ptr<BaseState>&>(state));
    return true;
}

bool FSM::unregister_state(const std::string &name)
{
    // 查找并删除状态
    auto it = states.find(name);
    if (it != states.end()) {
        states.erase(it);
        return true;
    }
    return false;
}

void FSM::on_state_transition_failed(const std::string &current_state, const std::string &target_state)
{
    std::cout << "[FSM] State transition failed! Current state: " 
              << current_state << ", Target state: " << target_state << std::endl;
}
