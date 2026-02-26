#pragma once


#include "fsm/base_state.hpp"
#include <memory>
#include <map>
#include <string>

class BaseState;

class FSM{
    public:
    explicit FSM(const std::string &init_state);
    ~FSM();

    void run();
    bool register_state(const std::unique_ptr<BaseState> state);
    bool unregister_state(const std::string &name);

    protected:
    virtual void on_state_transition_failed(const std::string &current_state, const std::string &target_state);

    public:

    bool first_run;
    std::string last_state;
    std::map<std::string,std::unique_ptr<BaseState>> states;
};
