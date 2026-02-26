#pragma once

#include <string>

class FSM;

class BaseState{
    public:
    explicit BaseState(const std::string &name){this->name=name;};
    virtual ~BaseState()= default;

    virtual bool enter(const FSM *fsm,const std::string &last_status)=0;
    virtual std::string update(const FSM *fsm)=0;
    std::string name;       //建议状态名不要超过15个字节，超过SSO容量后会触发string的堆内存分配，影响效率
};
