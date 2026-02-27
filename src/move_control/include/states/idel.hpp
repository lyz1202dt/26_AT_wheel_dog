#pragma once

#include "fsm/base_state.hpp"

// 前向声明
class Robot;

class IdelState : public BaseState<Robot>{
public:
    IdelState(Robot* robot);
    
    bool enter(Robot* robot, const std::string &last_status) override;
    std::string update(Robot* robot) override;
    
private:
    int debug_cnt;
};
