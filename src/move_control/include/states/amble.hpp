#pragma once

#include "fsm/base_state.hpp"


// 前向声明
class Robot;

class AmbleState : public BaseState<Robot>{
public:
    AmbleState(Robot* robot);
    
    bool enter(Robot* robot, const std::string &last_status) override;
    std::string update(Robot* robot) override;
    
private:
    int debug_cnt;
};
