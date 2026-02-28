#include "core/robot.hpp"
#include "fsm/base_state.hpp"


class Robot;

class MPCState : public BaseState<Robot>{
public:
    MPCState(Robot* robot);
    
    bool enter(Robot* robot, const std::string &last_status) override;
    std::string update(Robot* robot) override;
    
private:

};

