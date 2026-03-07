#include "states/amble.hpp"
#include "core/robot.hpp"


AmbleState::AmbleState(Robot* robot)
    : BaseState<Robot>("amble") {
    (void)robot;
}

bool AmbleState::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    debug_cnt=0;
    return true;
}

std::string AmbleState::update(Robot* robot) {
    
    return "amble";
}
