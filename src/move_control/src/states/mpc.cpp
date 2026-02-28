#include "states/mpc.hpp"

MPCState::MPCState(Robot* robot) : BaseState<Robot>("mpc")
{

}

bool MPCState::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    return true;
}

std::string MPCState::update(Robot* robot) {

    return "mpc";
}
