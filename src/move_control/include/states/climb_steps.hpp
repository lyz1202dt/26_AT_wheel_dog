#include "core/robot.hpp"
#include "fsm/base_state.hpp"


class Robot;

class ClimbStepstate : public BaseState<Robot>{
public:
    explicit ClimbStepstate(Robot* robot);
    
    bool enter(Robot* robot, const std::string &last_status) override;
    std::string update(Robot* robot) override;
    
private:
std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> balance_force_calc(Robot* robot, double cur_roll, double cur_pitch);
    double exp_vel_kp{0.5};
    double current_exp_vel{0.0};
    double current_body_vel{0.0};
    int req_state{0};
    int last_state{0};
    int current_state{0};
    int foot_climbing_step{0};
    bool foot_trajectory_updated{false};
    rclcpp::Time foot_climbing_time;
    

    LegStep lf_leg_step, rf_leg_step, lb_leg_step, rb_leg_step;
    double step_time{0.5};                                        // 整个对角步态全程的时间
    double step_height{0.08};
    double step_support_rate{0.6};
    rclcpp::Time main_phrase_start_time, slave_phrase_start_time; // 第一、二相位步态开始时间
    rclcpp::Time slave_phrase_stop_time;
    bool step1_support_updated{false};
    bool step2_support_updated{false};
    bool step1_flight_updated{false};
    bool step2_flight_updated{false};

    double vy_dead_range{0.1};
    double vz_dead_range={0.1};

    double foot_obstruct_gate{-2.0};
};

