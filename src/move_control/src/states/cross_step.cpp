#include "states/cross_step.hpp"
#include "core/robot.hpp"
#include "leg/step.hpp"
#include <rcl/timer.h>
#include <rclcpp/logging.hpp>
#include <math.h>

bool use_limit_lf = false;
bool use_limit_rf = false;
bool use_limit_lb = false;
bool use_limit_rb = false;

Cross_StepState::Cross_StepState(Robot* robot)
    : BaseState<Robot>("cross_step") {
    robot->node_->declare_parameter<int>("cross_step_stage",-1);
    robot->node_->declare_parameter("step_k_F",1.0);
    robot->node_->declare_parameter("step_height",0.10);

    robot->add_param_cb([this](const rclcpp::Parameter& param){
        auto name=param.get_name();
        if(name=="cross_step_stage"){ cross_step_stage=param.as_int(); change_flag=true; }
        else if(name=="step_k_F"){ k_F=(float)param.as_double(); }
        else if(name=="step_height"){ step_height=param.as_double(); }
        return true;
    });

    mass = (robot->robot_lf_grivate+robot->robot_rf_grivate+robot->robot_lb_grivate+robot->robot_rb_grivate)/9.8;
    mass_center_pos = Vector2D(
        robot->robot_lf_grivate*robot->lf_leg_calc->pos_offset[0]+robot->robot_rf_grivate*robot->rf_leg_calc->pos_offset[0]
        +robot->robot_lb_grivate*robot->lb_leg_calc->pos_offset[0]+robot->robot_rb_grivate*robot->rb_leg_calc->pos_offset[0],
        robot->robot_lf_grivate*robot->lf_leg_calc->pos_offset[1]+robot->robot_rf_grivate*robot->rf_leg_calc->pos_offset[1]
        +robot->robot_lb_grivate*robot->lb_leg_calc->pos_offset[1]+robot->robot_rb_grivate*robot->rb_leg_calc->pos_offset[1]
    )/(mass*9.8);
}

bool Cross_StepState::enter(Robot* robot, const std::string& last_status){
    (void)robot; (void)last_status;
    cross_step_stage=-1;
    return true;
}

std::string Cross_StepState::update(Robot* robot){
    robot_interfaces::msg::RobotTarget joints_target;
    lf_wheel_force = 0.0f;
    static int last_stage = 1;
    if(cross_step_stage==-1) last_stage=-1;

    lf_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_lf_grivate);
    rf_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_rf_grivate);
    lb_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_lb_grivate);
    rb_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_rb_grivate);

    // 阶段 0: 接近台阶
    if(cross_step_stage==0 && change_flag==true){
        use_limit_lf=false; use_limit_lb=false; use_limit_rb=false; use_limit_rf=false;
        if(allow_vel){
             lf_wheel_vel=0.25;
             rf_wheel_vel=-0.25; 
             lb_wheel_vel=0.25; 
             rb_wheel_vel=-0.25;
             
            double wheel_F = k_F*((lf_wheel_vel/Robot::WHEEL_RADIUS)
            -(robot->lf_wheel_omega+robot->lb_wheel_omega-robot->rf_wheel_omega-robot->rb_wheel_omega)/4.0f);
            lf_wheel_force=wheel_F; lb_wheel_force=wheel_F; rf_wheel_force=-wheel_F; rb_wheel_force=-wheel_F;
            allow_vel=false;
        }
        auto lf_cart_force = robot->lf_leg_calc->foot_force(robot->lf_joint_pos, robot->lf_joint_torque, robot->lf_forward_torque);
        auto rf_cart_force = robot->rf_leg_calc->foot_force(robot->rf_joint_pos, robot->rf_joint_torque, robot->rf_forward_torque);
        if((lf_cart_force[0]>8.0||rf_cart_force[0]>8.0)&&!stopping){
            stopping=true; stop_t=0.0;
            lf_vel_start=lf_wheel_vel; rf_vel_start=rf_wheel_vel; lb_vel_start=lb_wheel_vel; rb_vel_start=rb_wheel_vel;
            lf_force_start=lf_wheel_force; rf_force_start=rf_wheel_force; lb_force_start=lb_wheel_force; rb_force_start=rb_wheel_force;
        }
        if(stopping){
            stop_t+=0.004;
            double tau=stop_t/stop_T; if(tau>1.0)tau=1.0;
            double s=10*pow(tau,3)-15*pow(tau,4)+6*pow(tau,5);
            lf_wheel_vel=lf_vel_start*(1-s); rf_wheel_vel=rf_vel_start*(1-s);
            lb_wheel_vel=lb_vel_start*(1-s); rb_wheel_vel=rb_vel_start*(1-s);
            double s_force=pow(s,2.0);
            lf_wheel_force=lf_force_start*(1-s_force); rf_wheel_force=rf_force_start*(1-s_force);
            lb_wheel_force=lb_force_start*(1-s_force); rb_wheel_force=rb_force_start*(1-s_force);
            if(tau>=1.0){
                stopping=false;
                lf_wheel_vel=0; rf_wheel_vel=0; lb_wheel_vel=0; rb_wheel_vel=0;
                lf_wheel_force=0; rf_wheel_force=0; lb_wheel_force=0; rb_wheel_force=0;
                step_lf_foot_pos=robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
                step_rf_foot_pos=robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
                step_lb_foot_pos=robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
                step_rb_foot_pos=robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
                lf_leg_step.update_support_trajectory(step_lf_foot_pos,Vector3D(0.0,0.05,0.0),2.0);
                rf_leg_step.update_support_trajectory(step_rf_foot_pos,Vector3D(0.0,0.05,0.0),2.0);
                lb_leg_step.update_support_trajectory(step_lb_foot_pos,Vector3D(0.0,0.05,0.0),2.0);
                rb_leg_step.update_support_trajectory(step_rb_foot_pos,Vector3D(0.0,0.05,0.0),2.0);
                cross_step_stage=1;
            }
        }
    }
    // 阶段 1: 左后腿抬起
    else if(cross_step_stage==1 && change_flag==true){
        if(cross_step_stage!=last_stage){ cross_step_stage_time=robot->node_->get_clock()->now(); 
            last_stage=cross_step_stage; }
        use_limit_lb=false; use_limit_lf=false; use_limit_rb=false; use_limit_rf=false;
        bool success=false;
        double time=(robot->node_->get_clock()->now()-cross_step_stage_time).seconds();
        std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time,success);
        std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time,success);
        std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time,success);
        std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time,success);
        auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);
        auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
        Eigen::Matrix<double,3,4> A; Eigen::Vector3d b;
        A(0,0)=1; A(0,1)=1; A(0,2)=1; A(0,3)=1; b(0)=mass*9.8;
        A(1,0)=lf_pos.x()-mass_center_pos.x(); A(1,1)=rf_pos.x()-mass_center_pos.x();
        A(1,2)=lb_pos.x()-mass_center_pos.x(); A(1,3)=rb_pos.x()-mass_center_pos.x(); b(1)=0;
        A(2,0)=lf_pos.y()-mass_center_pos.y(); A(2,1)=rf_pos.y()-mass_center_pos.y();
        A(2,2)=lb_pos.y()-mass_center_pos.y(); A(2,3)=rb_pos.y()-mass_center_pos.y(); b(2)=0;
        Eigen::Vector4d forces=A.transpose()*(A*A.transpose()).inverse()*b;
        lf_foot_exp_force=Vector3D(0,0,-forces(0)); rf_foot_exp_force=Vector3D(0,0,-forces(1));
        lb_foot_exp_force=Vector3D(0,0,-forces(2)); rb_foot_exp_force=Vector3D(0,0,-forces(3));
        if(!success){
            step_lf_foot_pos=lf_foot_exp_pos; step_rf_foot_pos=rf_foot_exp_pos;
            step_lb_foot_pos=lb_foot_exp_pos; step_rb_foot_pos=rb_foot_exp_pos;
            lf_leg_step.update_support_trajectory(step_lf_foot_pos,step_lf_foot_pos,2.0);
            rf_leg_step.update_support_trajectory(step_rf_foot_pos,step_rf_foot_pos,2.0);
            rb_leg_step.update_support_trajectory(step_rb_foot_pos,step_rb_foot_pos,2.0);
            lb_leg_step.update_flight_trajectory(step_lb_foot_pos,Vector3D(0,0,0),Vector3D(step_depth,0,step_height+0.05),Vector2D(0,0),2.0,step_height+0.05);
            cross_step_stage=2;
        }
    }
    // 阶段 2: 左后腿落下
    else if(cross_step_stage==2 && change_flag==true){
        if(cross_step_stage!=last_stage){ cross_step_stage_time=robot->node_->get_clock()->now(); last_stage=cross_step_stage; }
        bool success=false;
        use_limit_lb=false; use_limit_lf=false; use_limit_rb=false; use_limit_rf=false;
        double time=(robot->node_->get_clock()->now()-cross_step_stage_time).seconds();
        std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time,success);
        std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time,success);
        std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time,success);
        std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time,success);
        auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);
        auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
        auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
        Eigen::Matrix3d A; Eigen::Vector3d b;
        A(0,0)=1; A(0,1)=1; A(0,2)=1; b(0)=mass*9.8;
        A(1,0)=lf_pos.x()-mass_center_pos.x(); A(1,1)=rf_pos.x()-mass_center_pos.x(); A(1,2)=rb_pos.x()-mass_center_pos.x(); b(1)=0;
        A(2,0)=lf_pos.y()-mass_center_pos.y(); A(2,1)=rf_pos.y()-mass_center_pos.y(); A(2,2)=rb_pos.y()-mass_center_pos.y(); b(2)=0;
        Eigen::Vector3d forces=A.colPivHouseholderQr().solve(b);
        lf_foot_exp_force=Vector3D(0,0,-forces(0)); rf_foot_exp_force=Vector3D(0,0,-forces(1));
        lb_foot_exp_force=Vector3D::Zero(); rb_foot_exp_force=Vector3D(0,0,-forces(2));
        if(!success){
            step_lf_foot_pos=lf_foot_exp_pos; step_rf_foot_pos=rf_foot_exp_pos;
            step_lb_foot_pos=lb_foot_exp_pos; step_rb_foot_pos=rb_foot_exp_pos;
            lb_leg_step.update_support_trajectory(step_lb_foot_pos,Vector3D(step_depth,0,step_height),2.0);
            cross_step_stage=3;
        }
    }
    // 阶段 3: 重心前移
    else if(cross_step_stage==3 && change_flag==true){
        if(cross_step_stage!=last_stage){ cross_step_stage_time=robot->node_->get_clock()->now(); last_stage=cross_step_stage; }
        bool success=false;
        use_limit_lb=false; use_limit_lf=false; use_limit_rb=false; use_limit_rf=false;
        double time=(robot->node_->get_clock()->now()-cross_step_stage_time).seconds();
        std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time,success);
        std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time,success);
        std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time,success);
        std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time,success);
        if(time>1.5){ cross_step_stage=4; }
    }
    // 阶段 4: 右后腿抬起
    else if(cross_step_stage==4 && change_flag==true){
        if(cross_step_stage!=last_stage){ cross_step_stage_time=robot->node_->get_clock()->now(); last_stage=cross_step_stage; }
        bool success=false;
        use_limit_lb=false; use_limit_lf=false; use_limit_rb=false; use_limit_rf=false;
        double time=(robot->node_->get_clock()->now()-cross_step_stage_time).seconds();
        std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time,success);
        std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time,success);
        std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time,success);
        std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time,success);
        auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);
        auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
        Eigen::Matrix3d A; Eigen::Vector3d b;
        A(0,0)=1; A(0,1)=1; A(0,2)=1; b(0)=mass*9.8;
        A(1,0)=lf_pos.x()-mass_center_pos.x(); A(1,1)=rf_pos.x()-mass_center_pos.x(); A(1,2)=lb_pos.x()-mass_center_pos.x(); b(1)=0;
        A(2,0)=lf_pos.y()-mass_center_pos.y(); A(2,1)=rf_pos.y()-mass_center_pos.y(); A(2,2)=lb_pos.y()-mass_center_pos.y(); b(2)=0;
        Eigen::Vector3d forces=A.colPivHouseholderQr().solve(b);
        lf_foot_exp_force=Vector3D(0,0,-forces(0)); rf_foot_exp_force=Vector3D(0,0,-forces(1));
        lb_foot_exp_force=Vector3D(0,0,-forces(2)); rb_foot_exp_force=Vector3D::Zero();
        if(!success){
            step_lf_foot_pos=lf_foot_exp_pos; step_rf_foot_pos=rf_foot_exp_pos;
            step_lb_foot_pos=lb_foot_exp_pos; step_rb_foot_pos=rb_foot_exp_pos;
            rb_leg_step.update_flight_trajectory(step_rb_foot_pos,Vector3D(0,0,0),Vector3D(step_depth,0,step_height+0.05),Vector2D(0,0),2.0,step_height+0.05);
            cross_step_stage=5;
        }
    }
    // 阶段 5: 右后腿落下
    else if(cross_step_stage==5 && change_flag==true){
        if(cross_step_stage!=last_stage){ cross_step_stage_time=robot->node_->get_clock()->now(); last_stage=cross_step_stage; }
        bool success=false;
        use_limit_lb=false; use_limit_lf=false; use_limit_rb=false; use_limit_rf=false;
        double time=(robot->node_->get_clock()->now()-cross_step_stage_time).seconds();
        std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time,success);
        std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time,success);
        std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time,success);
        std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time,success);
        auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);
        auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
        Eigen::Matrix3d A; Eigen::Vector3d b;
        A(0,0)=1; A(0,1)=1; A(0,2)=1; b(0)=mass*9.8;
        A(1,0)=lf_pos.x()-mass_center_pos.x(); A(1,1)=rf_pos.x()-mass_center_pos.x(); A(1,2)=lb_pos.x()-mass_center_pos.x(); b(1)=0;
        A(2,0)=lf_pos.y()-mass_center_pos.y(); A(2,1)=rf_pos.y()-mass_center_pos.y(); A(2,2)=lb_pos.y()-mass_center_pos.y(); b(2)=0;
        Eigen::Vector3d forces=A.colPivHouseholderQr().solve(b);
        lf_foot_exp_force=Vector3D(0,0,-forces(0)); rf_foot_exp_force=Vector3D(0,0,-forces(1));
        lb_foot_exp_force=Vector3D(0,0,-forces(2)); rb_foot_exp_force=Vector3D::Zero();
        if(!success){
            step_lf_foot_pos=lf_foot_exp_pos; step_rf_foot_pos=rf_foot_exp_pos;
            step_lb_foot_pos=lb_foot_exp_pos; step_rb_foot_pos=rb_foot_exp_pos;
            rb_leg_step.update_support_trajectory(step_rb_foot_pos,Vector3D(step_depth,0,step_height),2.0);
            cross_step_stage=6;
        }
    }
    // 阶段 6: 左前腿抬起
    else if(cross_step_stage==6 && change_flag==true){
        if(cross_step_stage!=last_stage){ cross_step_stage_time=robot->node_->get_clock()->now(); last_stage=cross_step_stage; }
        bool success=false;
        use_limit_lb=false; use_limit_lf=false; use_limit_rb=false; use_limit_rf=false;
        double time=(robot->node_->get_clock()->now()-cross_step_stage_time).seconds();
        std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time,success);
        std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time,success);
        std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time,success);
        std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time,success);
        auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
        Eigen::Matrix3d A; Eigen::Vector3d b;
        A(0,0)=1; A(0,1)=1; A(0,2)=1; b(0)=mass*9.8;
        A(1,0)=rf_pos.x()-mass_center_pos.x(); A(1,1)=lb_pos.x()-mass_center_pos.x(); A(1,2)=rb_pos.x()-mass_center_pos.x(); b(1)=0;
        A(2,0)=rf_pos.y()-mass_center_pos.y(); A(2,1)=lb_pos.y()-mass_center_pos.y(); A(2,2)=rb_pos.y()-mass_center_pos.y(); b(2)=0;
        Eigen::Vector3d forces=A.colPivHouseholderQr().solve(b);
        lf_foot_exp_force=Vector3D::Zero();
        rf_foot_exp_force=Vector3D(0,0,-forces(0)); lb_foot_exp_force=Vector3D(0,0,-forces(1)); rb_foot_exp_force=Vector3D(0,0,-forces(2));
        if(!success){
            step_lf_foot_pos=lf_foot_exp_pos; step_rf_foot_pos=rf_foot_exp_pos;
            step_lb_foot_pos=lb_foot_exp_pos; step_rb_foot_pos=rb_foot_exp_pos;
            lf_leg_step.update_flight_trajectory(step_lf_foot_pos,Vector3D(0,0,0),Vector3D(step_depth,0,step_height+0.05),Vector2D(0,0),2.0,step_height+0.05);
            cross_step_stage=7;
        }
    }
    // 阶段 7: 左前腿落下
    else if(cross_step_stage==7 && change_flag==true){
        if(cross_step_stage!=last_stage){ cross_step_stage_time=robot->node_->get_clock()->now(); last_stage=cross_step_stage; }
        bool success=false;
        use_limit_lb=false; use_limit_lf=false; use_limit_rb=false; use_limit_rf=false;
        double time=(robot->node_->get_clock()->now()-cross_step_stage_time).seconds();
        std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time,success);
        std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time,success);
        std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time,success);
        std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time,success);
        auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
        Eigen::Matrix3d A; Eigen::Vector3d b;
        A(0,0)=1; A(0,1)=1; A(0,2)=1; b(0)=mass*9.8;
        A(1,0)=rf_pos.x()-mass_center_pos.x(); A(1,1)=lb_pos.x()-mass_center_pos.x(); A(1,2)=rb_pos.x()-mass_center_pos.x(); b(1)=0;
        A(2,0)=rf_pos.y()-mass_center_pos.y(); A(2,1)=lb_pos.y()-mass_center_pos.y(); A(2,2)=rb_pos.y()-mass_center_pos.y(); b(2)=0;
        Eigen::Vector3d forces=A.colPivHouseholderQr().solve(b);
        lf_foot_exp_force=Vector3D::Zero();
        rf_foot_exp_force=Vector3D(0,0,-forces(0)); lb_foot_exp_force=Vector3D(0,0,-forces(1)); rb_foot_exp_force=Vector3D(0,0,-forces(2));
        if(!success){
            step_lf_foot_pos=lf_foot_exp_pos; step_rf_foot_pos=rf_foot_exp_pos;
            step_lb_foot_pos=lb_foot_exp_pos; step_rb_foot_pos=rb_foot_exp_pos;
            lf_leg_step.update_support_trajectory(step_lf_foot_pos,Vector3D(step_depth,0,step_height),2.0);
            cross_step_stage=8;
        }
    }
    // 阶段 8: 右前腿抬起
    else if(cross_step_stage==8 && change_flag==true){
        if(cross_step_stage!=last_stage){ cross_step_stage_time=robot->node_->get_clock()->now(); last_stage=cross_step_stage; }
        bool success=false;
        use_limit_lb=false; use_limit_lf=false; use_limit_rb=false; use_limit_rf=false;
        double time=(robot->node_->get_clock()->now()-cross_step_stage_time).seconds();
        std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time,success);
        std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time,success);
        std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time,success);
        std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time,success);
        auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);
        auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
        Eigen::Matrix3d A; Eigen::Vector3d b;
        A(0,0)=1; A(0,1)=1; A(0,2)=1; b(0)=mass*9.8;
        A(1,0)=lf_pos.x()-mass_center_pos.x(); A(1,1)=lb_pos.x()-mass_center_pos.x(); A(1,2)=rb_pos.x()-mass_center_pos.x(); b(1)=0;
        A(2,0)=lf_pos.y()-mass_center_pos.y(); A(2,1)=lb_pos.y()-mass_center_pos.y(); A(2,2)=rb_pos.y()-mass_center_pos.y(); b(2)=0;
        Eigen::Vector3d forces=A.colPivHouseholderQr().solve(b);
        lf_foot_exp_force=Vector3D(0,0,-forces(0)); rf_foot_exp_force=Vector3D::Zero();
        lb_foot_exp_force=Vector3D(0,0,-forces(1)); rb_foot_exp_force=Vector3D(0,0,-forces(2));
        if(!success){
            step_lf_foot_pos=lf_foot_exp_pos; step_rf_foot_pos=rf_foot_exp_pos;
            step_lb_foot_pos=lb_foot_exp_pos; step_rb_foot_pos=rb_foot_exp_pos;
            rf_leg_step.update_flight_trajectory(step_rf_foot_pos,Vector3D(0,0,0),Vector3D(step_depth,0,step_height+0.05),Vector2D(0,0),2.0,step_height+0.05);
            cross_step_stage=9;
        }
    }
    // 阶段 9: 右前腿落下
    else if(cross_step_stage==9 && change_flag==true){
        if(cross_step_stage!=last_stage){ cross_step_stage_time=robot->node_->get_clock()->now(); last_stage=cross_step_stage; }
        bool success=false;
        use_limit_lb=false; use_limit_lf=false; use_limit_rb=false; use_limit_rf=false;
        double time=(robot->node_->get_clock()->now()-cross_step_stage_time).seconds();
        std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time,success);
        std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time,success);
        std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time,success);
        std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time,success);
        auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);
        auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
        Eigen::Matrix3d A; Eigen::Vector3d b;
        A(0,0)=1; A(0,1)=1; A(0,2)=1; b(0)=mass*9.8;
        A(1,0)=lf_pos.x()-mass_center_pos.x(); A(1,1)=lb_pos.x()-mass_center_pos.x(); A(1,2)=rb_pos.x()-mass_center_pos.x(); b(1)=0;
        A(2,0)=lf_pos.y()-mass_center_pos.y(); A(2,1)=lb_pos.y()-mass_center_pos.y(); A(2,2)=rb_pos.y()-mass_center_pos.y(); b(2)=0;
        Eigen::Vector3d forces=A.colPivHouseholderQr().solve(b);
        lf_foot_exp_force=Vector3D(0,0,-forces(0)); rf_foot_exp_force=Vector3D::Zero();
        lb_foot_exp_force=Vector3D(0,0,-forces(1)); rb_foot_exp_force=Vector3D(0,0,-forces(2));
        if(!success){
            step_lf_foot_pos=lf_foot_exp_pos; step_rf_foot_pos=rf_foot_exp_pos;
            step_lb_foot_pos=lb_foot_exp_pos; step_rb_foot_pos=rb_foot_exp_pos;
            rf_leg_step.update_support_trajectory(step_rf_foot_pos,Vector3D(step_depth,0,step_height),2.0);
            cross_step_stage=10;
        }
    }
    // 阶段 10: 完成
    else if(cross_step_stage==10 && change_flag==true){
        if(cross_step_stage!=last_stage){ cross_step_stage_time=robot->node_->get_clock()->now(); last_stage=cross_step_stage; }
        bool success=false;
        use_limit_lb=false; use_limit_lf=false; use_limit_rb=false; use_limit_rf=false;
        double time=(robot->node_->get_clock()->now()-cross_step_stage_time).seconds();
        std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time,success);
        std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time,success);
        std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time,success);
        std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time,success);
        if(time>1.0){ change_flag=false; return "stop"; }
    }

    // 关节控制输出
    if(use_limit_lf){
        for(int i=0;i<3;i++){ joints_target.legs[0].joints[i].kp=(float)robot->lf_leg_calc->kp[i]; joints_target.legs[0].joints[i].kd=(float)robot->lf_leg_calc->kd[i]; joints_target.legs[0].joints[i].rad=(float)lf_joint_exp_pos_[i]; }
        robot->lf_leg_calc->joint_pos_setarray(robot->lf_joint_pos);
        lf_foot_exp_pos=robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
    }else{
        joints_target.legs[0]=robot->lf_leg_calc->signal_leg_calc(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc,lf_foot_exp_force,&robot->lf_forward_torque,lf_wheel_vel,lf_wheel_force);
    }
    if(use_limit_rf){
        for(int i=0;i<3;i++){ joints_target.legs[1].joints[i].rad=(float)rf_joint_exp_pos_[i]; joints_target.legs[1].joints[i].kp=(float)robot->rf_leg_calc->kp[i]; joints_target.legs[1].joints[i].kd=(float)robot->rf_leg_calc->kd[i]; }
        robot->rf_leg_calc->joint_pos_setarray(robot->rf_joint_pos);
        rf_foot_exp_pos=robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
    }else{
        joints_target.legs[1]=robot->rf_leg_calc->signal_leg_calc(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc,rf_foot_exp_force,&robot->rf_forward_torque,rf_wheel_vel,rf_wheel_force);
    }
    if(use_limit_lb){
        for(int i=0;i<3;i++){ joints_target.legs[2].joints[i].kp=(float)robot->lb_leg_calc->kp[i]; joints_target.legs[2].joints[i].kd=(float)robot->lb_leg_calc->kd[i]; joints_target.legs[2].joints[i].rad=(float)lb_joint_exp_pos_[i]; }
        robot->lb_leg_calc->joint_pos_setarray(robot->lb_joint_pos);
        lb_foot_exp_pos=robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
    }else{
        joints_target.legs[2]=robot->lb_leg_calc->signal_leg_calc(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc,lb_foot_exp_force,&robot->lb_forward_torque,lb_wheel_vel,lb_wheel_force);
    }
    if(use_limit_rb){
        for(int i=0;i<3;i++){ joints_target.legs[3].joints[i].kp=(float)robot->rb_leg_calc->kp[i]; joints_target.legs[3].joints[i].kd=(float)robot->rb_leg_calc->kd[i]; joints_target.legs[3].joints[i].rad=(float)rb_joint_exp_pos_[i]; }
        robot->rb_leg_calc->joint_pos_setarray(robot->rb_joint_pos);
        rb_foot_exp_pos=robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
    }else{
        joints_target.legs[3]=robot->rb_leg_calc->signal_leg_calc(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc,rb_foot_exp_force,&robot->rb_forward_torque,rb_wheel_vel,rb_wheel_force);
    }

    robot->legs_target_pub->publish(joints_target);
    return "cross_step";
}