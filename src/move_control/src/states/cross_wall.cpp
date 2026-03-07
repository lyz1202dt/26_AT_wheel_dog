#include "states/cross_wall.hpp"
#include "core/robot.hpp"


Cross_WallState::Cross_WallState(Robot* robot)
    : BaseState<Robot>("cross_wall") {
    (void)robot;
}

bool Cross_WallState::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    cross_wall_stage=-1;
    return true;
}
//    auto lf_cart_force = robot->lf_leg_calc->foot_force(robot->lf_joint_pos, robot->lf_joint_torque, robot->lf_forward_torque);
std::string Cross_WallState::update(Robot* robot){
        if(cross_wall_stage == -1){

            lf_wheel_vel= 1.0;
            rf_wheel_vel=-1.0;
            lb_wheel_vel= 1.0;
            rb_wheel_vel=-1.0;

            auto lf_cart_force = robot->lf_leg_calc->foot_force(robot->lf_joint_pos, robot->lf_joint_torque, robot->lf_forward_torque);
            if (lf_cart_force[0] > 10.0){

                lf_wheel_vel= 0.0;
                rf_wheel_vel= 0.0;
                lb_wheel_vel= 0.0;
                rb_wheel_vel= 0.0;

                cross_wall_stage=0;
            }
        }
        if (cross_wall_stage == 0){        //设置腿长调节姿态
            enable_posture_safe=false;
            
            wall_lf_foot_pos=robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            wall_rf_foot_pos=robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            wall_lb_foot_pos=robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            wall_rb_foot_pos=robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.0,0.0,0.0),0.3);
            rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.0,0.0,0.0),0.3);
            lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0),0.3);
            rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,0.0),0.3);
            
            cross_wall_stage_time=robot->node_->get_clock()->now();
            cross_wall_stage=1;     //无条件跳转到状态1
        }
        if (cross_wall_stage == 1){         // 执行设置的腿长，调整质心位置，使其落在支撑三角形内
            
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            if(!success)
            {   
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                lf_wheel_vel=0.0;
                rf_wheel_vel=0.0;
                lb_wheel_vel=0.0;
                rb_wheel_vel=0.0;

                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.0,0.0,-0.1),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,-0.1),2.0);
                
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=2;     //轨迹执行完后跳转到状态2
            }
        }
        if (cross_wall_stage == 2){
            
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            lf_foot_exp_pos=wall_lf_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;

            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.2,0.2,0.24),4.0);
                
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=3;
            }
        }
        if (cross_wall_stage == 3) {
           
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            rf_foot_exp_pos=wall_rf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;
            if(!success)
            {   
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.35,0.0,0.10),4.0);
                
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=4;
            }
        }
        if(cross_wall_stage==4){
        
           
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            rf_foot_exp_pos=wall_rf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.40,0.0,0.10),2.0);
                
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=5;
            }
        }
        //右前腿规划：5-9
       if(cross_wall_stage==5){
            
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            rf_foot_exp_pos=wall_rf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,-0.10),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,-0.07),2.0);
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=6;
            }
        }
       if(cross_wall_stage==6){
            
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            rf_foot_exp_pos=wall_rf_foot_pos;
            lf_foot_exp_pos=wall_lf_foot_pos;
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(-0.20,-0.20,0.30),3.0);

                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=7;
            }
       }
       if(cross_wall_stage==7){
            
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            lf_foot_exp_pos=wall_lf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.10,-0.20,0.29),3.0);

                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=8;
            }
       }
        if(cross_wall_stage==8){

            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            lf_foot_exp_pos=wall_lf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;
            if(!success)
            {   
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.38,-0.10,0.30),3.0);
                
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=9;     
            }
        }
        if(cross_wall_stage==9){

            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            lf_foot_exp_pos=wall_lf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;
            if(!success)
            {   
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.38,0.0,0.30),3.0);
                
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=10;     
            }
        }
        if(cross_wall_stage==10)
        {
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            
            lf_foot_exp_pos=wall_lf_foot_pos;
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;
            if(!success)
            {   
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.30,0.0,0.29),2.0);
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.30,0.0,0.29),2.0);
                
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=11;     
            }
        }
        if(cross_wall_stage==11)
        {
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;
            if(!success)
            {   
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.25,0.0,0.0),0.65);
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.25,0.0,0.0),0.65);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,-0.12),0.65);
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=12;     
            }
        }
        if(cross_wall_stage==12)
        {
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            rb_foot_exp_pos=wall_rb_foot_pos;
            if(!success)
            {   
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.25,0.0,0.0),0.8);
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.25,0.0,0.0),0.8);

                lb_wheel_vel=  1.0;
                rb_wheel_vel= -1.0;

                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=13;     
            }
        }
        if(cross_wall_stage==13)
        {
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;
            if(!success)
            {   
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.25,0.0,-0.07),0.1);
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.25,0.0,-0.07),0.1);

                lb_wheel_vel= 0.0;
                rb_wheel_vel= 0.0;

                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=14;     
            }
        }
        if(cross_wall_stage==14)
        {
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();

            lf_foot_exp_pos=wall_lf_foot_pos;

            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(-0.10,0.0,-0.1),2.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.1,0.0,0.1),2.0);
                
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=15;
            }
        }
        if(cross_wall_stage==15)
        {
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();

            lf_foot_exp_pos=wall_lf_foot_pos;
            rf_foot_exp_pos=wall_rf_foot_pos;
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.1,-0.3,0.32),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.1,0.3,0.32),2.0);
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=16;
            }
        }
        if(cross_wall_stage==16)
        {
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            lf_foot_exp_pos=wall_lf_foot_pos;
            rf_foot_exp_pos=wall_rf_foot_pos;
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            //lb_foot_exp_pos=wall_lb_foot_pos;
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.10,0.0,-0.07),2.0);
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.10,0.0,-0.07),2.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.15,-0.15,0.32),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.15,0.15,0.32),2.0);
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=17;
            }
        }
        if(cross_wall_stage==17)
        {
          bool success=false;
          double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
          std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
          std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
          std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
          std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                //lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.35,0.0,0.28),2.0);
                //rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.35,0.0,0.28),2.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.34,-0.12,0.35),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.34,0.12,0.35),2.0);
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=18;
            }
        }
        if(cross_wall_stage==18)
        {
          bool success=false;
          double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();

          std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
          std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
          std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
          std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                //lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.35,0.0,0.18),2.0);
                //rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.35,0.0,0.18),2.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.35,-0.06,0.30),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.35,0.06,0.30),2.0);
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=19;

            }
        }
        if(cross_wall_stage==19)
        {
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            lf_foot_exp_pos=wall_lf_foot_pos;
            rf_foot_exp_pos=wall_rf_foot_pos;
            if(!success)
            {

                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.00,0.0,0.00),2.0);
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.00,0.0,0.00),2.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(-0.22,0.0,-0.05),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(-0.22,0.0,-0.05),2.0);
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=20;
            }
        }
        if(cross_wall_stage==20)
        {
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            //lb_foot_exp_pos=wall_lb_foot_pos;
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,0.0),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0),2.0);
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=21;
            }
        }
        if(cross_wall_stage==21)
        {
             bool success=false;
             double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);

            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                RCLCPP_INFO(robot->node_->get_logger(),"HELLO");
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,0.0),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0),2.0);
                cross_wall_stage_time=robot->node_->get_clock()->now();
                cross_wall_stage=21;
                return "stop";
            }
        }

        robot_interfaces::msg::Robot joints_target;
       
        joints_target.legs[0] =
            robot->signal_leg_calc(lf_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->lf_leg_calc, &lf_forward_torque,lf_wheel_vel,lf_wheel_force);
        joints_target.legs[1] =
            robot->signal_leg_calc(rf_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->rf_leg_calc, &rf_forward_torque,rf_wheel_vel,rf_wheel_force);
        joints_target.legs[2] =
            robot->signal_leg_calc(lb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->lb_leg_calc, &lb_forward_torque,lb_wheel_vel,lb_wheel_force);
        joints_target.legs[3] =
            robot->signal_leg_calc(rb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->rb_leg_calc, &rb_forward_torque,rb_wheel_vel,rb_wheel_force);

        robot->legs_target_pub->publish(joints_target);

        return "cross_wall";
    }










