#include "states/cross_wall.hpp"
#include "core/robot.hpp"
#include "leg/step.hpp"
#include <rcl/timer.h>
#include <rclcpp/logging.hpp>
#include <math.h>


Cross_WallState::Cross_WallState(Robot* robot)
    : BaseState<Robot>("cross_wall") {
        robot->node_->declare_parameter<int>("cross_wall_stage",-1);
        robot->node_->declare_parameter<int>("cross_k_F",1.0);


        robot->add_param_cb([this](const rclcpp::Parameter& param){
            auto name=param.get_name();
            if(name=="cross_wall_stage")
            {
                cross_wall_stage=param.as_int();
                change_flag = true;
            }
            else if(name=="cross_k_F")
            {
                k_F=(float)param.as_double();
            }
            return true;
        });

}

bool Cross_WallState::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    cross_wall_stage=-1;
    return true;
}

std::string Cross_WallState::update(Robot* robot){

        robot_interfaces::msg::Robot joints_target{};

 
        lf_wheel_force = 0.0f;
        static int last_stage = 1;

        if (cross_wall_stage == -1)
            last_stage = -1;

        static int cnt = 0;
        cnt++;
        if(cnt>=250)
        {
            cnt = 0;
            RCLCPP_ERROR(robot->node_->get_logger(),
                "\033[31mchange_flag = %d, cross_wall_stage = %d\033[0m",
                        change_flag, cross_wall_stage);
        }

        if(cross_wall_stage == 0  && change_flag == true){
             if(allow_vel == true)
             {
                lf_wheel_vel= 0.3;
                rf_wheel_vel=-0.3;
                lb_wheel_vel= 0.3;
                rb_wheel_vel=-0.3;

                double wheel_F = k_F * ((lf_wheel_vel / Robot::WHEEL_RADIUS) - (robot->lf_wheel_omega + robot->lb_wheel_omega - robot->rf_wheel_omega -robot->rb_wheel_omega) / 4.0f);
                lf_wheel_force = wheel_F;
                lb_wheel_force = wheel_F;
                rf_wheel_force = -wheel_F;
                rb_wheel_force = -wheel_F;

                allow_vel = false;
             }
            auto lf_cart_force = robot->lf_leg_calc->foot_force(robot->lf_joint_pos, robot->lf_joint_torque, robot->lf_forward_torque);
            auto rf_cart_force = robot->rf_leg_calc->foot_force(robot->rf_joint_pos, robot->rf_joint_torque, robot->rf_forward_torque);
            if (lf_cart_force[0] > 10.0 || rf_cart_force[0] > 10.0){

                lf_wheel_vel= 0.0;
                rf_wheel_vel= 0.0;
                lb_wheel_vel= 0.0;
                rb_wheel_vel= 0.0;
               
                lf_wheel_force = 0.0;
                lb_wheel_force = 0.0;
                rf_wheel_force = 0.0;
                rb_wheel_force = 0.0;
                
                change_flag=false;
                //cross_wall_stage=1;
            }
        }
        if (cross_wall_stage == 1 && change_flag == true){        
            
            wall_lf_foot_pos=robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            wall_rf_foot_pos=robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            wall_lb_foot_pos=robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            wall_rb_foot_pos=robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.0,0.0,0.0),0.3);
            rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.0,0.0,0.0),0.3);
            lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0),0.3);
            rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,0.0),0.3);
            
            change_flag=false;
            //cross_wall_stage=2;     
        }
        if (cross_wall_stage == 2 && change_flag == true){         // 执行设置的腿长，调整质心位置，使其落在支撑三角形内

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
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

                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.0,0.0,-0.1),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,-0.1),2.0);
                rb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,-0.03),2.0);
                // rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(cross_x_rf,cross_y_rf,cross_z_rf),time_s);
                // lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(cross_x_lb,cross_y_lb,cross_z_lb),time_s);
                
                change_flag=false;
                //cross_wall_stage=3;     //轨迹执行完后跳转到状态2
            }
        }
        if (cross_wall_stage == 3 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            lf_foot_exp_pos=wall_lf_foot_pos;

            if(!success)
            {
                // wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_leg_step.update_support_trajectory(robot->lf_joint_pos,Vector3D(1.2,0.822,-0.165),4.0);
                // lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(cross_x_lf,cross_y_lf,cross_z_lf),time_s);
                
                change_flag=false;
                //cross_wall_stage=4;
            }
        }
        if (cross_wall_stage == 4 && change_flag == true) {

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
           
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            rf_foot_exp_pos=wall_rf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;
            if(!success)
            {   
                // wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_leg_step.update_support_trajectory(Vector3D(1.2,0.822,-0.165),Vector3D(1.2,-0.92,0.48),4.0);
                // lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(cross_x_lf,cross_y_lf,cross_z_lf),time_s);

                change_flag=false;
                //cross_wall_stage=5;
            }
        }
        if(cross_wall_stage==5 && change_flag == true){

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
        
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            rf_foot_exp_pos=wall_rf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            if(!success)
            {
                // wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                lf_leg_step.update_support_trajectory(Vector3D(1.2,-0.92,0.48),Vector3D(-0.12,-1.00,0.96),2.0);
                // lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(cross_x_lf,cross_y_lf,cross_z_lf),time_s);

                change_flag=false;
                //cross_wall_stage=6;
            }
        }
        //右前腿规划：6-9
       if(cross_wall_stage == 6 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
        
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            rf_foot_exp_pos = wall_rf_foot_pos;
            lb_foot_exp_pos = wall_lb_foot_pos;
            rb_foot_exp_pos = wall_rb_foot_pos;
            if(!success)
            {
                int result;
                
                rf_joint_exp_pos_ = robot->rf_leg_calc->joint_pos(rf_foot_exp_pos,&result,Vector3D(-0.13,-0.42,-0.536));
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                // lf_leg_step.update_support_trajectory(Vector3D(0.12,-1.10,0.48),Vector3D(0.12,-0.80,0.48),2.0);
                rf_leg_step.update_support_trajectory(rf_joint_exp_pos_,Vector3D(-1.2,-0.827,-0.531),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,-0.03),2.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,-0.07),2.0);
                change_flag=false;
                //cross_wall_stage=7;
            }
        }
       if(cross_wall_stage == 7 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
               
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();

            // std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            
            if(!success)
            {
                wall_lf_foot_pos=lf_joint_exp_pos_;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_leg_step.update_support_trajectory(Vector3D(-1.2,-0.827,-0.531),Vector3D(-1.27,1.08,-0.531),3.0);

                change_flag=false;
                // cross_wall_stage=8;
            }
       }
       if(cross_wall_stage == 8 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            lf_joint_exp_pos_=wall_lf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;

            if(!success)
            {
                wall_lf_foot_pos=lf_joint_exp_pos_;
                wall_rf_foot_pos=rf_joint_exp_pos_;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_leg_step.update_support_trajectory(Vector3D(-1.27,1.08,-0.531),Vector3D(-0.60,1.25,-0.531),3.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,-0.03),3.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,-0.03),3.0);
                
                change_flag=false;
                //cross_wall_stage=9;
            }
       }
        if(cross_wall_stage == 9 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            lf_joint_exp_pos_=wall_lf_foot_pos;
           
            if(!success)
            {
                
                wall_rf_foot_pos = rf_joint_exp_pos_;
                wall_lf_foot_pos = lf_joint_exp_pos_;
                wall_lb_foot_pos = lb_foot_exp_pos;
                wall_rb_foot_pos = rb_foot_exp_pos;

                rf_leg_step.update_support_trajectory(Vector3D(-0.60,1.25,-0.531),Vector3D(-0.0267,-0.822,0.165),2.0); 
                lf_leg_step.update_support_trajectory(Vector3D(0.219,-1.19,0.488),Vector3D(0.0267,0.822,-0.165),2.0); 
               
                change_flag=false;               
                //cross_wall_stage=10;     
            }       
        }
        if(cross_wall_stage == 10 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            bool success=false;
            
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;

            if(!success)
            {    
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                change_flag=false;
            }
        }
        if(cross_wall_stage == 11 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();

            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;
            
            lf_wheel_vel= 0.4;
            rf_wheel_vel=-0.4;
            lb_wheel_vel= 0.4;
            rb_wheel_vel=-0.4;

            double wheel_F = k_F * ((lb_wheel_vel / Robot::WHEEL_RADIUS) - ( robot->lf_wheel_omega+robot->lb_wheel_omega - robot->rb_wheel_omega-robot->rf_wheel_omega) / 4.0f);
            lf_wheel_force =  wheel_F;
            rf_wheel_force = -wheel_F;
            lb_wheel_force =  wheel_F;   
            rb_wheel_force = -wheel_F;
            if(!success)
            { 
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(-0.10,-0.21,0.35),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(-0.10,0.21,0.35),2.0);

                change_flag=false;
                //cross_wall_stage=16;
            }
            // auto lb_cart_force = robot->lb_leg_calc->foot_force(robot->lb_joint_pos, robot->lb_joint_torque, robot->lb_forward_torque);
            // auto rb_cart_force = robot->rb_leg_calc->foot_force(robot->rb_joint_pos, robot->rb_joint_torque, robot->rb_forward_torque);
            // if (abs((int)(lb_cart_force[0])) > 16.0 || abs((int)(rb_cart_force[0])) > 16.0)
            // {
            //     RCLCPP_INFO(robot->node_->get_logger(),"HELLO");
            //     lf_wheel_vel= 0.0;
            //     rf_wheel_vel= 0.0;
            //     lb_wheel_vel= 0.0;
            //     rb_wheel_vel= 0.0;
                      
            //     lb_wheel_force = 0.0;
            //     rb_wheel_force = 0.0;
            //     lf_wheel_force =  0;
            //     rf_wheel_force =  0;
            //     change_flag=false;
            // }
            
        }
        if(cross_wall_stage == 12 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();

            lf_wheel_vel= 0.0;
            rf_wheel_vel= 0.0;
            lb_wheel_vel= 0.0;
            rb_wheel_vel= 0.0;

            lf_wheel_force = 0;
            rf_wheel_force = 0;
            lb_wheel_force = 0;   
            rb_wheel_force = 0;

            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
           
            if(!success)
            {
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.10,-0.1,0.35),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.10, 0.1,0.35),2.0);

                change_flag=false;
                //cross_wall_stage=17;
            }
        }
        if(cross_wall_stage == 13 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            if(!success)
            {
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.1,-0.15,0.30),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.1,0.15,0.30),2.0);

                change_flag=false;
                //cross_wall_stage=18;
            }
        }
        if(cross_wall_stage == 14 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
           
            if(!success)
            {

                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;


                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(-0.22,0.0,-0.05),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(-0.22,0.0,-0.05),2.0);

                change_flag=false;
                //cross_wall_stage=20;
            }
        }
        if(cross_wall_stage == 15 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
           
            if(!success)
            { 
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,0.0),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0),2.0);
              
                change_flag=false;
                //cross_wall_stage=21;
            }
        }
        if(cross_wall_stage==16 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);

            if(!success)
            {
               
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,0.0),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0),2.0);

              
                //cross_wall_stage=21;
                change_flag=false;
                return "stop";
            }
        }
        if(cross_wall_stage == 4|| cross_wall_stage == 5 || cross_wall_stage == 6)
        {
            lf_joint_omega  = robot->lf_leg_calc->joint_vel(lf_joint_exp_pos_, Vector3D{0,0,0});
            lf_joint_torque = robot->lf_leg_calc->joint_torque_foot_force(lf_joint_exp_pos_, Vector3D{0,0,0});
            lf_joint_torque +=robot->lf_leg_calc->joint_torque_dynamic(lf_joint_exp_pos_, lf_joint_omega, Vector3D{0,0,0});

            for (int i = 0; i < 3; i++) {
            joints_target.legs[0].joints[i].rad    = static_cast<float>(lf_joint_exp_pos_[i]);
            joints_target.legs[0].joints[i].omega  = static_cast<float>(lf_joint_omega[i]);
            joints_target.legs[0].joints[i].torque = static_cast<float>(lf_joint_torque[i]);
            }
            joints_target.legs[1] =
                robot->signal_leg_calc(rf_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->rf_leg_calc, &rf_forward_torque,rf_wheel_vel,rf_wheel_force);
            joints_target.legs[2] =
                robot->signal_leg_calc(lb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->lb_leg_calc, &lb_forward_torque,lb_wheel_vel,lb_wheel_force);
            joints_target.legs[3] =
                 robot->signal_leg_calc(rb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->rb_leg_calc, &rb_forward_torque,rb_wheel_vel,rb_wheel_force);
        }
        else if(cross_wall_stage == 7 || cross_wall_stage == 8 || cross_wall_stage == 9 || cross_wall_stage == 10 || cross_wall_stage == 11)
        {
            lf_joint_omega  = robot->lf_leg_calc->joint_vel(lf_joint_exp_pos_, Vector3D{0,0,0});
            lf_joint_torque = robot->lf_leg_calc->joint_torque_foot_force(lf_joint_exp_pos_, Vector3D{0,0,0});
            lf_joint_torque +=robot->lf_leg_calc->joint_torque_dynamic(lf_joint_exp_pos_, lf_joint_omega, Vector3D{0,0,0});

            rf_joint_omega  = robot->rf_leg_calc->joint_vel(rf_joint_exp_pos_, Vector3D{0,0,0});
            rf_joint_torque = robot->rf_leg_calc->joint_torque_foot_force(rf_joint_exp_pos_, Vector3D{0,0,0});
            rf_joint_torque +=robot->rf_leg_calc->joint_torque_dynamic(rf_joint_exp_pos_, rf_joint_omega, Vector3D{0,0,0});

            for (int i = 0; i < 3; i++) {
            joints_target.legs[0].joints[i].rad    = static_cast<float>(lf_joint_exp_pos_[i]);
            joints_target.legs[0].joints[i].omega  = static_cast<float>(lf_joint_omega[i]);
            joints_target.legs[0].joints[i].torque = static_cast<float>(lf_joint_torque[i]);
            }

            for (int i = 0; i < 3; i++) {
            joints_target.legs[1].joints[i].rad    = static_cast<float>(rf_joint_exp_pos_[i]);
            joints_target.legs[1].joints[i].omega  = static_cast<float>(rf_joint_omega[i]);
            joints_target.legs[1].joints[i].torque = static_cast<float>(rf_joint_torque[i]);
            }

            joints_target.legs[2] =
                robot->signal_leg_calc(lb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->lb_leg_calc, &lb_forward_torque,lb_wheel_vel,lb_wheel_force);
            joints_target.legs[3] =
                robot->signal_leg_calc(rb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->rb_leg_calc, &rb_forward_torque,rb_wheel_vel,rb_wheel_force);
        }
        else if(cross_wall_stage == 12 || cross_wall_stage == 13 || cross_wall_stage == 14 || cross_wall_stage == 15 || cross_wall_stage == 16)
        {
            lf_joint_omega  = robot->lf_leg_calc->joint_vel(lf_joint_exp_pos_, Vector3D{0,0,0});
            lf_joint_torque = robot->lf_leg_calc->joint_torque_foot_force(lf_joint_exp_pos_, Vector3D{0,0,0});
            lf_joint_torque +=robot->lf_leg_calc->joint_torque_dynamic(lf_joint_exp_pos_, lf_joint_omega, Vector3D{0,0,0});

            rf_joint_omega  = robot->rf_leg_calc->joint_vel(rf_joint_exp_pos_, Vector3D{0,0,0});
            rf_joint_torque = robot->rf_leg_calc->joint_torque_foot_force(rf_joint_exp_pos_, Vector3D{0,0,0});
            rf_joint_torque +=robot->rf_leg_calc->joint_torque_dynamic(rf_joint_exp_pos_, rf_joint_omega, Vector3D{0,0,0});

            for (int i = 0; i < 3; i++) {
            joints_target.legs[0].joints[i].rad    = static_cast<float>(lf_joint_exp_pos_[i]);
            joints_target.legs[0].joints[i].omega  = static_cast<float>(lf_joint_omega[i]);
            joints_target.legs[0].joints[i].torque = static_cast<float>(lf_joint_torque[i]);
            }

            for (int i = 0; i < 3; i++) {
            joints_target.legs[1].joints[i].rad    = static_cast<float>(rf_joint_exp_pos_[i]);
            joints_target.legs[1].joints[i].omega  = static_cast<float>(rf_joint_omega[i]);
            joints_target.legs[1].joints[i].torque = static_cast<float>(rf_joint_torque[i]);
            }

            joints_target.legs[2] =
                robot->signal_leg_calc(lb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->lb_leg_calc, &lb_forward_torque,lb_wheel_vel,lb_wheel_force);
            joints_target.legs[3] =
                robot->signal_leg_calc(rb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->rb_leg_calc, &rb_forward_torque,rb_wheel_vel,rb_wheel_force);
        }
        else
        {
            joints_target.legs[0] =
                robot->signal_leg_calc(lf_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->lf_leg_calc, &lf_forward_torque,lf_wheel_vel,lf_wheel_force);
            joints_target.legs[1] =
                robot->signal_leg_calc(rf_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->rf_leg_calc, &rf_forward_torque,rf_wheel_vel,rf_wheel_force);
            joints_target.legs[2] =
                robot->signal_leg_calc(lb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->lb_leg_calc, &lb_forward_torque,lb_wheel_vel,lb_wheel_force);
            joints_target.legs[3] =
                robot->signal_leg_calc(rb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), robot->rb_leg_calc, &rb_forward_torque,rb_wheel_vel,rb_wheel_force);
        }
        robot->legs_target_pub->publish(joints_target);

        return "cross_wall";
    }









