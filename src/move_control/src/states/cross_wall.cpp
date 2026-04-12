#include "states/cross_wall.hpp"
#include "core/robot.hpp"
#include "leg/step.hpp"
#include <rcl/timer.h>
#include <rclcpp/logging.hpp>
#include <math.h>


        bool use_limit_lf = false;
        bool use_limit_rf = false;
        bool use_limit_lb = false;
        bool use_limit_rb = false;
Cross_WallState::Cross_WallState(Robot* robot)
    : BaseState<Robot>("cross_wall") {
        robot->node_->declare_parameter<int>("cross_wall_stage",-1);
        robot->node_->declare_parameter("cross_k_F",1.0);


        robot->add_param_cb([this](const rclcpp::Parameter& param){
            auto name=param.get_name();
            if(name=="cross_wall_stage")
            {
                cross_wall_stage=param.as_int();
                change_flag = true;
            }
            if(name=="cross_k_F")
            {
                k_F=(float)param.as_double();
            }
            return true;
        });

        mass = (robot->robot_lf_grivate + robot->robot_rf_grivate + robot->robot_lb_grivate + robot->robot_rb_grivate) / 9.8;
        mass_center_pos =
        Vector2D(
            robot->robot_lf_grivate * robot->lf_leg_calc->pos_offset[0] + robot->robot_rf_grivate * robot->rf_leg_calc->pos_offset[0]
                + robot->robot_lb_grivate * robot->lb_leg_calc->pos_offset[0] + robot->robot_rb_grivate * robot->rb_leg_calc->pos_offset[0],
            robot->robot_lf_grivate * robot->lf_leg_calc->pos_offset[1] + robot->robot_rf_grivate * robot->rf_leg_calc->pos_offset[1]
                + robot->robot_lb_grivate * robot->lb_leg_calc->pos_offset[1] + robot->robot_rb_grivate * robot->rb_leg_calc->pos_offset[1])
        / (mass * 9.8);

}

bool Cross_WallState::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    cross_wall_stage=-1;
    change_flag = false;
    cross_wall_stage_time = robot->node_->get_clock()->now();
    return true;
}

std::string Cross_WallState::update(Robot* robot){

        robot_interfaces::msg::RobotTarget joints_target;

        static int last_stage = -1;
        
        // if (cross_wall_stage == -1)
        // {
        //     last_stage = -1;
        //     double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
        //     if(time > 0.5)
        //         cross_wall_stage = 0;
        // }

        static int cnt = 0;
        cnt++;
        if(cnt>=250)
        {
            cnt = 0;
            RCLCPP_ERROR(robot->node_->get_logger(),
                "\033[31mchange_flag = %d, cross_wall_stage = %d\033[0m",
                        change_flag, cross_wall_stage);
        }

        lf_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_lf_grivate);
        rf_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_rf_grivate);
        lb_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_lb_grivate);
        rb_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_rb_grivate);

        if(cross_wall_stage == 0  && change_flag == true){

            use_limit_lf = false;
            use_limit_lb = false;
            use_limit_rb = false;
            use_limit_rf = false;
                 
            wall_lf_foot_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            wall_rf_foot_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            wall_lb_foot_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            wall_rb_foot_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_support_trajectory(wall_lf_foot_pos, Vector3D(0.0,0.08,0.0), 1.0);
            rf_leg_step.update_support_trajectory(wall_rf_foot_pos, Vector3D(0.0,0.08,0.0), 1.0);
            lb_leg_step.update_support_trajectory(wall_lb_foot_pos, Vector3D(0.0,0.08,0.0), 1.0);
            rb_leg_step.update_support_trajectory(wall_rb_foot_pos, Vector3D(0.0,0.08,0.0), 1.0);
            //change_flag=false;
            cross_wall_stage = 1;
                
        }
        else if (cross_wall_stage == 1 && change_flag == true){        
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }

            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;

            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            
            auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);   //在简化的二维平面模型中，提供支撑力的位置
            auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
            
            // 使用Eigen求解每条腿的支撑力
            // 构建方程组: A * f = b
            // 约束条件:
            // 1. 垂直力平衡: f_lf + f_rf + f_lb + f_rb = mg
            // 2. 绕质心的力矩平衡(x方向): (lf_pos.x - mass_center_pos.x)*f_lf + ... = 0
            // 3. 绕质心的力矩平衡(y方向): (lf_pos.y - mass_center_pos.y)*f_lf + ... = 0
            Eigen::Matrix<double, 3, 4> A;
            Eigen::Vector3d b;
            
            // 第一行: 力平衡约束
            A(0, 0) = 1.0;  // lf
            A(0, 1) = 1.0;  // rf
            A(0, 2) = 1.0;  // lb
            A(0, 3) = 1.0;  // rb
            b(0) = mass * 9.8;
            
            // 第二行: 绕质心的x方向力矩平衡
            A(1, 0) = lf_pos.x() - mass_center_pos.x();
            A(1, 1) = rf_pos.x() - mass_center_pos.x();
            A(1, 2) = lb_pos.x() - mass_center_pos.x();
            A(1, 3) = rb_pos.x() - mass_center_pos.x();
            b(1) = 0.0;
            
            // 第三行: 绕质心的y方向力矩平衡
            A(2, 0) = lf_pos.y() - mass_center_pos.y();
            A(2, 1) = rf_pos.y() - mass_center_pos.y();
            A(2, 2) = lb_pos.y() - mass_center_pos.y();
            A(2, 3) = rb_pos.y() - mass_center_pos.y();
            b(2) = 0.0;
            
            // 使用最小二乘法求解超定方程组
            Eigen::Vector4d forces = A.transpose() * (A * A.transpose()).inverse() * b;
            
            // 将求解的垂直力存入各腿的期望力向量（z分量）
            lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
            rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
            lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));
            rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(3));
            
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,wall_lf_foot_pos,1.0);
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,wall_rf_foot_pos,1.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,wall_rb_foot_pos,1.0);
                lb_leg_step.update_flight_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0), Vector3D(0.12,0.08,0.0), Vector2D(0.0,0.0), 1.0,0.06);
                //change_flag=false;
                cross_wall_stage=2;     
            }
        }//1:其它保持不变,迈左后腿
        else if (cross_wall_stage == 2 && change_flag == true){         // 执行设置的腿长，调整质心位置，使其落在支撑三角形内

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            bool success=false;
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            
            // 3足支撑力计算 (lf, rf, rb支撑，lb摆动)
            auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);
            auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
            auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
            
            Eigen::Matrix3d A;
            Eigen::Vector3d b;
            
            // 力平衡约束
            A(0, 0) = 1.0;  // lf
            A(0, 1) = 1.0;  // rf
            A(0, 2) = 1.0;  // rb
            b(0) = mass * 9.8;
            
            // 绕质心的x方向力矩平衡
            A(1, 0) = lf_pos.x() - mass_center_pos.x();
            A(1, 1) = rf_pos.x() - mass_center_pos.x();
            A(1, 2) = rb_pos.x() - mass_center_pos.x();
            b(1) = 0.0;
            
            // 绕质心的y方向力矩平衡
            A(2, 0) = lf_pos.y() - mass_center_pos.y();
            A(2, 1) = rf_pos.y() - mass_center_pos.y();
            A(2, 2) = rb_pos.y() - mass_center_pos.y();
            b(2) = 0.0;
            
            // 求解3x3方程组
            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);
            
            lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
            rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
            lb_foot_exp_force = Vector3D::Zero();  // 摆动腿无支撑力
            rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));

            if(!success)
            {   
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_step.update_support_trajectory(robot->lf_joint_pos,Vector3D(1.44,0.822,-0.165),2.0);
                
                //change_flag=false;
                cross_wall_stage=3;    
            }
        }
        else if (cross_wall_stage == 3 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success); 
            rf_foot_exp_pos=wall_rf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;

            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 摆动腿

            if(!success)
            {
               
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_step.update_support_trajectory(Vector3D(1.44,0.822,-0.165),Vector3D(1.44,-0.92,0.48),2.0);
                
                // change_flag=false;
                cross_wall_stage=4;
            }
        }
        else if (cross_wall_stage == 4 && change_flag == true) {

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            

            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 摆动腿

            if(!success)
            {   

                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.0,0.0,0.0),1.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0),1.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,0.0),1.0);

                //change_flag=false;
                cross_wall_stage=5;
            }
        }
        else if(cross_wall_stage==5 && change_flag == true){

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            
            
            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 墙上腿
            
            if(!success)
            {

                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_step.update_support_trajectory(Vector3D(1.44,-0.92,0.48),Vector3D(-0.12,-1.00,0.96),1.0);
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,wall_rf_foot_pos,1.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,wall_lb_foot_pos,1.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,wall_rb_foot_pos,1.0);
                // change_flag=false;
                cross_wall_stage=6;
            }
        }
        //右前腿规划：6-9
        else if(cross_wall_stage == 6 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();  
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);

            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 墙上腿

            if(!success)
            {
                int result;
                robot->rf_leg_calc->set_init_joint_pos(Vector3D(-0.13,-0.42,-0.536));
                rf_joint_exp_pos_ = robot->rf_leg_calc->joint_pos(rf_foot_exp_pos,&result);
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_step.update_support_trajectory(rf_joint_exp_pos_,Vector3D(-1.35,-0.827,-0.531),1.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,-0.03),1.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,-0.07),1.0);
                //change_flag=false;
                cross_wall_stage=7;
            }
        }
        else if(cross_wall_stage == 7 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();

            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            
            if(!success)
            {
                wall_lf_foot_pos=lf_joint_exp_pos_;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_step.update_support_trajectory(Vector3D(-1.35,-0.827,-0.531),Vector3D(-1.5,1.08,-0.531),2.0);

                //change_flag=false;
                cross_wall_stage=8;
            }
       }
        else if(cross_wall_stage == 8 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            lf_joint_exp_pos_=wall_lf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;

            if(!success)
            {
                wall_lf_foot_pos=lf_joint_exp_pos_;
                wall_rf_foot_pos=rf_joint_exp_pos_;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_step.update_support_trajectory(Vector3D(-1.5,1.08,-0.531),Vector3D(-0.60,1.25,-0.531),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,-0.03),2.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,-0.03),2.0);
                
                //change_flag=false;
                cross_wall_stage=9;
            }
        }
        else if(cross_wall_stage == 9 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success = false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            lf_joint_exp_pos_=wall_lf_foot_pos;
           
            if(!success)
            {
                wall_rf_foot_pos = rf_joint_exp_pos_;
                wall_lf_foot_pos = lf_joint_exp_pos_;
                wall_lb_foot_pos = lb_foot_exp_pos;
                wall_rb_foot_pos = rb_foot_exp_pos;

                lf_step.update_support_trajectory(Vector3D(0.219,-1.19,0.488),Vector3D( 0.0267, 2.1,-0.2),2.0); 
                rf_step.update_support_trajectory(Vector3D(-0.60,1.25,-0.531),Vector3D(-0.0267,-2.1, 0.2),2.0); 
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,-0.06),2.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,-0.06),2.0);

                //change_flag=false;               
                cross_wall_stage=10;     
            }
        }
        else if(cross_wall_stage == 10 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);

            if(!success)
            {    
                wall_lb_foot_pos = lb_foot_exp_pos;
                wall_rb_foot_pos = rb_foot_exp_pos;
                wall_lf_foot_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
                wall_rf_foot_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
                
                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.0,0.0,0.05),2.0); 
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.0,0.0,0.05),2.0); 
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(-0.15,0.0,-0.075),2.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(-0.15,0.0,-0.075),2.0);
                cross_wall_stage = 11;
                change_flag=true;
            }
        }
        else if(cross_wall_stage == 11 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
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

                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.0,0.0,0.23),2.0); 
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.0,0.0,0.23),2.0); 
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(-0.10,-0.21,0.40),1.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(-0.10,0.21,0.40),1.0);

                // change_flag=false;
                cross_wall_stage=12;
            }
        }
        else if(cross_wall_stage == 12 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();

            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
           
            if(!success)
            {   
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.18,-0.08,0.4),1.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.18, 0.08,0.4),1.0);

                //change_flag=false;
                cross_wall_stage=13;
            }
        }
        else if(cross_wall_stage == 13 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
           
            if(!success)
            {

                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;


                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(-0.2,0.0,-0.05),1.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(-0.2,0.0,-0.05),1.0);

                //change_flag=false;
                cross_wall_stage=14;
            }
        }
        else if(cross_wall_stage == 14 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
           
            if(!success)
            { 
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,0.0),1.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0),1.0);
              
                //change_flag=false;
                cross_wall_stage=15;
            }
        }
        else if(cross_wall_stage==15 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);

            if(!success)
            {
               
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,0.0),0.5);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0),0.5);

                change_flag=false;
                auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
                auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
                auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
                auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
                robot->lf_z_vmc->reset(lf_cart_pos.z(), 0.0);
                robot->rf_z_vmc->reset(rf_cart_pos.z(), 0.0);
                robot->lb_z_vmc->reset(lb_cart_pos.z(), 0.0);
                robot->rb_z_vmc->reset(rb_cart_pos.z(), 0.0);

                return "stop";
            }
        }



/*******************************lf**********************************/
        if(use_limit_lf)
        {

            for(int i=0;i<3;i++)
            {
                joints_target.legs[0].joints[i].kp   = (float)robot->lf_leg_calc->kp[i];
                joints_target.legs[0].joints[i].kd   = (float)robot->lf_leg_calc->kd[i];
                joints_target.legs[0].joints[i].rad    = (float)lf_joint_exp_pos_[i];
            }
            robot->lf_leg_calc->joint_pos_setarray(robot->lf_joint_pos);
            lf_foot_exp_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            // static int cnt_ = 0;
            // cnt_++;
            // if(cnt_>=1)
            // {
            //     cnt_ = 0;
            //     RCLCPP_ERROR(robot->node_->get_logger(),
            //         "\033[31mlf_joint_pos = (%.2f, %.2f %.2f), lf_joint_exp_pos_ = (%.2f, %.2f, %.2f)\033[0m",
            //                 robot->lf_joint_pos[0], robot->lf_joint_pos[1],robot->lf_joint_pos[2],lf_joint_exp_pos_[0],lf_joint_exp_pos_[1],lf_joint_exp_pos_[2]);
            // }   
        }
        else
        {
            joints_target.legs[0] =
                robot->lf_leg_calc->signal_leg_calc(lf_foot_exp_pos, lf_foot_exp_vel, 
                    lf_foot_exp_acc, lf_foot_exp_force,  
                    &robot->lf_forward_torque,lf_wheel_vel,lf_wheel_force);
        }

/*******************************rf**********************************/
        if(use_limit_rf)
        {
            for(int i=0;i<3;i++)
            {
            joints_target.legs[1].joints[i].rad =(float)rf_joint_exp_pos_[i];
            joints_target.legs[1].joints[i].kp = (float)robot->rf_leg_calc->kp[i];
            joints_target.legs[1].joints[i].kd = (float)robot->rf_leg_calc->kd[i];
            }
            robot->rf_leg_calc->joint_pos_setarray(robot->rf_joint_pos);
            rf_foot_exp_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        }
        else
        {
        joints_target.legs[1] =
            robot->rf_leg_calc->signal_leg_calc(rf_foot_exp_pos, rf_foot_exp_vel, 
            rf_foot_exp_acc, rf_foot_exp_force,  
            &robot->rf_forward_torque,rf_wheel_vel,rf_wheel_force);
        }
/*******************************lb**********************************/
        if(use_limit_lb)
        {
            for(int i=0;i<3;i++)
            {
            joints_target.legs[2].joints[i].kp = (float)robot->lb_leg_calc->kp[i];
            joints_target.legs[2].joints[i].kd = (float)robot->lb_leg_calc->kd[i];
            joints_target.legs[2].joints[i].rad =(float)lb_joint_exp_pos_[i];
            }
            robot->lb_leg_calc->joint_pos_setarray(robot->lb_joint_pos);
            lb_foot_exp_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        }
        else
        {
            joints_target.legs[2] =
            robot->lb_leg_calc->signal_leg_calc(lb_foot_exp_pos, lb_foot_exp_vel, 
            lb_foot_exp_acc, lb_foot_exp_force,  
            &robot->lb_forward_torque,lb_wheel_vel,lb_wheel_force);
        }
/*******************************rb**********************************/
        if(use_limit_rb)
        {
            for(int i=0;i<3;i++)
            {
            joints_target.legs[3].joints[i].kp = (float)robot->rb_leg_calc->kp[i];
            joints_target.legs[3].joints[i].kd = (float)robot->rb_leg_calc->kd[i];
            joints_target.legs[3].joints[i].rad =(float)rb_joint_exp_pos_[i];
            }
            robot->rb_leg_calc->joint_pos_setarray(robot->rb_joint_pos);
            rb_foot_exp_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
        }else
        {
            joints_target.legs[3] =
            robot->rb_leg_calc->signal_leg_calc(rb_foot_exp_pos, rb_foot_exp_vel, 
            rb_foot_exp_acc, rb_foot_exp_force,  
            &robot->rb_forward_torque,rb_wheel_vel,rb_wheel_force);
        }

        robot->legs_target_pub->publish(joints_target);

        return "cross_wall";
    }


void Cross_Step::update_support_trajectory(const Vector3D& cur_pos,
                                           const Vector3D final_pos,
                                           double time)
{
    T = time;

    for (int i = 0; i < 3; i++)
    {
        double p0 = cur_pos[i];
        double pT = final_pos[i];

        // ⭐ 平均速度（关键！避免前段冲）
        double v_avg = (pT - p0) / time;

        // ⭐ 给一点初速度（非常重要）
        double v0 = 0.3 * v_avg;
        double vT = 0.0;

        if (i == 0)
            set_cubic(lx.lx, p0, v0, pT, vT, time);
        else if (i == 1)
            set_cubic(ly.ly, p0, v0, pT, vT, time);
        else
            set_cubic(lz.lz, p0, v0, pT, vT, time);
    }
}

std::tuple<Vector3D, Vector3D, Vector3D>
Cross_Step::get_target(double time, bool &success)
{
    Vector3D pos, vel, acc;

    if (time >= T)
    {
        time = T;
        success = false;
    }
    else
    {
        success = true;
    }

    // ⭐ 计算三轴
    pos[0] = get_cubic_value(lx.lx, time);
    vel[0] = get_cubic_dt(lx.lx, time);
    acc[0] = get_cubic_dtdt(lx.lx, time);

    pos[1] = get_cubic_value(ly.ly, time);
    vel[1] = get_cubic_dt(ly.ly, time);
    acc[1] = get_cubic_dtdt(ly.ly, time);

    pos[2] = get_cubic_value(lz.lz, time);
    vel[2] = get_cubic_dt(lz.lz, time);
    acc[2] = get_cubic_dtdt(lz.lz, time);

    return {pos, vel, acc};
}





