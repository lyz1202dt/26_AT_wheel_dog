#include "states/cross_wall.hpp"
#include "core/robot.hpp"
#include "leg/step.hpp"
#include <rcl/timer.h>
#include <rclcpp/logging.hpp>

using Vec3 = Eigen::Vector3d;
trajectory::Bezier<Vec3> lf_curve;
Cross_WallState::Cross_WallState(Robot* robot)
    : BaseState<Robot>("cross_wall") {
        robot->node_->declare_parameter<int>("cross_wall_stage",-1);
        robot->node_->declare_parameter<int>("cross_k_F",1.0);
        // robot->node_->declare_parameter("cross_lf_x",0.0);
        // robot->node_->declare_parameter("cross_lf_y",0.0);
        // robot->node_->declare_parameter("cross_lf_z",0.0);
        // robot->node_->declare_parameter("cross_rb_x",0.0);
        // robot->node_->declare_parameter("cross_rb_y",0.0);
        // robot->node_->declare_parameter("cross_rb_z",0.0);
        // robot->node_->declare_parameter("cross_lb_x",0.0);
        // robot->node_->declare_parameter("cross_lb_y",0.0);
        // robot->node_->declare_parameter("cross_lb_z",0.0);
        // robot->node_->declare_parameter("cross_rf_x",0.0);
        // robot->node_->declare_parameter("cross_rf_y",0.0);
        // robot->node_->declare_parameter("cross_rf_z",0.0);
        // robot->node_->declare_parameter("cross_time",1.0);

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
            // else if(name=="cross_lf_x")
            // {
            //     cross_x_lf=param.as_double();
            // }
            // else if(name=="cross_lf_y")
            // {
            //     cross_y_lf=param.as_double();
            // }
            // else if(name=="cross_lf_z")
            // {
            //     cross_z_lf=param.as_double();
            // }
            // else if(name=="cross_rb_x")
            // {
            //     cross_x_rb=param.as_double();
            // }
            // else if(name=="cross_rb_y")
            // {
            //     cross_y_rb=param.as_double();
            // }
            // else if(name=="cross_rb_z")
            // {
            //     cross_z_rb=param.as_double();
            // }
            // else if(name=="cross_lb_x")
            // {
            //     cross_x_lb=param.as_double();
            // }
            // else if(name=="cross_lb_y")
            // {
            //     cross_y_lb=param.as_double();
            // }
            // else if(name=="cross_lb_z")
            // {
            //     cross_z_lb=param.as_double();
            // }
            // else if(name=="cross_rf_x")
            // {
            //     cross_x_rf=param.as_double();
            // }
            // else if(name=="cross_rf_y")
            // {
            //     cross_y_rf=param.as_double();
            // }
            // else if(name=="cross_rf_z")
            // {
            //     cross_z_rf=param.as_double();
            // }
            // else if(name=="cross_time")
            // {
            //     time_s=param.as_double();
            // }
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
    cross_wall_stage=0;
    return true;
}

/*
*cross_wall_stage_0:四轮转动，并做碰撞检测
*cross_wall_stage_1:获取当前四足的笛卡尔空间位置，并规划四足至足端中性点
*cross_wall_stage_2:右前腿_左后腿z轴为-0.1作为支撑
*
*
*
*
*
*
*/

std::string Cross_WallState::update(Robot* robot){
        robot_interfaces::msg::RobotTarget joints_target;

        static int last_stage = -1;

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


        auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        lf_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_lf_grivate);
        rf_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_rf_grivate);
        lb_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_lb_grivate);
        rb_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_rb_grivate);

        if(cross_wall_stage == 0  && change_flag==true){

                lf_wheel_vel= 0.3;
                rf_wheel_vel=-0.3;
                lb_wheel_vel= 0.3;
                rb_wheel_vel=-0.3;

                double wheel_F = k_F * ((lf_wheel_vel / Robot::WHEEL_RADIUS) - (robot->lf_wheel_omega + robot->lb_wheel_omega - robot->rf_wheel_omega -robot->rb_wheel_omega) / 4.0f);
                lf_wheel_force = wheel_F;
                lb_wheel_force = wheel_F;
                rf_wheel_force = -wheel_F;
                rb_wheel_force = -wheel_F;

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
                //cross_wall_stage=0;
            }
        }

        if (cross_wall_stage == 1 && change_flag == true){        
            

            lf_leg_step.update_support_trajectory(lf_cart_pos, Vector3D(0.0, -y_target, 0.0), 2.0);
            rf_leg_step.update_support_trajectory(rf_cart_pos, Vector3D(0.0, -y_target, 0.0), 2.0);
            lb_leg_step.update_support_trajectory(lb_cart_pos, Vector3D(0.0, -y_target, 0.0), 2.0);
            rb_leg_step.update_support_trajectory(rb_cart_pos, Vector3D(0.0, -y_target, 0.0), 2.0);

            
            change_flag=false;
            //cross_wall_stage=1;     
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

            Vec3 lf_start_pos = lf_cart_pos;// 起点
            Vec3 lf_peak_pos = lf_start_pos + Vec3(-0.02, 0.1, 0.15);  // P1: 抬高离墙
            Vec3 lf_end_pos  = lf_start_pos + Vec3(0.02, 0.1, 0.24);// P2: 终点

            std::vector<Vec3> control_points = {lf_start_pos, lf_peak_pos, lf_end_pos};
            lf_curve.setControlPoints(control_points);
                
                change_flag=false;
                //cross_wall_stage=2;     //轨迹执行完后跳转到状态2
            }
        }
        if (cross_wall_stage == 3 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }

            double duration = 2.0; // 轨迹总时间2秒
            double t_now = (robot->node_->get_clock()->now() - cross_wall_stage_time).seconds();
            double s = std::min(t_now / duration, 1.0); // 归一化 t ∈ [0,1]

            lf_foot_exp_pos = lf_curve.evaluate(s);
            lf_foot_exp_vel = lf_curve.velocity(s);
            lf_foot_exp_acc = lf_curve.acceleration(s);

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
            if(t_now > duration)
            {

                change_flag=false;
                //cross_wall_stage=3;
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

                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.36,0.0,0.10),4.0);
                // lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(cross_x_lf,cross_y_lf,cross_z_lf),time_s);

                change_flag=false;
                //cross_wall_stage=4;
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
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                lf_leg_step.update_support_trajectory(robot->lf_joint_pos,Vector3D(-0.055,-1.0,0.0),2.0);
                // lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(cross_x_lf,cross_y_lf,cross_z_lf),time_s);

                change_flag=false;
                //cross_wall_stage=5;
            }
        }
        //右前腿规划：5-9
       if(cross_wall_stage == 6 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
        
            bool success=false;
            double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            lf_joint_exp_pos_ = std::get<0>(lf_leg_step.get_target(time, success));
            rf_foot_exp_pos=wall_rf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;
            if(!success)
            {
                wall_lf_foot_pos=robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.08,0.0,-0.14),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.08,0.0,-0.12),2.0);
                
                // rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(cross_x_rb,cross_y_rb,cross_z_rb),time_s);
                // lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(cross_x_lb,cross_y_lb,cross_z_lb),time_s);

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
                lf_wheel_vel= 0.0;
                rf_wheel_vel=0.0;
                lb_wheel_vel= 0.05;
                rb_wheel_vel=-0.05;

                double wheel_F = k_F * ((lb_wheel_vel / Robot::WHEEL_RADIUS) - (robot->lb_wheel_omega -robot->rb_wheel_omega) / 2.0f);
                lb_wheel_force = wheel_F;
                rb_wheel_force = -wheel_F;
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



                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(-0.20,-0.20,0.40),3.0);
                // rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(cross_x_rf,cross_y_rf,cross_z_rf),time_s);

                change_flag=false;
                //cross_wall_stage=7;
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

                lf_wheel_vel= 0.0;
                rf_wheel_vel= 0.0;
                lb_wheel_vel= 0.0;
                rb_wheel_vel= 0.0;

                lf_wheel_force = 0.0;
                lb_wheel_force = 0.0;
                rf_wheel_force = 0.0;
                rb_wheel_force = 0.0;

                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.10,-0.20,0.29),3.0);
                // rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(cross_x_rf,cross_y_rf,cross_z_rf),time_s);

                change_flag=false;
                //cross_wall_stage=8;
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
                // rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(cross_x_rf,cross_y_rf,cross_z_rf),time_s);

                change_flag=false;
                //cross_wall_stage=9;     
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

                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.40,0.0,0.30),3.0);
                // rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(cross_x_rf,cross_y_rf,cross_z_rf),time_s);

                change_flag=false;
                //cross_wall_stage=10;     
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
                
                // lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.30,0.0,0.29),2.0);
                // rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.30,0.0,0.29),2.0);
                // lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(cross_x_lf,cross_y_lf,cross_z_lf),time_s);
                // rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(cross_x_rf,cross_y_rf,cross_z_rf),time_s);
                
                //cross_wall_stage_time=robot->node_->get_clock()->now();
                change_flag=false;
                //cross_wall_stage=11;     
            }
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
            
            // std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            // std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            lf_foot_exp_pos=wall_lf_foot_pos;
            rf_foot_exp_pos=wall_rf_foot_pos;
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
                // lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(cross_x_lf,cross_y_lf,cross_z_lf),time_s);
                // rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(cross_x_rf,cross_y_rf,cross_z_rf),time_s);
                // lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(cross_x_lb,cross_y_lb,cross_z_lb),time_s);

                //cross_wall_stage_time=robot->node_->get_clock()->now();
                change_flag=false;
                //cross_wall_stage=12;     
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
            
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            rb_foot_exp_pos=wall_rb_foot_pos;

            int result;
            Vector3D rf_exp = robot->rf_leg_calc->joint_pos(rf_foot_exp_pos, &result);
            Vector3D lb_exp = robot->lb_leg_calc->joint_pos(lb_foot_exp_pos, &result);
            Vector3D rb_exp = robot->rb_leg_calc->joint_pos(rb_foot_exp_pos, &result);
            //Vector3D lf_exp = robot->lf_leg_calc->joint_pos(lf_foot_exp_pos, &result, robot->lf_joint_pos);
            for(int i = 0;i<3;i++)
            {
                joints_target.legs[1].joints[i].rad    = (float)rf_exp[i];
                joints_target.legs[2].joints[i].rad    = (float)lb_exp[i];
                joints_target.legs[3].joints[i].rad    = (float)rb_exp[i];
                //joints_target.legs[0].joints[i].rad    = (float)lf_exp[i];            
            }

            if(!success)
            {   
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.25,0.0,0.0),0.8);
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.25,0.0,0.0),0.8);
                // lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(cross_x_lf,cross_y_lf,cross_z_lf),time_s);
                // rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(cross_x_rf,cross_y_rf,cross_z_rf),time_s);
                
                lb_wheel_vel=  0.3;
                rb_wheel_vel= -0.3;
                double wheel_F = k_F * ((lb_wheel_vel / Robot::WHEEL_RADIUS) - (robot->lb_wheel_omega  -robot->rb_wheel_omega) / 4.0f);
                lb_wheel_force = wheel_F;
                rb_wheel_force = -wheel_F;

                //cross_wall_stage_time=robot->node_->get_clock()->now();
                change_flag=false;
                //cross_wall_stage=13;     
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

                auto lb_cart_force = robot->lb_leg_calc->foot_force(robot->lb_joint_pos, robot->lb_joint_torque, robot->lb_forward_torque);
                if (lb_cart_force[0] > 15.0 || lb_cart_force[0] < -15.0){
                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.25,0.0,-0.07),0.1);
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.25,0.0,-0.07),0.1);
                // lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(cross_x_lf,cross_y_lf,cross_z_lf),time_s);
                // rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(cross_x_rf,cross_y_rf,cross_z_rf),time_s);

                lb_wheel_vel= 0.0;
                rb_wheel_vel= 0.0;
                double wheel_F = k_F * ((lb_wheel_vel / Robot::WHEEL_RADIUS) - (robot->lb_wheel_omega  -robot->rb_wheel_omega) / 4.0f);
                lb_wheel_force = wheel_F;
                rb_wheel_force = -wheel_F;

                //cross_wall_stage_time=robot->node_->get_clock()->now();
                change_flag=false;
                //cross_wall_stage=14; 
                }    
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
                
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(-0.10,0.0,-0.1),2.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.1,0.0,0.1),2.0);
                // lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(cross_x_lb,cross_y_lb,cross_z_lb),time_s);
                // rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(cross_x_rb,cross_y_rb,cross_z_rb),time_s);

                //cross_wall_stage_time=robot->node_->get_clock()->now();
                change_flag=false;
                //cross_wall_stage=15;
            }
        }
        if(cross_wall_stage == 16 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
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
                // rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(cross_x_rb,cross_y_rb,cross_z_rb),time_s);
                // lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(cross_x_lb,cross_y_lb,cross_z_lb),time_s);

                //cross_wall_stage_time=robot->node_->get_clock()->now();
                change_flag=false;
                //cross_wall_stage=16;
            }
        }
        if(cross_wall_stage == 17 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
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
                // lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(cross_x_lf,cross_y_lf,cross_z_lf),time_s);
                // rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(cross_x_rf,cross_y_rf,cross_z_rf),time_s);
                // rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(cross_x_rb,cross_y_rb,cross_z_rb),time_s);
                // lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(cross_x_lb,cross_y_lb,cross_z_lb),time_s);

                //cross_wall_stage_time=robot->node_->get_clock()->now();
                change_flag=false;
                //cross_wall_stage=17;
            }
        }
        if(cross_wall_stage == 18 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
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
//!!!!!!!!!!这里未改
                //lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.35,0.0,0.28),2.0);
                //rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.35,0.0,0.28),2.0);

//!!!!!!!!!
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.34,-0.12,0.35),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.34,0.12,0.35),2.0);
                // rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(cross_x_rb,cross_y_rb,cross_z_rb),time_s);
                // lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(cross_x_lb,cross_y_lb,cross_z_lb),time_s);

                //cross_wall_stage_time=robot->node_->get_clock()->now();
                change_flag=false;
                //cross_wall_stage=18;
            }
        }
        if(cross_wall_stage == 19 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
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
//!!!!!!!!!!这里未改
                //lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.35,0.0,0.18),2.0);
                //rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.35,0.0,0.18),2.0);
//!!!!!!!!!
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.35,-0.06,0.30),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.35,0.06,0.30),2.0);
                // rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(cross_x_rb,cross_y_rb,cross_z_rb),time_s);
                // lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(cross_x_lb,cross_y_lb,cross_z_lb),time_s);

                //cross_wall_stage_time=robot->node_->get_clock()->now();
                change_flag=false;
                //cross_wall_stage=19;

            }
        }
        if(cross_wall_stage == 20 && change_flag == true)
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

                // lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(cross_x_lf,cross_y_lf,cross_z_lf),time_s);
                // rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(cross_x_rf,cross_y_rf,cross_z_rf),time_s);
                // rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(cross_x_rb,cross_y_rb,cross_z_rb),time_s);
                // lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(cross_x_lb,cross_y_lb,cross_z_lb),time_s);

                //cross_wall_stage_time=robot->node_->get_clock()->now();
                change_flag=false;
                //cross_wall_stage=20;
            }
        }
        if(cross_wall_stage == 21 && change_flag == true)
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
                // rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(cross_x_rb,cross_y_rb,cross_z_rb),time_s);
                // lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(cross_x_lb,cross_y_lb,cross_z_lb),time_s);

                //cross_wall_stage_time=robot->node_->get_clock()->now();
                change_flag=false;
                //cross_wall_stage=21;
            }
        }
        if(cross_wall_stage==22 && change_flag == true)
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
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                RCLCPP_INFO(robot->node_->get_logger(),"HELLO");
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,0.0),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0),2.0);

                // rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(cross_x_rb,cross_y_rb,cross_z_rb),time_s);
                // lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(cross_x_lb,cross_y_lb,cross_z_lb),time_s);

                //cross_wall_stage_time=robot->node_->get_clock()->now();
                //cross_wall_stage=21;
                change_flag=false;
                return "stop";
            }
        }

        
       
        
        joints_target.legs[0] =
            robot->lf_leg_calc->signal_leg_calc(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force,  &robot->lf_forward_torque,lf_wheel_vel,lf_wheel_force);
        joints_target.legs[1] =
            robot->rf_leg_calc->signal_leg_calc(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, lf_foot_exp_force,  &robot->rf_forward_torque,rf_wheel_vel,rf_wheel_force);
        joints_target.legs[2] =
            robot->lb_leg_calc->signal_leg_calc(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lf_foot_exp_force,  &robot->lb_forward_torque,lb_wheel_vel,lb_wheel_force);
        joints_target.legs[3] =
            robot->rb_leg_calc->signal_leg_calc(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, lf_foot_exp_force,  &robot->rb_forward_torque,rb_wheel_vel,rb_wheel_force);
        

        return "cross_wall";
    }









