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
            else if(name=="cross_k_F")
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
    return true;
}

std::string Cross_WallState::update(Robot* robot){

        robot_interfaces::msg::RobotTarget joints_target;

 
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

        auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        lf_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_lf_grivate);
        rf_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_rf_grivate);
        lb_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_lb_grivate);
        rb_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_rb_grivate);

        if(cross_wall_stage == 0  && change_flag == true){
                use_limit_lf = false;
                use_limit_lb = false;
                use_limit_rb = false;
                use_limit_rf = false;

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
            static int cnt = 0;
            cnt++;
            if(cnt>=25 && (!stopping))
            {
            RCLCPP_ERROR(robot->node_->get_logger(),
                "\033[31m这是碰撞前的数据:lf_cart_force[0] = %f, rf_cart_force[0] = %f\033[0m",
                        lf_cart_force[0], rf_cart_force[0]);
            }
            if ((lf_cart_force[0] > 10.0 || rf_cart_force[0] > 10.0) && (!stopping)){

                if(!stopping) stop_t = 0.0;
                stopping = true;
                

                // 记录初值
                lf_vel_start = lf_wheel_vel;
                rf_vel_start = rf_wheel_vel;
                lb_vel_start = lb_wheel_vel;
                rb_vel_start = rb_wheel_vel;

                lf_force_start = lf_wheel_force;
                rf_force_start = rf_wheel_force;
                lb_force_start = lb_wheel_force;
                rb_force_start = rb_wheel_force;


                // wall_lf_foot_pos=robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
                // wall_rf_foot_pos=robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
                // wall_lb_foot_pos=robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
                // wall_rb_foot_pos=robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

                // lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.0,0.08,0.0),2.0);
                // rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.0,0.08,0.0),2.0);
                // lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.08,0.0),2.0);
                // rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.08,0.0),2.0);
                
                // //change_flag=false;
                // cross_wall_stage=1;
            }
            // ================= 平滑减速 =================
            if (stopping)
            {
                static int cnt = 0;
                cnt++;
                if(cnt>=25)
                {
                RCLCPP_ERROR(robot->node_->get_logger(),
                    "\033[31m数据:lf_cart_force[0] = %f, rf_cart_force[0] = %f\033[0m",
                            lf_cart_force[0], rf_cart_force[0]);
                }
                stop_t += 0.004;

                double tau = stop_t / stop_T;
                if (tau > 1.0) tau = 1.0;


                double s = 10*pow(tau,3) - 15*pow(tau,4) + 6*pow(tau,5);

                // 速度快速降
                lf_wheel_vel = lf_vel_start * (1 - s);
                rf_wheel_vel = rf_vel_start * (1 - s);
                lb_wheel_vel = lb_vel_start * (1 - s);
                rb_wheel_vel = rb_vel_start * (1 - s);

                // ⚠️ 力慢一点降（关键！防翻）
                double s_force = pow(s, 2.0);

                lf_wheel_force = lf_force_start * (1 - s_force);
                rf_wheel_force = rf_force_start * (1 - s_force);
                lb_wheel_force = lb_force_start * (1 - s_force);
                rb_wheel_force = rb_force_start * (1 - s_force);
                // static int cnt = 0;
                // cnt++;
                // if(cnt>=25)
                // {
                // RCLCPP_ERROR(robot->node_->get_logger(),
                //     "\033[31mtau = %.2f,\n s = %.2f\n, s_force = %.2f\033[0m",
                //             tau, s,s_force);
                // }

                // ================= 停稳后再切阶段 =================
                if (tau >= 1.0)
                {
                    stopping = false;

                    lf_wheel_vel = 0.0;
                    rf_wheel_vel = 0.0;
                    lb_wheel_vel = 0.0;
                    rb_wheel_vel = 0.0;

                    // ⚠️ 力不要全清（可以留一点贴墙）
                    lf_wheel_force = 0.0;
                    rf_wheel_force = 0.0;
                    lb_wheel_force = 0.0;
                    rb_wheel_force = 0.0;

                    // ======= 这里才记录足端位置 =======
                    wall_lf_foot_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
                    wall_rf_foot_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
                    wall_lb_foot_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
                    wall_rb_foot_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

                    lf_leg_step.update_support_trajectory(wall_lf_foot_pos, Vector3D(0.0,0.08,0.0), 2.0);
                    rf_leg_step.update_support_trajectory(wall_rf_foot_pos, Vector3D(0.0,0.08,0.0), 2.0);
                    lb_leg_step.update_support_trajectory(wall_lb_foot_pos, Vector3D(0.0,0.08,0.0), 2.0);
                    rb_leg_step.update_support_trajectory(wall_rb_foot_pos, Vector3D(0.0,0.08,0.0), 2.0);
                    //change_flag=false;
                    cross_wall_stage = 1;
                }
            }

    }  //0:碰撞检测后,狗身向右倾斜，准备迈左后腿
      
        else if (cross_wall_stage == 1 && change_flag == true){        
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
                    lf_wheel_vel = 0.0;
                    rf_wheel_vel = 0.0;
                    lb_wheel_vel = 0.0;
                    rb_wheel_vel = 0.0;

                    // ⚠️ 力不要全清（可以留一点贴墙）
                    lf_wheel_force = 0.0;
                    rf_wheel_force = 0.0;
                    lb_wheel_force = 0.0;
                    rb_wheel_force = 0.0;
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

                lf_leg_step.update_support_trajectory(wall_lf_foot_pos,wall_lf_foot_pos,2.0);
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,wall_rf_foot_pos,2.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,wall_rb_foot_pos,2.0);
                lb_leg_step.update_flight_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0), Vector3D(0.12,0.08,0.0), Vector2D(0.0,0.0), 2.0,0.06);
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

                lf_leg_step.update_support_trajectory(robot->lf_joint_pos,Vector3D(1.44,0.822,-0.165),4.0);
                
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
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success); 
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

                lf_leg_step.update_support_trajectory(Vector3D(1.44,0.822,-0.165),Vector3D(1.44,-0.92,0.48),4.0);
                
                   
                //change_flag=false;
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
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            

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
                // wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.0,0.0,0.0),4.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0),4.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,0.0),4.0);

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
                // wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_leg_step.update_support_trajectory(Vector3D(1.44,-0.92,0.48),Vector3D(-0.12,-1.00,0.96),2.0);
                rf_leg_step.update_support_trajectory(wall_rf_foot_pos,wall_rf_foot_pos,2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,wall_lb_foot_pos,2.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,wall_rb_foot_pos,2.0);
                //change_flag=false;
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
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
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

                // lf_leg_step.update_support_trajectory(Vector3D(0.12,-1.10,0.48),Vector3D(0.12,-0.80,0.48),2.0);
                rf_leg_step.update_support_trajectory(rf_joint_exp_pos_,Vector3D(-1.35,-0.827,-0.531),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,-0.03),2.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,-0.07),2.0);
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

            // std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            
            if(!success)
            {
                wall_lf_foot_pos=lf_joint_exp_pos_;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_leg_step.update_support_trajectory(Vector3D(-1.35,-0.827,-0.531),Vector3D(-1.5,1.08,-0.531),3.0);

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

                rf_leg_step.update_support_trajectory(Vector3D(-1.5,1.08,-0.531),Vector3D(-0.60,1.25,-0.531),3.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,-0.03),3.0);
                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,-0.03),3.0);
                
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
            
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target(time, success);
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;

            if(!success)
            {    
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                cross_wall_stage = 11;
                //change_flag=false;
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
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            double s = 2.0f;
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
            if(time > s)
            { 
                lf_wheel_vel= 0.0;
                rf_wheel_vel= 0.0;
                lb_wheel_vel= 0.0;
                rb_wheel_vel= 0.0;
                lf_wheel_force = 0.0;
                rf_wheel_force = 0.0;
                lb_wheel_force = 0.0;   
                rb_wheel_force = 0.0;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(-0.10,-0.21,0.40),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(-0.10,0.21,0.40),2.0);

                //change_flag=false;
                cross_wall_stage=12;
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
        else if(cross_wall_stage == 12 && change_flag == true)
        {
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

                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.18,-0.08,0.39),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.18, 0.08,0.39),2.0);

                //change_flag=false;
                cross_wall_stage=13;
            }
        }
        // if(cross_wall_stage == 13 && change_flag == true)
        // {
        //     if (cross_wall_stage != last_stage)
        //     {
        //         cross_wall_stage_time = robot->node_->get_clock()->now();
        //         last_stage = cross_wall_stage;
        //     }
        //     bool success=false;
        //     double time=(robot->node_->get_clock()->now()-cross_wall_stage_time).seconds();
        //     std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
        //     std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
        //     if(!success)
        //     {
        //         wall_lb_foot_pos=lb_foot_exp_pos;
        //         wall_rb_foot_pos=rb_foot_exp_pos;

        //         rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.1,-0.15,0.30),2.0);
        //         lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.1,0.15,0.30),2.0);

        //         change_flag=false;
        //         //cross_wall_stage=18;
        //     }
        // }
        else if(cross_wall_stage == 13 && change_flag == true)
        {
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
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
           
            if(!success)
            {

                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;


                rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(-0.22,0.0,-0.05),2.0);
                lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(-0.22,0.0,-0.05),2.0);

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
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
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
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
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
        // if(cross_wall_stage == 3 || cross_wall_stage == 4 || cross_wall_stage == 5 || cross_wall_stage == 6)
        // {
        //     lf_joint_omega  = robot->lf_leg_calc->joint_vel(lf_joint_exp_pos_, Vector3D{0,0,0});
        //     lf_joint_torque = robot->lf_leg_calc->joint_torque_foot_force(lf_joint_exp_pos_, Vector3D{0,0,0});
        //     lf_joint_torque +=robot->lf_leg_calc->joint_torque_dynamic(lf_joint_exp_pos_, lf_joint_omega, Vector3D{0,0,0});

        //     for (int i = 0; i < 3; i++) {
        //     joints_target.legs[0].joints[i].rad    = static_cast<float>(lf_joint_exp_pos_[i]);
        //     joints_target.legs[0].joints[i].omega  = static_cast<float>(lf_joint_omega[i]);
        //     joints_target.legs[0].joints[i].torque = static_cast<float>(lf_joint_torque[i]);
        //     }
        //     joints_target.legs[1] =
        //         robot->rf_leg_calc->signal_leg_calc(rf_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), &rf_forward_torque,rf_wheel_vel,rf_wheel_force);
        //     joints_target.legs[2] =
        //         robot->lb_leg_calc->signal_leg_calc(lb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), &lb_forward_torque,lb_wheel_vel,lb_wheel_force);
        //     joints_target.legs[3] =
        //          robot->rb_leg_calc->signal_leg_calc(rb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0),&rb_forward_torque,rb_wheel_vel,rb_wheel_force);
        // }
        // else if(cross_wall_stage == 7 || cross_wall_stage == 8 || cross_wall_stage == 9 || cross_wall_stage == 10 || cross_wall_stage == 11)
        // {
        //     lf_joint_omega  = robot->lf_leg_calc->joint_vel(lf_joint_exp_pos_, Vector3D{0,0,0});
        //     lf_joint_torque = robot->lf_leg_calc->joint_torque_foot_force(lf_joint_exp_pos_, Vector3D{0,0,0});
        //     lf_joint_torque +=robot->lf_leg_calc->joint_torque_dynamic(lf_joint_exp_pos_, lf_joint_omega, Vector3D{0,0,0});

        //     rf_joint_omega  = robot->rf_leg_calc->joint_vel(rf_joint_exp_pos_, Vector3D{0,0,0});
        //     rf_joint_torque = robot->rf_leg_calc->joint_torque_foot_force(rf_joint_exp_pos_, Vector3D{0,0,0});
        //     rf_joint_torque +=robot->rf_leg_calc->joint_torque_dynamic(rf_joint_exp_pos_, rf_joint_omega, Vector3D{0,0,0});

        //     for (int i = 0; i < 3; i++) {
        //     joints_target.legs[0].joints[i].rad    = static_cast<float>(lf_joint_exp_pos_[i]);
        //     joints_target.legs[0].joints[i].omega  = static_cast<float>(lf_joint_omega[i]);
        //     joints_target.legs[0].joints[i].torque = static_cast<float>(lf_joint_torque[i]);
        //     }

        //     for (int i = 0; i < 3; i++) {
        //     joints_target.legs[1].joints[i].rad    = static_cast<float>(rf_joint_exp_pos_[i]);
        //     joints_target.legs[1].joints[i].omega  = static_cast<float>(rf_joint_omega[i]);
        //     joints_target.legs[1].joints[i].torque = static_cast<float>(rf_joint_torque[i]);
        //     }

        //     joints_target.legs[2] =
        //         robot->lb_leg_calc->signal_leg_calc(lb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0),  &lb_forward_torque,lb_wheel_vel,lb_wheel_force);
        //     joints_target.legs[3] =
        //         robot->rb_leg_calc->signal_leg_calc(rb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0), &rb_forward_torque,rb_wheel_vel,rb_wheel_force);
        // }
        // else if(cross_wall_stage == 12 || cross_wall_stage == 13 || cross_wall_stage == 14 || cross_wall_stage == 15)
        // {
        //     lf_joint_omega  = robot->lf_leg_calc->joint_vel(lf_joint_exp_pos_, Vector3D{0,0,0});
        //     lf_joint_torque = robot->lf_leg_calc->joint_torque_foot_force(lf_joint_exp_pos_, Vector3D{0,0,0});
        //     lf_joint_torque +=robot->lf_leg_calc->joint_torque_dynamic(lf_joint_exp_pos_, lf_joint_omega, Vector3D{0,0,0});

        //     rf_joint_omega  = robot->rf_leg_calc->joint_vel(rf_joint_exp_pos_, Vector3D{0,0,0});
        //     rf_joint_torque = robot->rf_leg_calc->joint_torque_foot_force(rf_joint_exp_pos_, Vector3D{0,0,0});
        //     rf_joint_torque +=robot->rf_leg_calc->joint_torque_dynamic(rf_joint_exp_pos_, rf_joint_omega, Vector3D{0,0,0});

        //     for (int i = 0; i < 3; i++) {
        //     joints_target.legs[0].joints[i].rad    = static_cast<float>(lf_joint_exp_pos_[i]);
        //     joints_target.legs[0].joints[i].omega  = static_cast<float>(lf_joint_omega[i]);
        //     joints_target.legs[0].joints[i].torque = static_cast<float>(lf_joint_torque[i]);
        //     }

        //     for (int i = 0; i < 3; i++) {
        //     joints_target.legs[1].joints[i].rad    = static_cast<float>(rf_joint_exp_pos_[i]);
        //     joints_target.legs[1].joints[i].omega  = static_cast<float>(rf_joint_omega[i]);
        //     joints_target.legs[1].joints[i].torque = static_cast<float>(rf_joint_torque[i]);
        //     }

        //     joints_target.legs[2] =
        //         robot->lb_leg_calc->signal_leg_calc(lb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0),  &lb_forward_torque,lb_wheel_vel,lb_wheel_force);
        //     joints_target.legs[3] =
        //         robot->rb_leg_calc->signal_leg_calc(rb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0),  &rb_forward_torque,rb_wheel_vel,rb_wheel_force);
        // }
        // else
        // {
        //     joints_target.legs[0] =
        //         robot->lf_leg_calc->signal_leg_calc(lf_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0),  &lf_forward_torque,lf_wheel_vel,lf_wheel_force);
        //     joints_target.legs[1] =
        //         robot->rf_leg_calc->signal_leg_calc(rf_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0),  &rf_forward_torque,rf_wheel_vel,rf_wheel_force);
        //     joints_target.legs[2] =
        //         robot->lb_leg_calc->signal_leg_calc(lb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0),  &lb_forward_torque,lb_wheel_vel,lb_wheel_force);
        //     joints_target.legs[3] =
        //         robot->rb_leg_calc->signal_leg_calc(rb_foot_exp_pos, Vector3D(0,0,0), Vector3D(0,0,0), Vector3D(0,0,0),  &rb_forward_torque,rb_wheel_vel,rb_wheel_force);
        // }


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









