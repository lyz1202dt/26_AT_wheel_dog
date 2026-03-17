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
        robot->node_->declare_parameter("cross_k_F",1.0);
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







    //         // 打开 CSV 文件（使用时间戳命名，避免覆盖）
    // auto now = std::chrono::system_clock::now();
    // auto t_c = std::chrono::system_clock::to_time_t(now);
    // std::stringstream filename;
    // filename << "joint_data_" << std::put_time(std::localtime(&t_c), "%Y%m%d_%H%M%S") << ".csv";
    // csv_file_.open(filename.str());
    // if (csv_file_.is_open())
    // {
    //     // 写入表头（根据你实际记录的变量调整列名）
    //     csv_file_ << "timestamp_sec,"
    //               << "q0_exp_pos,q0_pos,"
    //               << "q1_exp_pos,q1_pos,"
    //               << "q2_exp_pos,q2_pos,"
    //               << "dq0_exp_v,dq0_v,"
    //               << "dq1_exp_v,dq1_v,"
    //               << "dq2_exp_v,dq2_v,"
    //               << "tau0,tau1,tau2\n";
    //     csv_file_.flush();
    //     RCLCPP_INFO(robot->node_->get_logger(), "CSV recording started: %s", filename.str().c_str());
    // }
    // else
    // {
    //     RCLCPP_ERROR(robot->node_->get_logger(), "Failed to open CSV file for recording!");
    // }

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
        bool use_limit_lf = false;
        bool use_limit_rf = false;
        bool use_limit_lb = false;
        bool use_limit_rb = false;
std::string Cross_WallState::update(Robot* robot){
        // lf_foot_exp_vel = Vector3D::Zero();
        // lf_foot_exp_acc = Vector3D::Zero();
        // rf_foot_exp_vel = Vector3D::Zero();
        // rf_foot_exp_acc = Vector3D::Zero();
        // lb_foot_exp_vel = Vector3D::Zero();
        // lb_foot_exp_acc = Vector3D::Zero();
        // rb_foot_exp_vel = Vector3D::Zero();
        // rb_foot_exp_acc = Vector3D::Zero();

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
                use_limit_lf = false;
                use_limit_lb = false;
                use_limit_rb = false;
                use_limit_rf = false;

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

                lf_leg_step.update_support_trajectory(lf_cart_pos, Vector3D(-0.05, y_target, -0.06), 4.0);
                rf_leg_step.update_support_trajectory(rf_cart_pos, Vector3D(-0.05, y_target, -0.06), 4.0);
                lb_leg_step.update_support_trajectory(lb_cart_pos, Vector3D(-0.05, y_target, -0.06), 4.0);
                rb_leg_step.update_support_trajectory(rb_cart_pos, Vector3D(-0.05, y_target, -0.06), 4.0);

            
                // change_flag=false;
                 cross_wall_stage=1;
            }
        }

        else if (cross_wall_stage == 1 && change_flag == true){         // 执行设置的腿长，调整质心位置，使其落在支撑三角形内

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            bool success=false;
            use_limit_lf = false;
            use_limit_lb = false;
            use_limit_rb = false;
            use_limit_rf = false;
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
                lf_leg_step.update_support_trajectory(lf_cart_pos, Vector3D(-0.05, y_target, -0.06), 2.0);
                rf_leg_step.update_support_trajectory(rf_cart_pos, Vector3D(-0.05, y_target, -0.06), 2.0);
                lb_leg_step.update_flight_trajectory(lb_cart_pos, Vector3D(0.0, 0.0,0.0), 
                Vector3D(0.22, y_target, -0.06), Vector2D(0.0, 0.0), 2.0, 0.04);
                rb_leg_step.update_support_trajectory(rb_cart_pos, Vector3D(-0.05, y_target, -0.06), 2.0);
 
                // change_flag=false;
                cross_wall_stage=2;     //轨迹执行完后跳转到状态2
            }
        }

        else if (cross_wall_stage == 2 && change_flag == true){         // 执行设置的腿长，调整质心位置，使其落在支撑三角形内

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            bool success=false;
            use_limit_lf = false;
            use_limit_lb = false;
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
                //规划lf腿轨迹
                Vec3 lf_start_pos = lf_cart_pos;// 起点
                Vec3 lf_peak_pos = lf_start_pos + Vec3(-0.2,0.05,0.15);// P1: 抬高离墙
                Vec3 lf_mid_pos = lf_start_pos + Vec3(0.0,0.04,0.45);
                Vec3 lf_end_pos  = lf_start_pos + Vec3(0.35,-0.05,0.435);// P2: 终点


                std::vector<Vec3> control_points = {lf_start_pos, lf_peak_pos, lf_mid_pos, lf_end_pos};
                lf_curve.setControlPoints(control_points); 
                //change_flag=false;
                cross_wall_stage=3;     //轨迹执行完后跳转到状态2
            }
        }

        // if (cross_wall_stage == -100 && change_flag == true){
        //     if (cross_wall_stage != last_stage)
        //     {
        //         cross_wall_stage_time = robot->node_->get_clock()->now();
        //         last_stage = cross_wall_stage;
        //         foot_init = false;
        //     }
        //     double duration = 4.0; // 轨迹总时间4秒
        //     double t_now = (robot->node_->get_clock()->now() - cross_wall_stage_time).seconds();
        //     double s = std::min(t_now / duration, 1.0); // 归一化 t ∈ [0,1]
            
        //     // -------------------
        //     // 抬高身体
        //     // -------------------

        //     if (!foot_init)
        //     {
        //         rf_init_pos.z() = rf_foot_exp_pos.z();
        //         lb_init_pos.z() = lb_foot_exp_pos.z();
        //         rb_init_pos.z() = rb_foot_exp_pos.z();
        //         rf_init_pos.x() = rf_foot_exp_pos.x();
        //         lb_init_pos.x() = lb_foot_exp_pos.x();
        //         rb_init_pos.x() = rb_foot_exp_pos.x();
        //         rf_init_pos.y() = rf_foot_exp_pos.y();
        //         lb_init_pos.y() = lb_foot_exp_pos.y();
        //         rb_init_pos.y() = rb_foot_exp_pos.y();

        //         foot_init = true;
        //     }

        //     double lift_height = 0.04;
        //     double smooth = 3*s*s - 2*s*s*s;
        //     double dz = lift_height * smooth;

        //     rf_foot_exp_pos.z() = rf_init_pos.z() - dz;
        //     lb_foot_exp_pos.z() = lb_init_pos.z() - dz;
        //     rb_foot_exp_pos.z() = rb_init_pos.z() - dz;

        //     rf_foot_exp_pos.x() = rf_init_pos.x();
        //     rf_foot_exp_pos.y() = rf_init_pos.y();

        //     lb_foot_exp_pos.x() = lb_init_pos.x();
        //     lb_foot_exp_pos.y() = lb_init_pos.y();

        //     rb_foot_exp_pos.x() = rb_init_pos.x();
        //     rb_foot_exp_pos.y() = rb_init_pos.y();


        //     // 三足支撑：rf, lb, rb
        //     auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
        //     auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
        //     auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

        //     Eigen::Matrix3d A;
        //     Eigen::Vector3d b;

        //     A << 1, 1, 1,
        //         rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
        //         rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
        //     b << mass*9.8, 0, 0;

        //     Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

        //     rf_foot_exp_force = Vector3D(0,0,-forces(0));
        //     lb_foot_exp_force = Vector3D(0,0,-forces(1));
        //     rb_foot_exp_force = Vector3D(0,0,-forces(2));
        //     lf_foot_exp_force = Vector3D::Zero(); // 摆动腿

            
        //     if(t_now > duration)
        //     {
        //     //规划lf腿轨迹
        //     Vec3 lf_start_pos = lf_cart_pos;// 起点
        //     Vec3 lf_peak_pos = lf_start_pos + Vec3(-0.15,0.08,0.20);// P1: 抬高离墙
        //     Vec3 lf_mid_pos = lf_start_pos + Vec3(0.1,0.06,0.42);
        //     Vec3 lf_end_pos  = lf_start_pos + Vec3(0.35,-0.02,0.40);// P2: 终点


        //     std::vector<Vec3> control_points = {lf_start_pos, lf_peak_pos, lf_mid_pos, lf_end_pos};
        //     lf_curve.setControlPoints(control_points);
        //         change_flag=false;
        //         //cross_wall_stage=3;
        //     }
        // }
        else if (cross_wall_stage == 3 && change_flag == true) {

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
                
            }
            use_limit_lf = false;
            use_limit_lb = false;
            use_limit_rb = false;
            use_limit_rf = false;

            double duration = 6.0;
            double t_now = (robot->node_->get_clock()->now() - cross_wall_stage_time).seconds();
            double t = std::min(t_now / duration, 1.0);

            // quintic time scaling
            double s = 10*pow(t,3) - 15*pow(t,4) + 6*pow(t,5);

            lf_foot_exp_pos = lf_curve.evaluate(s);
            lf_foot_exp_vel = lf_curve.velocity(s) / duration;
            lf_foot_exp_acc = lf_curve.acceleration(s) / (duration*duration);

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

                lf_joint_target_pos = {0.88,-0.32,-0.88};
                
                //change_flag=false;
                cross_wall_stage=4;
            }
        }
        // if(cross_wall_stage==3 && change_flag){

        //     if (cross_wall_stage != last_stage)
        //     {
        //         cross_wall_stage_time = robot->node_->get_clock()->now();
        //         last_stage = cross_wall_stage;
        //         foot_init = false;
        //     }
        //     double duration = 4.0;
        //     double t_now = (robot->node_->get_clock()->now() - cross_wall_stage_time).seconds();
        //     double s = std::min(t_now / duration, 1.0);
        //     double smooth = 3*s*s - 2*s*s*s;

        //     // -------------------
        //     // 初始化支撑腿位置
        //     // -------------------
        //     if (!foot_init)
        //     {
        //         rf_init_pos = rf_foot_exp_pos;
        //         lb_init_pos = lb_foot_exp_pos;
        //         rb_init_pos = rb_foot_exp_pos;
        //         foot_init = true;
        //     }

        //     // -------------------
        //     // 最大平移量
        //     // -------------------
        //     double dx_max = 0.02;
        //     //double dy_max = 0.02;
        //     double dx_max_lb = 0.04;
        //     double dx = dx_max * smooth;
        //     double dx_lb = dx_max_lb * smooth;
        //     //double dy = dy_max * smooth;

        //     // -------------------
        //     // 更新支撑腿期望位置
        //     // -------------------
        //     rf_foot_exp_pos.x() = rf_init_pos.x() - dx;
        //     rf_foot_exp_pos.y() = rf_init_pos.y();//+ dy;
        //     rf_foot_exp_pos.z() = rf_init_pos.z();

        //     lb_foot_exp_pos.x() = lb_init_pos.x() - dx_lb;
        //     lb_foot_exp_pos.y() = lb_init_pos.y();// + dy;
        //     lb_foot_exp_pos.z() = lb_init_pos.z();

        //     rb_foot_exp_pos.x() = rb_init_pos.x() - dx;
        //     rb_foot_exp_pos.y() = rb_init_pos.y();// + dy;
        //     rb_foot_exp_pos.z() = rb_init_pos.z();

        //     // -------------------
        //     // 计算三足支撑力
        //     // -------------------
        //     Eigen::Matrix3d A;
        //     Eigen::Vector3d b;
        //     A << 1, 1, 1,
        //         rf_foot_exp_pos.x() - mass_center_pos.x(), lb_foot_exp_pos.x() - mass_center_pos.x(), rb_foot_exp_pos.x() - mass_center_pos.x(),
        //         rf_foot_exp_pos.y() - mass_center_pos.y(), lb_foot_exp_pos.y() - mass_center_pos.y(), rb_foot_exp_pos.y() - mass_center_pos.y();
        //     b << mass*9.8, 0, 0;

        //     Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);
        //     rf_foot_exp_force = Vector3D(0,0,-forces(0));
        //     lb_foot_exp_force = Vector3D(0,0,-forces(1));
        //     rb_foot_exp_force = Vector3D(0,0,-forces(2));
        //     lf_foot_exp_force = Vector3D::Zero(); // 摆动腿

            // -------------------
            // 规划 LF 轨迹
            // -------------------
            // if(t_now > duration){
            //     Vec3 lf_start_pos = lf_cart_pos;
            //     Vec3 lf_peak_pos  = lf_start_pos + Vec3(0.05,-0.1,0.05);
            //     Vec3 lf_mid_pos   = lf_start_pos + Vec3(0.0,-0.15,0.1);
            //     Vec3 lf_end_pos   = lf_start_pos + Vec3(-0.15,-0.2,0.0);

            //     std::vector<Vec3> control_points = {lf_start_pos, lf_peak_pos, lf_mid_pos, lf_end_pos};
            //     lf_curve.setControlPoints(control_points);

            //     change_flag = false;
            //     //cross_wall_stage=5;
            // }                    double s2 = (t - 0.7) / 0.3;
        //}
        //右前腿规划：5-9
        else if(cross_wall_stage == 4 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot->node_->get_clock()->now();
                last_stage = cross_wall_stage;
                lf_joint_init_pos = robot->lf_joint_pos;
                                RCLCPP_ERROR(robot->node_->get_logger(),
                    "\033[31mlf_joint_target_pos = (%f, %f, %f)\033[0m",
                            lf_joint_target_pos[0],lf_joint_target_pos[1],lf_joint_target_pos[2]);
            }
            use_limit_lb = false;
            use_limit_rb = false;
            use_limit_rf = false;
            use_limit_lf = true;
            double duration = 3.0;
            double t_now = (robot->node_->get_clock()->now() - cross_wall_stage_time).seconds();
            double t = std::min(t_now / duration, 1.0);


            double s = 10*pow(t,3) - 15*pow(t,4) + 6*pow(t,5);
            // double sd  = (30*pow(t,2) - 60*pow(t,3) + 30*pow(t,4)) / duration;
            // double sdd = (60*t - 180*pow(t,2) + 120*pow(t,3)) / (duration*duration);

            Vector3D dq = lf_joint_target_pos - lf_joint_init_pos;

            lf_joint_exp_pos_ = lf_joint_init_pos + s * dq;
            // lf_joint_exp_vel_ = sd * dq;
            // lf_joint_exp_acc_ = sdd * dq;

            static int cnt_ = 0;
            cnt_++;
            if(cnt_>=10)
            {
                cnt_ = 0;
                RCLCPP_ERROR(robot->node_->get_logger(),
                    "\033[31mlf_joint_exp_pos_ = (%f, %f, %f)\n lf_joint_pos = (%f, %f, %f)\033[0m",
                            lf_joint_exp_pos_[0], lf_joint_exp_pos_[1], lf_joint_exp_pos_[2], robot->lf_joint_pos[0], robot->lf_joint_pos[1], robot->lf_joint_pos[2]);
                // RCLCPP_ERROR(robot->node_->get_logger(),
                //     "\033[31mlf_joint_exp_vel_ = (%f, %f, %f)\n lf_joint_vel = (%f, %f, %f)\033[0m",
                //             lf_joint_exp_vel_[0], lf_joint_exp_vel_[1], lf_joint_exp_vel_[2], robot->lf_joint_vel[0], robot->lf_joint_vel[1], robot->lf_joint_vel[2]);
            }

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
                //cross_wall_stage=4;
            }
        }
        else if(cross_wall_stage == 5 && change_flag == true){
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
        else if(cross_wall_stage == 6 && change_flag == true){
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
        else if(cross_wall_stage == 9 && change_flag == true){
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
/*******************************lf**********************************/
        if(use_limit_lf)
        {
            // double kp[3] = {15, 15, 15};
            // double kd[3] = {0,  0, 0};
            // double kv[3] = {1, 2, 3};

            for(int i=0;i<3;i++)
            {
                // double q   = robot->lf_joint_pos[i];
                // double dq_ = robot->lf_joint_vel[i];

                // dq_ = std::clamp(dq_, -5.0, 5.0);

                // double q_des  = lf_joint_exp_pos_[i];
                // double dq_des = lf_joint_exp_vel_[i];

                // double tau = kp[i]*(q_des - q) + kd[i]*(dq_des - dq_) - kv[i]*dq_;

                // tau = std::clamp(tau, -20.0, 20.0);

                // joints_target.legs[0].joints[i].torque = (float)tau;
                joints_target.legs[0].joints[i].rad    = (float)lf_joint_exp_pos_[i];//(float)q_des;
            }
            // static int n = 0;
            // n++;
            // if(n>50)
            // {
            //     RCLCPP_ERROR(robot->node_->get_logger(),
            //         "\033[31mtau = (%f, %f, %f)\033[0m",
            //                 joints_target.legs[0].joints[0].torque,joints_target.legs[0].joints[1].torque,joints_target.legs[0].joints[2].torque);
            //                 n=0;
            // }

            //     // 记录 CSV 数据（在计算完 tau 之后）
            // if (csv_enabled_ && csv_file_.is_open() && change_flag == false)
            // {
            //                     // ... existing code ...
            //                     std::lock_guard<std::mutex> lock(csv_mutex_);  // 如果需要线程安全
            //                     double t = robot->node_->get_clock()->now().seconds();
                
            //                     // 格式化一行数据
            //                     std::ostringstream oss;
            //                     oss << std::fixed << std::setprecision(6)
            //                             << t << ","
            //                             << lf_joint_exp_pos_[0] << "," << robot->lf_joint_pos[0] << ","
            //                             << lf_joint_exp_pos_[1] << "," << robot->lf_joint_pos[1] << ","
            //                             << lf_joint_exp_pos_[2] << "," << robot->lf_joint_pos[2] << ","
            //                             << lf_joint_exp_vel_[0] << "," << robot->lf_joint_vel[0] << ","
            //                             << lf_joint_exp_vel_[1] << "," << robot->lf_joint_vel[1] << ","
            //                             << lf_joint_exp_vel_[2] << "," << robot->lf_joint_vel[2] << ","
            //                             << joints_target.legs[0].joints[0].torque << ","
            //                             << joints_target.legs[0].joints[1].torque << ","
            //                             << joints_target.legs[0].joints[2].torque << "\n";
                                
            //                     csv_file_ << oss.str();
            //                     csv_lines_.push_back( oss.str() ); // 将格式化后的字符串存入 vector
            //                     if (csv_lines_.size() >= CSV_FLUSH_THRESHOLD)
            //                     {
            //                         for (const auto& line : csv_lines_)
            //                             csv_file_ << line;
            //                         csv_lines_.clear();
            //                         csv_file_.flush(); // 确保数据落盘，也可不 flush 依靠系统缓存
            //                     }
                                    
            //     // ... existing code ...
                    
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
            joints_target.legs[1].joints[i].rad =(float)rf_joint_exp_pos_[i];
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
            joints_target.legs[2].joints[i].rad =(float)lb_joint_exp_pos_[i];
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
            joints_target.legs[3].joints[i].rad =(float)rb_joint_exp_pos_[i];
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


    Cross_WallState::~Cross_WallState()
{
    // if (csv_file_.is_open())
    // {
    //     csv_file_.close();
    // }
}








