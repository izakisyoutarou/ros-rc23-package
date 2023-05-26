#pragma once
#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <string>
//使うmsg
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "controller_interface_msg/msg/pad.hpp"
#include "controller_interface_msg/msg/pole.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "controller_interface_msg/msg/injection.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
//他のpkg
#include "utilities/can_utils.hpp"
#include "utilities/utils.hpp"
#include "socket_udp.hpp"
#include "trapezoidal_velocity_planner.hpp"
#include "my_visibility.h"

#include "send_udp.hpp"
#include "super_command.hpp"

namespace controller_interface
{
    using VelPlanner = velocity_planner::trapezoidal_velocity_planner::TrapezoidalVelocityPlanner;
    using VelPlannerLimit = velocity_planner::trapezoidal_velocity_planner::Limit_t;

    class SmartphoneGamepad : public rclcpp::Node
    {
        public:
            CONTROLLER_INTERFACE_PUBLIC
            explicit SmartphoneGamepad(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            
            CONTROLLER_INTERFACE_PUBLIC
            explicit SmartphoneGamepad(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        private:
            //ER_mainのcontrollerから
            rclcpp::Subscription<controller_interface_msg::msg::Pad>::SharedPtr _sub_pad_main;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_state_num_ER;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_initial_state;

            //ER_subのcontrollerから
            rclcpp::Subscription<controller_interface_msg::msg::Pad>::SharedPtr _sub_pad_sub;

            //mainボードから
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_main_injection_possible;
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_main_injection_complete;

            //spline_pidから
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_spline;

            //injection_param_calculatorから
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_injection_calculator_0;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_injection_calculator_1;

            //sequencerから
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_injection_pole_m0;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_injection_pole_m1;

            //CanUsbへ
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb;

            //各nodeと共有
            rclcpp::Publisher<controller_interface_msg::msg::BaseControl>::SharedPtr _pub_base_control;
            rclcpp::Publisher<controller_interface_msg::msg::Convergence>::SharedPtr _pub_convergence;
            rclcpp::Publisher<controller_interface_msg::msg::Injection>::SharedPtr _pub_injection;

            //gazebo_simulator用のpub
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_gazebo;

            //timer
            rclcpp::TimerBase::SharedPtr _pub_heartbeat;
            rclcpp::TimerBase::SharedPtr _pub_timer_convergence;
            rclcpp::TimerBase::SharedPtr _socket_timer;
            // rclcpp::TimerBase::SharedPtr _start_timer;

            //QoS
            rclcpp::QoS _qos = rclcpp::QoS(10);
            
            //controller_mainからのcallback
            void callback_pad_main(const controller_interface_msg::msg::Pad::SharedPtr msg);
            void callback_state_num_ER(const std_msgs::msg::String::SharedPtr msg);
            void callback_initial_state(const std_msgs::msg::String::SharedPtr msg);

            //controller_subからのcallback
            void callback_pad_sub(const controller_interface_msg::msg::Pad::SharedPtr msg);

            //mainからのcallback
            void callback_main(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            void callback_injection_complete(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);

            //splineからのcallback
            void callback_spline(const std_msgs::msg::Bool::SharedPtr msg);

            //injection_param_calculatorからのcallback
            void callback_injection_calculator_0(const std_msgs::msg::Bool::SharedPtr msg);
            void callback_injection_calculator_1(const std_msgs::msg::Bool::SharedPtr msg);

            //sequencerからのcallback
            void callback_injection_pole_m0(const std_msgs::msg::String::SharedPtr msg);
            void callback_injection_pole_m1(const std_msgs::msg::String::SharedPtr msg);

            void _recv_callback();

            void _recv_joy_main(const unsigned char data[16]);

            controller_interface_msg::msg::BaseControl msg_base_control;
            
            //base_control用
            bool is_reset = false;
            bool is_emergency = false;
            bool is_move_autonomous = false;
            bool is_injection_autonomous = false;
            bool is_slow_speed = false;
            bool is_injection_mech_stop_m0 = false;
            bool is_injection_mech_stop_m1 = false;
            std::string initial_state = "";

            //convergence用
            bool is_spline_convergence;
            bool is_injection_calculator0_convergence;
            bool is_injection_calculator1_convergence;
            bool is_injection0_convergence;
            bool is_injection1_convergence;

            //初期化指定用
            const float high_manual_linear_max_vel;
            const float slow_manual_linear_max_vel;
            const float manual_angular_max_vel;
            
            const bool defalt_restart_flag;
            const bool defalt_move_autonomous_flag;
            const bool defalt_injection_autonomous_flag;
            const bool defalt_emergency_flag;
            const bool defalt_slow_speed_flag;

            const bool defalt_spline_convergence;
            const bool defalt_injection_calculator0_convergence;
            const bool defalt_injection_calculator1_convergence;
            const bool defalt_injection0_convergence;
            const bool defalt_injection1_convergence;

            const int16_t can_emergency_id;
            const int16_t can_heartbeat_id;
            const int16_t can_restart_id;
            const int16_t can_linear_id;
            const int16_t can_angular_id;
            const int16_t can_main_button_id;
            const int16_t can_sub_button_id;

            const std::string er_pc;
            const std::string rr_pc;

            const std::string initial_pickup_state;
            const std::string initial_inject_state;

            //udp初期化用
            const int udp_port_state;
            const int udp_port_pole;
            const int udp_port_spline_state;

            bool start_er_main;

            bool start_flag;

            //計画機
            VelPlanner high_velPlanner_linear_x;
            VelPlanner high_velPlanner_linear_y;
            const VelPlannerLimit high_limit_linear;

            VelPlanner slow_velPlanner_linear_x;
            VelPlanner slow_velPlanner_linear_y;
            const VelPlannerLimit slow_limit_linear;

            VelPlanner velPlanner_angular_z;
            const VelPlannerLimit limit_angular;

            send_udp send;
            super_command command;

            RecvUDP joy_main;
    };
}