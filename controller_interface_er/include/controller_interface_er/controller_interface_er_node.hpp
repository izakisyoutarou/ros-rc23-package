#pragma once
#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <string>
//使うmsg
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "controller_interface_msg/msg/sub_pad.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
//他のpkg
#include "utilities/can_utils.hpp"
#include "utilities/utils.hpp"
#include "trapezoidal_velocity_planner.hpp"
#include "visibility_control.h"
//UDP
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>//memcpyのため

namespace controller_interface
{
    using VelPlanner = velocity_planner::trapezoidal_velocity_planner::TrapezoidalVelocityPlanner;
    using VelPlannerLimit = velocity_planner::trapezoidal_velocity_planner::Limit_t;

    class SmartphoneGamepad : public rclcpp::Node
    {
        public:
            CONTROLLER_INTERFACE_ER_PUBLIC
            explicit SmartphoneGamepad(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            
            CONTROLLER_INTERFACE_ER_PUBLIC
            explicit SmartphoneGamepad(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        private:
            //ER_mainのcontrollerから
            rclcpp::Subscription<controller_interface_msg::msg::SubPad>::SharedPtr _sub_pad_main;
            //ER_subのcontrollerから
            rclcpp::Subscription<controller_interface_msg::msg::SubPad>::SharedPtr _sub_pad_sub;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_scrn_pole_0;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_scrn_pole_1;

            //mainボードから
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_main_injection_possible;
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_main_injection_complete;

            //spline_pidから
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_spline;

            //injection_param_calculatorから
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_injection_calculator_0;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_injection_calculator_1;

            //CanUsbへ
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb;

            //controllerへ
            rclcpp::Publisher<controller_interface_msg::msg::Convergence>::SharedPtr _pub_convergence;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_scrn_string;

            //各nodeへリスタートと手自動の切り替えをpub
            rclcpp::Publisher<controller_interface_msg::msg::BaseControl>::SharedPtr _pub_base_control;

            //gazebo_simulator用のpub
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_gazebo;

            //timer
            rclcpp::TimerBase::SharedPtr _pub_heartbeat;
            rclcpp::TimerBase::SharedPtr _move_injection_heteronomy;

            //QoS
            rclcpp::QoS _qos = rclcpp::QoS(10);

            //controllerからのcallback
            void callback_pad_main(const controller_interface_msg::msg::SubPad::SharedPtr msg);
            void callback_pad_sub(const controller_interface_msg::msg::SubPad::SharedPtr msg);
            void callback_scrn_pole(const std_msgs::msg::String::SharedPtr msg);

            void callback_udp_main(int sockfd);
            void callback_udp_sub(int sockfd);

            //mainからのcallback
            void callback_main(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            void callback_injection_complete(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);

            //splineからのcallback
            void callback_spline(const std_msgs::msg::Bool::SharedPtr msg);

            //injection_param_calculatorからのcallback
            void callback_injection_calculator_0(const std_msgs::msg::Bool::SharedPtr msg);
            void callback_injection_calculator_1(const std_msgs::msg::Bool::SharedPtr msg);

            //上物と足回り手動のtimerによる周期関数
            void callback_move_injection_heteronomy();
            
            //base_control用
            bool is_reset = false;
            bool is_emergency = false;
            bool is_move_autonomous = false;
            bool is_injection_autonomous = false;
            int injection_mec = 0;

            //convergence用
            bool is_spline_convergence;
            bool is_injection_calculator0_convergence;
            bool is_injection_calculator1_convergence;
            bool is_injection0_convergence;
            bool is_injection1_convergence;

            //初期化指定用
            const float manual_linear_max_vel;
            const float manual_angular_max_vel;
            const float manual_injection_max_vel;
            const float defalt_pitch;
            const bool defalt_restart_flag;
            const bool defalt_move_autonomous_flag;
            const bool defalt_injection_autonomous_flag;
            const bool defalt_emergency_flag;
            const bool defalt_injection_mec;
            const int udp_port_main;
            const int udp_port_sub;
            const int udp_timeout_ms;

            //Joyの値を格納(main)
            float analog_l_x_main = 0.0f;
            float analog_l_y_main = 0.0f;
            float analog_r_x_main = 0.0f;
            float analog_r_y_main = 0.0f;

            //Joyの値を格納(sub)
            float analog_l_x_sub = 0.0f;
            float analog_l_y_sub = 0.0f;
            float analog_r_x_sub = 0.0f;
            float analog_r_y_sub = 0.0f;

            //UDP用
            int sockfd, n;
            socklen_t clilen;
            char* buffer = new char[16];
            struct sockaddr_in servaddr, cliaddr;
            std::thread udp_thread_;

            int sockfd2, n2;
            socklen_t clilen2;
            char* buffer2 = new char[16];
            struct sockaddr_in servaddr2, cliaddr2;
            std::thread udp_thread_2;

            //計画機
            VelPlanner velPlanner_linear_x;
            VelPlanner velPlanner_linear_y;
            const VelPlannerLimit limit_linear;

            VelPlanner velPlanner_angular_z;
            const VelPlannerLimit limit_angular;

            VelPlanner velPlanner_injection_v;
            const VelPlannerLimit limit_injection;
    };
}