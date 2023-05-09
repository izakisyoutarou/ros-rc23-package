#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "controller_interface_msg/msg/pad.hpp"
#include "injection_interface_msg/msg/injection_command.hpp"
#include "injection_param_calculator/my_visibility.h"

namespace injection_param_calculator{
    class InjectionParamCalculator : public rclcpp::Node{
        public:
            INJECTION_PARAM_CALCULATOR_PUBLIC
            explicit InjectionParamCalculator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(), const int mech_num=0);

            INJECTION_PARAM_CALCULATOR_PUBLIC
            explicit InjectionParamCalculator(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions(), const int mech_num=0);

        private:
            rclcpp::Subscription<injection_interface_msg::msg::InjectionCommand>::SharedPtr _sub_injection_command;
            rclcpp::QoS _qos = rclcpp::QoS(10);

            rclcpp::Subscription<controller_interface_msg::msg::Convergence>::SharedPtr _sub_is_convergence;

            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_can;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_isConvergenced;

            void callback_injection(const injection_interface_msg::msg::InjectionCommand::SharedPtr msg);
            void callback_is_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg);
            void callback_sub_pad(const controller_interface_msg::msg::Pad::SharedPtr msg);
            bool calculateVelocity();
            double calculateFirstVelocity();
            double f(double v0);
            double diff(double v0);

            injection_interface_msg::msg::InjectionCommand injection_comand;
            double velocity;

            const int mech_num;
            const std::vector<double> yow_limit;  //旋回角の最小最大
            const double mass;
            const double gravitational_accelerastion; //重力加速度
            const double air_resistance;
            const double foundation_hight;    //射出機構の地面からの高さ
            const double velocity_lim_max;  //最大初速度
            const double injection_angle;   //射出角度
            const double first_velocity;    //初速度
            const double first_velocity_low; //初速度(近いポールを狙う時)
            const double angle_bound;   //初速度の境界角度
            const int max_loop; //ニュートン法の最大数
            const double eps = 1e-6;
            bool is_convergence;
    };
}