#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
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

            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_can;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_isConvergenced;
            //rclcpp::Publisher<injection_interface_msg::msg::InjectionCommand>::SharedPtr _pub_test_injection;

            void callback_injection(const injection_interface_msg::msg::InjectionCommand::SharedPtr msg);

            double f(double v0);
            double diff(double v0);
            void calculateElevation();
            bool calculateVelocity();
            std::string int_to_string(int mech_num);

            injection_interface_msg::msg::InjectionCommand injection_comand;
            double velocity;
            double elevation;
            double direction;

            const int mech_num;
            const std::vector<double> yow_limit;  //旋回角の最小最大
            const std::vector<double> pitch_limit;  //射角の最小最大
            const double ring_weight; //リングの質量
            const double gravitational_accelerastion; //重力加速度
            const double air_resistance_coefficient;  //空気抵抗係数
            const double injection_length;    //射出機構の長さ
            const double foundation_hight;    //射出機構の地面からの高さ
            const double calculat_first_velocity; //初速度の初期値
            const double velocity_lim_max;    //最大初速度
            const double angle_choice;    //リングが入りやすい角度
            const double angle_bounds;    //角度の境界
            const int max_loop;   //ニュートン法の最大繰り返し数
            const double eps = 1e-6;
    };
}