#include "injection_param_calculator/injection_param_calculator.hpp"
#include "utilities/utils.hpp"
#include "utilities/can_utils.hpp"

using namespace std;
using namespace utils;
namespace injection_param_calculator{
    InjectionParamCalculator::InjectionParamCalculator(const rclcpp::NodeOptions &options, const int mech_num) : InjectionParamCalculator("",options, mech_num){}
    InjectionParamCalculator::InjectionParamCalculator(const std::string &name_space, const rclcpp::NodeOptions &options, const int mech_num)
        :rclcpp::Node("injection_param_calculator_node" ,name_space,options),
        mech_num(mech_num),
        mass(get_parameter("mass").as_double()),
        gravitational_accelerastion(get_parameter("gravitational_accelerastion").as_double()),
        air_resistance(get_parameter("air_resistance").as_double()),
        foundation_hight(get_parameter("foundation_hight").as_double()),
        velocity_lim_max(get_parameter("velocity_lim_max").as_double()),
        injection_angle(get_parameter("injection_angle").as_double()),
        max_loop(get_parameter("max_loop").as_int()),
        singular_point_coefficient(get_parameter("singular_point_coefficient").as_double_array()),
        yow_limit(get_parameter("yow_limit_m"+to_string(mech_num)).as_double_array())
        {
            _sub_injection_command = this->create_subscription<injection_interface_msg::msg::InjectionCommand>(
                "injection_command_m"+to_string(mech_num),
                _qos,
                std::bind(&InjectionParamCalculator::callback_injection,this,std::placeholders::_1)
            );
            _sub_is_convergence = this->create_subscription<controller_interface_msg::msg::Convergence>(
                "pub_convergence",
                _qos,
                std::bind(&InjectionParamCalculator::callback_is_convergence,this,std::placeholders::_1)
            );

            _pub_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx",_qos);
            _pub_isConvergenced = this->create_publisher<std_msgs::msg::Bool>("is_calculator_convergenced_"+to_string(mech_num),_qos);
            RCLCPP_INFO(this->get_logger(),"create injection_ER");
        }
    void InjectionParamCalculator::callback_injection(const injection_interface_msg::msg::InjectionCommand::SharedPtr msg){
        auto msg_injection = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        auto msg_isConvergenced = std::make_shared<std_msgs::msg::Bool>();
        bool isConvergenced = false;
        injection_comand.distance = msg->distance;
        injection_comand.direction = msg->direction;
        injection_comand.height = msg->height;
        injection_comand.velocity_gain = msg->velocity_gain;

        isConvergenced = calculateVelocity();        
        msg_isConvergenced->data = isConvergenced;

        msg_injection->canid = 0x210 + mech_num;
        msg_injection->candlc = 8;

        //送信
        uint8_t _candata[8];
        float_to_bytes(_candata, static_cast<float>(velocity*injection_comand.velocity_gain));
        float_to_bytes(_candata+4, static_cast<float>(injection_comand.direction));
        for(int i=0; i<msg_injection->candlc; i++) msg_injection->candata[i] = _candata[i];
        _pub_isConvergenced->publish(*msg_isConvergenced);

        if(isConvergenced){
            RCLCPP_INFO(get_logger(),"mech_num: %d 計算が収束しました",mech_num);
            _pub_can->publish(*msg_injection);
        }
    }

    double InjectionParamCalculator::calculateFirstVelocity(){
        double first_velocity;
        first_velocity = singular_point_coefficient[0]*injection_comand.distance + singular_point_coefficient[1];
        first_velocity = round(first_velocity);
        return first_velocity;
    }

    void InjectionParamCalculator::callback_is_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg){
        is_convergence = msg->spline_convergence;
    }
    
    bool InjectionParamCalculator::calculateVelocity(){
        bool isConvergence = false;
        bool isAiming = false;
        int num_loop = 0;
        double old_velocity = calculateFirstVelocity();
        if(!(yow_limit[0] < injection_comand.direction && injection_comand.direction < yow_limit[1])){
            RCLCPP_INFO(get_logger(),"mech_num: %d 範囲外です!",mech_num);
            isConvergence = false;
            return isConvergence;
        }
        while (!isAiming){
            double new_velocity = old_velocity - f(old_velocity)/diff(old_velocity);
            if(fabs(new_velocity - old_velocity)<eps && 0 < new_velocity && new_velocity < velocity_lim_max){
                velocity = new_velocity;
                isAiming = true;
                isConvergence = true;
                break;
            }
            old_velocity = new_velocity;
            num_loop++;
            if(num_loop>max_loop){
                isAiming = false;
                isConvergence = false;
                RCLCPP_INFO(get_logger(),"mech_num: %d 発散しました!",mech_num);
                break;
            }
        }
        return isConvergence;
    }
    double InjectionParamCalculator::f(double v0){
        double m = mass;
        double g = gravitational_accelerastion;
        double k = air_resistance;
        double y0 = foundation_hight;
        double angle = dtor(injection_angle);
        double c_cos = cos(angle);
        double c_tan = tan(angle);
        double x = injection_comand.distance;
        double y = injection_comand.height;
        return x*c_tan + m*g*x/(k*v0*c_cos) + m*m*g/(k*k)*log(abs(1.0-k*x/(m*v0*c_cos))) + y0 - y;
    }
    double InjectionParamCalculator::diff(double v0){
        return (f(v0 + eps) - f(v0 - eps))/(2.0*eps);
    }
}