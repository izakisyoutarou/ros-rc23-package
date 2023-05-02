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
        gravitational_accelerastion(get_parameter("gravitational_accelerastion").as_double()),
        foundation_hight(get_parameter("foundation_hight").as_double()),
        velocity_lim_max(get_parameter("velocity_lim_max").as_double()),
        injection_angle(get_parameter("injection_angle").as_double()),
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
            _sub_pad = this->create_subscription<controller_interface_msg::msg::Pad>(
                "sub_pad_er_sub",
                _qos,
                std::bind(&InjectionParamCalculator::callback_sub_pad,this,std::placeholders::_1)
            );

            _pub_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx",_qos);
            _pub_isConvergenced = this->create_publisher<std_msgs::msg::Bool>("is_calculator_convergenced_"+to_string(mech_num),_qos);
            RCLCPP_INFO(this->get_logger(),"create injection_ER");
        }
    void InjectionParamCalculator::callback_injection(const injection_interface_msg::msg::InjectionCommand::SharedPtr msg){
        auto msg_injection = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        auto msg_yaw = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        auto msg_isConvergenced = std::make_shared<std_msgs::msg::Bool>();
        bool isConvergenced = false;
        injection_comand.distance = msg->distance;
        injection_comand.direction = msg->direction;
        injection_comand.height = msg->height;
        RCLCPP_INFO(get_logger(),"distance: %lf",injection_comand.distance);
        RCLCPP_INFO(get_logger(),"height: %lf",injection_comand.height);
        RCLCPP_INFO(get_logger(),"direction(deg): %lf",rtod(injection_comand.direction));

        isConvergenced = calculateVelocity();
        msg_isConvergenced->data = isConvergenced;
        RCLCPP_INFO(this->get_logger(),"mech_num: %d velocity: %lf[m/s]",mech_num,velocity);
        RCLCPP_INFO(this->get_logger(),"mech_num: %d direction: %lf[rad]",mech_num,injection_comand.direction);

        msg_injection->canid = 0x210 + 2*mech_num;
        msg_injection->candlc = 4;

        //送信
        uint8_t _candata[4];
        float_to_bytes(_candata, static_cast<float>(velocity));
        for(int i=0; i<msg_injection->candlc; i++) msg_injection->candata[i] = _candata[i];

        msg_yaw->canid = 0x210 + 2*mech_num + 1;
        msg_yaw->candlc = 4;
        float_to_bytes(_candata, static_cast<float>(injection_comand.direction));
        for(int i=0; i<msg_yaw->candlc; i++) msg_yaw->candata[i] = _candata[i];
        _pub_isConvergenced->publish(*msg_isConvergenced);
        if(isConvergenced){
            _pub_can->publish(*msg_injection);
            _pub_can->publish(*msg_yaw);
        }
    }
    void InjectionParamCalculator::callback_sub_pad(const controller_interface_msg::msg::Pad::SharedPtr msg){
        auto msg_injection = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        if(msg->x){
            velocity = 0.0;
            msg_injection->canid = 0x210 + 2*mech_num;
            msg_injection->candlc = 4;
            //送信
            uint8_t _candata[4];
            float_to_bytes(_candata, static_cast<float>(velocity));
            for(int i=0; i<msg_injection->candlc; i++) msg_injection->candata[i] = _candata[i];
            _pub_can->publish(*msg_injection);
        }
    }

    void InjectionParamCalculator::callback_is_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg){
        is_convergence = msg->spline_convergence;
    }
    
    bool InjectionParamCalculator::calculateVelocity(){
        bool isConvergence = false;
        double g = gravitational_accelerastion;
        double angle = dtor(injection_angle);
        double y0 = foundation_hight;
        double x = injection_comand.distance;
        double y = injection_comand.height;
        double c_tan = tan(angle);
        double c_cos = cos(angle);
        double Discriminant = x*c_tan + y0 - y;
        if(!(dtor(yow_limit[0]) < injection_comand.direction && injection_comand.direction < dtor(yow_limit[1]))){
            RCLCPP_INFO(get_logger(),"範囲外です");
            isConvergence = false;
            return isConvergence;

        }
        if(Discriminant<=0){
            RCLCPP_INFO(get_logger(),"角度が足りません");
            isConvergence = false;
            return isConvergence;
        }
        velocity = sqrt(g*x*x/(2*c_cos*c_cos*Discriminant));
        if(velocity > velocity_lim_max || velocity<=0){
            RCLCPP_INFO(get_logger(),"速度が足りません");
            isConvergence = false;
        }
        else{
            isConvergence = true;
        }
        return isConvergence;   
    }
}