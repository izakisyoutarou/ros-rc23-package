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
        ring_weight(get_parameter("ring_weight").as_double()),
        gravitational_accelerastion(get_parameter("gravitational_accelerastion").as_double()),
        air_resistance_coefficient(get_parameter("air_resistance_coefficient").as_double()),
        injection_length(get_parameter("injection_length").as_double()),
        foundation_hight(get_parameter("foundation_hight").as_double()),
        calculat_first_velocity(get_parameter("calculat_first_velocity").as_double()),
        calculate_first_velocity_low(get_parameter("calculate_first_velocity_low").as_double()),
        velocity_lim_max(get_parameter("velocity_lim_max").as_double()),
        angle_choice(get_parameter("angle_choice").as_double()),
        angle_bounds(get_parameter("angle_bounds").as_double()),
        max_loop(get_parameter("max_loop").as_int()),
        yow_limit(get_parameter("yow_limit_m"+to_string(mech_num)).as_double_array()),
        pitch_limit(get_parameter("pitch_limit").as_double_array())
        {
            _sub_injection_command = this->create_subscription<injection_interface_msg::msg::InjectionCommand>(
                "injection_command_m"+to_string(mech_num),
                _qos,
                std::bind(&InjectionParamCalculator::callback_injection,this,std::placeholders::_1)
            );
            _sub_is_move_tracing = this->create_subscription<std_msgs::msg::Bool>(
                "is_move_tracking",
                _qos,
                std::bind(&InjectionParamCalculator::callback_is_move_tracing,this,std::placeholders::_1)
            );

            _pub_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx",_qos);
            _pub_isConvergenced = this->create_publisher<std_msgs::msg::Bool>("is_calculator_convergenced_"+to_string(mech_num),_qos);
            RCLCPP_INFO(this->get_logger(),"create injection_"+to_string(mech_num));
        }
    void InjectionParamCalculator::callback_injection(const injection_interface_msg::msg::InjectionCommand::SharedPtr msg){
        auto msg_injection = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        auto msg_yaw = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        auto msg_isConvergenced = std::make_shared<std_msgs::msg::Bool>();
        bool isConvergenced = false;
        injection_comand.distance = msg->distance;
        injection_comand.direction = msg->direction;
        injection_comand.height = msg->height;
        RCLCPP_INFO(get_logger(),"distance: %lf",msg->distance);
        RCLCPP_INFO(get_logger(),"height: %lf",msg->height);
        RCLCPP_INFO(get_logger(),"direction: %lf",msg->direction);

        calculateElevation();
        isConvergenced = calculateVelocity();
        msg_isConvergenced->data = isConvergenced;
        RCLCPP_INFO(this->get_logger(),"mech_num: %d velocity: %lf[m/s] elevation: %lf[rad]",mech_num,velocity, elevation);
        RCLCPP_INFO(this->get_logger(),"mech_num: %d direction: %lf[rad]",mech_num,injection_comand.direction);

        msg_injection->canid = 0x210 + 2*mech_num;
        msg_injection->candlc = 8;

        //送信
        uint8_t _candata[8];
        float_to_bytes(_candata, static_cast<float>(elevation));
        float_to_bytes(_candata+4, static_cast<float>(velocity));
        if(is_move_tracking){
            float_to_bytes(_candata+4, 0);
        }
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

    void InjectionParamCalculator::calculateElevation(){
        double pitch = atan2(injection_comand.height,injection_comand.distance);
        if(dtor(angle_bounds) > pitch){
            elevation = dtor(angle_choice);
        }
        else{
            elevation = dtor(pitch_limit[1]);
        }
    }

    void InjectionParamCalculator::callback_is_move_tracing(const std_msgs::msg::Bool::SharedPtr msg)
    {
        is_move_tracking = msg->data;
    }

    bool InjectionParamCalculator::calculateVelocity(){
        int num_loop = 0;
        double old_velocity = calculateFirstVelocity();
        RCLCPP_INFO(get_logger(),"first_velocity: %lf",old_velocity);
        
        bool isConvergenced = false;
        bool isAiming = false;
        while(!isAiming){
            if(!(dtor(yow_limit[0]) <= injection_comand.direction && injection_comand.direction <= dtor(yow_limit[1]))){
                isConvergenced = false;
                RCLCPP_INFO(this->get_logger(),"mech_num: %d 範囲外です!!",mech_num,injection_comand.direction);
                velocity = 0.0;
                break;
            }
            double new_velocity = old_velocity -f(old_velocity)/diff(old_velocity);
            if(abs(new_velocity-old_velocity)<eps && 0 < new_velocity && new_velocity < velocity_lim_max){
                isAiming = true;
                isConvergenced=true;
                velocity = new_velocity;
            }
            old_velocity = new_velocity;
            num_loop++;
            
            if(num_loop>max_loop){
                isAiming=false;
                isConvergenced=false;
                velocity = 0.0;
                RCLCPP_INFO(this->get_logger(),"mech_num: %d 発散しました",mech_num);
                break;
            }
        }

        return isConvergenced;
    }
    double InjectionParamCalculator::calculateFirstVelocity(){
        double c_sin = sin(elevation);
        double c_cos = cos(elevation);
        double c_tan = tan(elevation);
        double x = injection_comand.distance - injection_length*c_cos;
        double y = injection_comand.height -(foundation_hight + injection_length*c_sin);
        double first_velocity = sqrt(gravitational_accelerastion*x*x/(2*c_cos*c_cos*(x*c_tan - y)));
    }
    double InjectionParamCalculator::f(double v0){
        double c_sin = sin(elevation);
        double c_cos = cos(elevation);
        double c_tan = tan(elevation);
        double m = ring_weight;
        double g = gravitational_accelerastion;
        double k = air_resistance_coefficient;
        double l = injection_length;
        double y0 = foundation_hight;
        double x = injection_comand.distance - l*c_cos;
        double y = injection_comand.height -(y0 + l*c_sin);
        return x*c_tan + m*g*x/(k*v0*c_cos) + m*m*g/(k*k)*log(abs(1-k*x/(m*v0*c_cos))) - y;
    }
    double InjectionParamCalculator::diff(double v0){
        return (f(v0 + eps) - f(v0 - eps))/(2*eps);
    }


}