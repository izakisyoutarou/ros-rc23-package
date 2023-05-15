#include "controller_interface/controller_interface_node.hpp"
#include <sys/time.h>
#include <sys/types.h>
#include <chrono>
#include <iostream>
#include <future>

using namespace utils;

namespace controller_interface
{
    using std::string;

    SmartphoneGamepad::SmartphoneGamepad(const rclcpp::NodeOptions &options) : SmartphoneGamepad("", options) {}
    SmartphoneGamepad::SmartphoneGamepad(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("controller_interface_node", name_space, options),
        limit_linear(DBL_MAX,
        get_parameter("linear_max_vel").as_double(),
        get_parameter("linear_max_acc").as_double(),
        get_parameter("linear_max_dec").as_double() ),
        limit_angular(DBL_MAX,
        dtor(get_parameter("angular_max_vel").as_double()),
        dtor(get_parameter("angular_max_acc").as_double()),
        dtor(get_parameter("angular_max_dec").as_double()) ),

        joy_main(get_parameter("port.joy_main").as_int()),

        manual_linear_max_vel(static_cast<float>(get_parameter("linear_max_vel").as_double())),
        manual_angular_max_vel(dtor(static_cast<float>(get_parameter("angular_max_vel").as_double()))),

        defalt_restart_flag(get_parameter("defalt_restart_flag").as_bool()),
        defalt_emergency_flag(get_parameter("defalt_emergency_flag").as_bool()),
        defalt_move_autonomous_flag(get_parameter("defalt_move_autonomous_flag").as_bool()),

        defalt_spline_convergence(get_parameter("defalt_spline_convergence").as_bool()),
        defalt_injection_calculator0_convergence(get_parameter("defalt_injection_calculator0_convergence").as_bool()),
        defalt_injection_calculator1_convergence(get_parameter("defalt_injection_calculator1_convergence").as_bool()),
        defalt_injection0_convergence(get_parameter("defalt_injection0_convergence").as_bool()),
        defalt_injection1_convergence(get_parameter("defalt_injection1_convergence").as_bool()),
        
        udp_port_state(get_parameter("port.robot_state").as_int()),
        udp_port_pole(get_parameter("port.pole_share").as_int()),
        udp_port_spline_state(get_parameter("port.spline_state").as_int()),

        can_emergency_id(get_parameter("canid.emergency").as_int()),
        can_heartbeat_id(get_parameter("canid.heartbeat").as_int()),
        can_restart_id(get_parameter("canid.restart").as_int()),
        can_linear_id(get_parameter("canid.linear").as_int()),
        can_angular_id(get_parameter("canid.angular").as_int()),
        can_main_button_id(get_parameter("canid.main_digital_button").as_int()),
        can_sub_button_id(get_parameter("canid.sub_digital_button").as_int()),

        er_pc(get_parameter("ip.er_pc").as_string()),
        rr_pc(get_parameter("ip.rr_pc").as_string()),

        initial_pickup_state(get_parameter("initial_pickup_state").as_string()),
        initial_inject_state(get_parameter("initial_inject_state").as_string())
        {
            const auto heartbeat_ms = this->get_parameter("heartbeat_ms").as_int();
            const auto convergence_ms = this->get_parameter("convergence_ms").as_int();

            //controller_mainからsub
            _sub_pad_main = this->create_subscription<controller_interface_msg::msg::Pad>(
                "pad_main",
                _qos,
                std::bind(&SmartphoneGamepad::callback_pad_main, this, std::placeholders::_1)
            );

            _sub_state_num_ER = this->create_subscription<std_msgs::msg::String>(
                "state_num_ER",
                _qos,
                std::bind(&SmartphoneGamepad::callback_state_num_ER, this, std::placeholders::_1)
            );

            _sub_initial_state = this->create_subscription<std_msgs::msg::String>(
                "initial_state",
                _qos,
                std::bind(&SmartphoneGamepad::callback_initial_state, this, std::placeholders::_1)
            );

            //controller_subからsub
            _sub_pad_sub = this->create_subscription<controller_interface_msg::msg::Pad>(
                "pad_sub",
                _qos,
                std::bind(&SmartphoneGamepad::callback_pad_sub, this, std::placeholders::_1)
            );

            //mainからsub
            _sub_main_injection_possible = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_201",
                _qos,
                std::bind(&SmartphoneGamepad::callback_main, this, std::placeholders::_1)
            );

            _sub_main_injection_complete = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_202",
                _qos,
                std::bind(&SmartphoneGamepad::callback_main, this, std::placeholders::_1)
            );

            //spline_pidからsub
            _sub_spline = this->create_subscription<std_msgs::msg::Bool>(
                "is_move_tracking",
                _qos,
                std::bind(&SmartphoneGamepad::callback_spline, this, std::placeholders::_1)
            );

            //injection_param_calculatorからsub
            _sub_injection_calculator_0 = this->create_subscription<std_msgs::msg::Bool>(
                "is_calculator_convergenced_0",
                _qos,
                std::bind(&SmartphoneGamepad::callback_injection_calculator_0, this, std::placeholders::_1)
            );

            _sub_injection_calculator_1 = this->create_subscription<std_msgs::msg::Bool>(
                "is_calculator_convergenced_1",
                _qos,
                std::bind(&SmartphoneGamepad::callback_injection_calculator_1, this, std::placeholders::_1)
            );

            //canusbへpub
            _pub_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

            //各nodeへ共有。
            _pub_base_control = this->create_publisher<controller_interface_msg::msg::BaseControl>("pub_base_control",_qos);
            _pub_convergence = this->create_publisher<controller_interface_msg::msg::Convergence>("pub_convergence" , _qos);
            _pub_injection = this->create_publisher<controller_interface_msg::msg::Injection>("injection_mech", _qos);

            //gazebo用のpub
            _pub_gazebo = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", _qos);

            //デフォルト値をpub.。各種、boolに初期値を代入。
            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_base_control->is_restart = defalt_restart_flag;
            msg_base_control->is_emergency = defalt_emergency_flag;
            msg_base_control->is_move_autonomous = defalt_move_autonomous_flag;
            msg_base_control->initial_state = "O";
            this->is_reset = defalt_restart_flag;
            this->is_emergency = defalt_emergency_flag;
            this->is_move_autonomous = defalt_move_autonomous_flag;
            this->initial_state = "O";
            _pub_base_control->publish(*msg_base_control);

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = can_emergency_id;
            msg_emergency->candlc = 1;
            msg_emergency->candata[0] = defalt_emergency_flag;
            _pub_canusb->publish(*msg_emergency);

            auto msg_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            msg_convergence->spline_convergence = defalt_spline_convergence;
            msg_convergence->injection_calculator0 = defalt_injection_calculator0_convergence;
            msg_convergence->injection_calculator1 = defalt_injection_calculator1_convergence;
            msg_convergence->injection0 = defalt_injection0_convergence;
            msg_convergence->injection1 = defalt_injection1_convergence;
            _pub_convergence->publish(*msg_convergence);

            //ハートビート
            _pub_heartbeat = this->create_wall_timer(
                std::chrono::milliseconds(heartbeat_ms),
                [this] {
                    auto msg_heartbeat = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_heartbeat->canid = can_heartbeat_id;
                    msg_heartbeat->candlc = 0;
                    _pub_canusb->publish(*msg_heartbeat);
                }
            );

            //convergence
            _pub_timer_convergence = this->create_wall_timer(
                std::chrono::milliseconds(convergence_ms),
                [this] {
                    auto msg_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
                    msg_convergence->spline_convergence = is_spline_convergence;
                    msg_convergence->injection_calculator0 = is_injection_calculator0_convergence;
                    msg_convergence->injection_calculator1 = is_injection_calculator1_convergence;
                    msg_convergence->injection0 = is_injection0_convergence;
                    msg_convergence->injection1 = is_injection1_convergence;
                    
                    _pub_convergence->publish(*msg_convergence);
                }
            );

            _socket_timer = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("interval_ms").as_int()),
                [this] { _recv_callback(); }
            );

            //計画機
            velPlanner_linear_x.limit(limit_linear);
            velPlanner_linear_y.limit(limit_linear);
            velPlanner_angular_z.limit(limit_angular);
        }

        void SmartphoneGamepad::callback_pad_main(const controller_interface_msg::msg::Pad::SharedPtr msg)
        {
            auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_restart->canid = can_restart_id;
            msg_restart->candlc = 0;

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = can_emergency_id;
            msg_emergency->candlc = 1;

            auto msg_btn = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_btn->canid = can_main_button_id;
            msg_btn->candlc = 8;

            auto msg_injection = std::make_shared<controller_interface_msg::msg::Injection>();

            uint8_t _candata_btn[8];

            bool robotcontrol_flag = false;//base_control(手自動、緊急、リスタート)が押されたらpubする
            bool flag_restart = false;//resertがtureをpubした後にfalseをpubする

            //l1,r1で残弾数に+1する。射出のトリガーが動いたら、残弾数が減っていくが、打ち損じがある場合があるので、その時用
            if(msg->l1)
            {
                msg_injection->injectable_rings[0] = 1;
            }

            if(msg->r1)
            {
                msg_injection->injectable_rings[1] = 1;
            }

            //l2,r2で射出機構をロックする。ロックされた射出機構は次のポール選択で選ばれない。
            if(msg->l2)
            {
                msg_injection->is_release_mech[0] = true;
            }

            if(msg->r2)
            {
                msg_injection->is_release_mech[1] = true;
            }

            //r3は足回りの手自動の切り替え。is_move_autonomousを使って、トグルになるようにしてる。ERの上物からもらう必要はない。
            if(msg->r3)
            {
                robotcontrol_flag = true;
                if(is_move_autonomous == false) is_move_autonomous = true;
                else is_move_autonomous = false;
            }

            if(msg->l3)
            {
                start_er_main = true;
            }

            //gは緊急。is_emergencyを使って、トグルになるようにしてる。
            if(msg->g)
            {
                robotcontrol_flag = true;
                is_emergency = true;
            }

            //sはリスタート。緊急と手自動のboolをfalseにしてリセットしている。
            if(msg->s)
            {
                robotcontrol_flag = true;
                flag_restart = true;
                is_move_autonomous = defalt_move_autonomous_flag;
                is_emergency = false;
                initial_state = "O";

                is_spline_convergence = defalt_spline_convergence;
                is_injection_calculator0_convergence = defalt_injection_calculator0_convergence;
                is_injection_calculator1_convergence = defalt_injection_calculator1_convergence;
                is_injection0_convergence = defalt_injection0_convergence;
                is_injection1_convergence = defalt_injection1_convergence; 
            }

            is_reset = msg->s;

            //basecontrolへの代入
            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_base_control->is_restart = is_reset;
            msg_base_control->is_emergency = is_emergency;
            msg_base_control->is_move_autonomous = is_move_autonomous;
            msg_base_control->initial_state = initial_state;

            //mainへ緊急を送る代入
            _candata_btn[0] = is_emergency;
            for(int i=0; i<msg_emergency->candlc; i++) msg_emergency->candata[i] = _candata_btn[i];

            //mainへボタン情報を送る代入
            _candata_btn[0] = msg->a;
            _candata_btn[1] = msg->b;
            _candata_btn[2] = msg->y;
            _candata_btn[3] = msg->x;
            _candata_btn[4] = msg->right;
            _candata_btn[5] = msg->down;
            _candata_btn[6] = msg->left;
            _candata_btn[7] = msg->up;
            for(int i=0; i<msg_btn->candlc; i++) msg_btn->candata[i] = _candata_btn[i];

            if(msg->a || msg->b || msg->y || msg->x || msg->right || msg->down || msg->left || msg->up)
            {
                _pub_canusb->publish(*msg_btn);
            }
            if(msg->l1 || msg->r1 || msg->l2 || msg->r2)
            {
                _pub_injection->publish(*msg_injection);
            }
            if(start_er_main)
            {
                std::future<void> fooFuture = std::async(std::launch::async, &controller_interface::SmartphoneGamepad::start_integration_async,this);
                fooFuture.wait();
                start_er_main = false;
            }
            if(msg->g)
            {
                _pub_canusb->publish(*msg_emergency);
            }
            if(robotcontrol_flag)
            {
                _pub_base_control->publish(*msg_base_control);            
            }
            if(msg->s)
            {
                _pub_canusb->publish(*msg_restart);
                _pub_canusb->publish(*msg_emergency);
            }
            if(flag_restart)
            {
                msg_base_control->is_restart = false;
                _pub_base_control->publish(*msg_base_control);
            }
        }

        void SmartphoneGamepad::callback_pad_sub(const controller_interface_msg::msg::Pad::SharedPtr msg)
        {
            auto msg_btn = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_btn->canid = can_sub_button_id;
            msg_btn->candlc = 8;

            uint8_t _candata_btn[8];

            //mainへボタン情報を送る代入
            _candata_btn[0] = msg->a;
            _candata_btn[1] = msg->b;
            _candata_btn[2] = msg->y;
            _candata_btn[3] = msg->x;
            _candata_btn[4] = msg->right;
            _candata_btn[5] = msg->down;
            _candata_btn[6] = msg->left;
            _candata_btn[7] = msg->up;
            for(int i=0; i<msg_btn->candlc; i++) msg_btn->candata[i] = _candata_btn[i];

            if(msg->a || msg->b || msg->y || msg->x || msg->right || msg->down || msg->left || msg->up)
            {
                _pub_canusb->publish(*msg_btn);
            }
        }

        void SmartphoneGamepad::callback_initial_state(const std_msgs::msg::String::SharedPtr msg)
        {
            initial_state = msg->data[0];
        }

        void SmartphoneGamepad::callback_state_num_ER(const std_msgs::msg::String::SharedPtr msg)
        {
            const unsigned char data[2] = {msg->data[0], msg->data[1]};
            command.state_num_ER(data, er_pc,udp_port_state);
        }
        
        void SmartphoneGamepad::callback_main(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg)
        {
            ///mainから射出可能司令のsub。上物の収束状況。
            is_injection0_convergence = static_cast<bool>(msg->candata[0]);
            is_injection1_convergence = static_cast<bool>(msg->candata[1]);
        }

        void SmartphoneGamepad::callback_spline(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //spline_pidから足回り収束のsub。足回りの収束状況。
            if(msg->data == false) is_spline_convergence = true;
            else is_spline_convergence = false;
        }

        void SmartphoneGamepad::callback_injection_calculator_0(const std_msgs::msg::Bool::SharedPtr msg)
        {
             //injection_calculatorから上モノ指令値計算収束のsub。上物の指令値の収束情報。
            is_injection_calculator0_convergence = msg->data;
        }

        void SmartphoneGamepad::callback_injection_calculator_1(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //injection_calculatorから上モノ指令値計算収束のsub。上物の指令値の収束情報。
            is_injection_calculator1_convergence = msg->data;
        }

        void SmartphoneGamepad::start_integration_async()
        {
            const char* char_ptr = initial_pickup_state.c_str();
            const unsigned char* pickup = reinterpret_cast<const unsigned char*>(char_ptr);

            const string initial_inject_state_with_null = initial_inject_state + '\0';
            const char* char_ptr2 = initial_inject_state_with_null.c_str();
            const unsigned char* inject = reinterpret_cast<const unsigned char*>(char_ptr2);
                  
            command.state_num_ER(pickup, er_pc,udp_port_state);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            command.state_num_ER(inject, er_pc,udp_port_state);
        }

        void SmartphoneGamepad::_recv_callback()
        {
            if(joy_main.is_recved())
            {
                unsigned char data[16];
                _recv_joy_main(joy_main.data(data, sizeof(data)));
            }
        }

        void SmartphoneGamepad::_recv_joy_main(const unsigned char data[16])
        {
            float values[4];
            memcpy(values, data, sizeof(float)*4);

            auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = can_linear_id;
            msg_linear->candlc = 8;

            auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = can_angular_id;
            msg_angular->candlc = 4;

            auto msg_gazebo = std::make_shared<geometry_msgs::msg::Twist>();

            bool flag_move_autonomous = false;

            uint8_t _candata_joy[8];

            if(is_move_autonomous == false)
            {
                velPlanner_linear_x.vel(static_cast<double>(values[1]));//unityとロボットにおける。xとyが違うので逆にしている。
                velPlanner_linear_y.vel(static_cast<double>(-values[0]));
                velPlanner_angular_z.vel(static_cast<double>(-values[2]));

                velPlanner_linear_x.cycle();
                velPlanner_linear_y.cycle();
                velPlanner_angular_z.cycle();

                float_to_bytes(_candata_joy, static_cast<float>(velPlanner_linear_x.vel()) * manual_linear_max_vel);
                float_to_bytes(_candata_joy+4, static_cast<float>(velPlanner_linear_y.vel()) * manual_linear_max_vel);
                for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];

                float_to_bytes(_candata_joy, static_cast<float>(velPlanner_angular_z.vel()) * manual_angular_max_vel);
                for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];

                _pub_canusb->publish(*msg_linear);
                _pub_canusb->publish(*msg_angular);

                msg_gazebo->linear.x = velPlanner_linear_x.vel();
                msg_gazebo->linear.y = velPlanner_linear_y.vel();
                msg_gazebo->angular.z = velPlanner_angular_z.vel();
                //_pub_gazebo->publish(*msg_gazebo);

                flag_move_autonomous = true;
            }
        }
}
