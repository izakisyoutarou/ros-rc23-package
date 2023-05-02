#include "controller_interface/controller_interface_node.hpp"
#include <sys/time.h>
#include <sys/types.h>

using namespace utils;

#define IP_ER_PC "192.168.1.2"
#define IP_RR_PC "192.168.1.11"

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
        limit_injection(DBL_MAX,
        dtor(get_parameter("injection_max_vel").as_double()),
        dtor(get_parameter("injection_max_acc").as_double()),
        dtor(get_parameter("injection_max_dec").as_double()) ),
        joy_main(get_parameter("port.joy_main").as_int()),
        joy_sub(get_parameter("port.joy_sub").as_int()),
        pole(get_parameter("udp_port_pole_execution").as_int()),
        defalt_pitch(static_cast<float>(get_parameter("defalt_pitch").as_double())),
        manual_linear_max_vel(static_cast<float>(get_parameter("linear_max_vel").as_double())),
        manual_angular_max_vel(dtor(static_cast<float>(get_parameter("angular_max_vel").as_double()))),
        manual_injection_max_vel(dtor(static_cast<float>(get_parameter("injection_max_vel").as_double()))),
        defalt_restart_flag(get_parameter("defalt_restart_flag").as_bool()),
        defalt_emergency_flag(get_parameter("defalt_emergency_flag").as_bool()),
        defalt_move_autonomous_flag(get_parameter("defalt_move_autonomous_flag").as_bool()),
        defalt_injection_autonomous_flag(get_parameter("defalt_injection_autonomous_flag").as_bool()),
        defalt_injection_mec(get_parameter("defalt_injection_mec").as_int()),
        udp_port_pole_execution(get_parameter("udp_port_pole_execution").as_int()),
        udp_port_state_num_er(get_parameter("udp_port_state_num_er").as_int()),
        udp_port_state_num_rr(get_parameter("udp_port_state_num_rr").as_int()),
        udp_port_pole(get_parameter("udp_port_pole").as_int())
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

            _sub_state_num_RR = this->create_subscription<std_msgs::msg::String>(
                "state_num_RR",
                _qos,
                std::bind(&SmartphoneGamepad::callback_state_num_RR, this, std::placeholders::_1)
            );

            _sub_initial_state = this->create_subscription<std_msgs::msg::String>(
                "initial_state",
                _qos,
                std::bind(&SmartphoneGamepad::callback_initial_state, this, std::placeholders::_1)
            );

            _sub_pad_sub = this->create_subscription<controller_interface_msg::msg::Pad>(
                "pad_sub",
                _qos,
                std::bind(&SmartphoneGamepad::callback_pad_sub, this, std::placeholders::_1)
            );

            _sub_pole = this->create_subscription<controller_interface_msg::msg::Pole>(
                "pole",
                _qos,
                std::bind(&SmartphoneGamepad::callback_pole, this, std::placeholders::_1)
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

            //controllerへpub
            _pub_convergence = this->create_publisher<controller_interface_msg::msg::Convergence>("pub_convergence" , _qos);
            _pub_scrn_pole = this->create_publisher<controller_interface_msg::msg::Pole>("scrn_pole", _qos);

            //各nodeへリスタートと手自動の切り替えをpub。
            _pub_base_control = this->create_publisher<controller_interface_msg::msg::BaseControl>("pub_base_control",_qos);

            //gazebo用のpub
            _pub_gazebo = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", _qos);

            //デフォルト値をpub.。各種、boolに初期値を代入。
            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_base_control->is_restart = defalt_restart_flag;
            msg_base_control->is_emergency = defalt_emergency_flag;
            msg_base_control->is_move_autonomous = defalt_move_autonomous_flag;
            msg_base_control->is_injection_autonomous = defalt_injection_autonomous_flag;
            msg_base_control->injection_mec = defalt_injection_mec;
            this->is_reset = defalt_restart_flag;
            this->is_emergency = defalt_emergency_flag;
            this->is_move_autonomous = defalt_move_autonomous_flag;
            this->is_injection_autonomous = defalt_injection_autonomous_flag;
            this->injection_mec = defalt_injection_mec;
            _pub_base_control->publish(*msg_base_control);

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = 0x000;
            msg_emergency->candlc = 1;
            msg_emergency->candata[0] = defalt_emergency_flag;
            _pub_canusb->publish(*msg_emergency);

            //ハートビート
            _pub_heartbeat = this->create_wall_timer(
                std::chrono::milliseconds(heartbeat_ms),
                [this] {
                    auto msg_heartbeat = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_heartbeat->canid = 0x001;
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

            _pole_timer = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("pole_ms").as_int()),
                [this] { pole_integration(); }
            );

            //計画機
            velPlanner_linear_x.limit(limit_linear);
            velPlanner_linear_y.limit(limit_linear);
            velPlanner_angular_z.limit(limit_angular);
            velPlanner_injection_v.limit(limit_injection);
        }

        void SmartphoneGamepad::callback_pad_main(const controller_interface_msg::msg::Pad::SharedPtr msg)
        {
            auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_restart->canid = 0x002;
            msg_restart->candlc = 0;

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = 0x000;
            msg_emergency->candlc = 1;

            auto msg_btn = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_btn->canid = 0x220;
            msg_btn->candlc = 8;

            uint8_t _candata_btn[8];

            bool robotcontrol_flag = false;//base_control(手自動、緊急、リスタート)が押されたらpubする
            bool flag_restart = false;//resertがtureをpubした後にfalseをpubする

            //r3は足回りの手自動の切り替え。is_move_autonomousを使って、トグルになるようにしてる。ERの上物からもらう必要はない。
            //ERの上物の場合は、上物の切り替えに当てている。

            if(msg->r3)
            {
                robotcontrol_flag = true;
                if(is_move_autonomous == false) is_move_autonomous = true;
                else is_move_autonomous = false;
            }

            //gは緊急。is_emergencyを使って、トグルになるようにしてる。
            if(msg->g)
            {
                robotcontrol_flag = true;
                if(is_emergency == false) is_emergency = true;
                else is_emergency = false;
            }

            //sはリスタート。緊急と手自動のboolをfalseにしてリセットしている。
            if(msg->s)
            {
                robotcontrol_flag = true;
                flag_restart = true;
                is_move_autonomous = defalt_move_autonomous_flag;
                is_injection_autonomous = defalt_injection_autonomous_flag;
                is_emergency = defalt_emergency_flag;
                injection_mec = defalt_injection_mec;
            }

            is_reset = msg->s;

            //basecontrolへの代入
            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_base_control->is_restart = is_reset;
            msg_base_control->is_emergency = is_emergency;
            msg_base_control->is_move_autonomous = is_move_autonomous;
            msg_base_control->is_injection_autonomous = is_injection_autonomous;
            msg_base_control->injection_mec = injection_mec;
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
            if(msg->g)_pub_canusb->publish(*msg_emergency);
            if(robotcontrol_flag)
            {
                _pub_base_control->publish(*msg_base_control);            
            }
            if(msg->s)
            {
                _pub_canusb->publish(*msg_restart);
            }
            if(flag_restart)
            {
                msg_base_control->is_restart = false;
                _pub_base_control->publish(*msg_base_control);
            }
        }

        void SmartphoneGamepad::callback_pad_sub(const controller_interface_msg::msg::Pad::SharedPtr msg)
        {
            auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_restart->canid = 0x002;
            msg_restart->candlc = 0;

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = 0x000;
            msg_emergency->candlc = 1;

            auto msg_injection = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_injection->canid = 0x200;
            msg_injection->candlc = 2;

            auto msg_btn = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_btn->canid = 0x221;
            msg_btn->candlc = 8;

            auto msg_scrn_pole_restart = std::make_shared<controller_interface_msg::msg::Pole>(); 

            uint8_t _candata_btn[8];

            bool robotcontrol_flag = false;//base_control(手自動、緊急、リスタート)が押されたらpubする
            bool flag_restart = false;//resertがtureをpubした後にfalseをpubする
            bool flag_injection0 = false;//左の発射機構の最終射出許可
            bool flag_injection1 = false;//右の発射機構の最終射出許可

            //r3は足回りの手自動の切り替え。is_move_autonomousを使って、トグルになるようにしてる。ERの上物からもらう必要はない。
            //ERの上物の場合は、上物の切り替えに当てている。

            if(msg->r3)
            {
                robotcontrol_flag = true;
                if(injection_mec == 0) 
                {
                    injection_mec = 1;
                }
                else if(injection_mec != 0)  injection_mec = 0;
            }

            //l3は上物の手自動の切り替え。is_injection_autonomousを使って、トグルになるようにしてる。ERの足回りからもらう必要はない
            if(msg->l3)
            {
                robotcontrol_flag = true;
                if(is_injection_autonomous == false) is_injection_autonomous = true;
                else is_injection_autonomous = false;
            }

            //gは緊急。is_emergencyを使って、トグルになるようにしてる。
            if(msg->g)
            {
                robotcontrol_flag = true;
                if(is_emergency == false) is_emergency = true;
                else is_emergency = false;
            }

            //sはリスタート。緊急と手自動のboolをfalseにしてリセットしている。
            if(msg->s)
            {
                msg_scrn_pole_restart->a = false;
                msg_scrn_pole_restart->b = false;
                msg_scrn_pole_restart->c = false;
                msg_scrn_pole_restart->d = false;
                msg_scrn_pole_restart->e = false;
                msg_scrn_pole_restart->f = false;
                msg_scrn_pole_restart->g = false;
                msg_scrn_pole_restart->h = false;
                msg_scrn_pole_restart->i = false;
                msg_scrn_pole_restart->j = false;
                msg_scrn_pole_restart->k = false;
            }

            //l2が左、r2が右の発射機構のトリガー。
            //それぞれ、発射されたら収束がfalseにするようにしている。
            if(msg->l2)
            {
                if(is_spline_convergence && is_injection0_convergence && is_injection_calculator0_convergence)
                {
                    flag_injection0 = true;
                    is_injection0_convergence = false;
                    is_injection_calculator0_convergence = false;
                }
            }

            if(msg->r2)
            {
                if(is_spline_convergence && is_injection1_convergence && is_injection_calculator1_convergence)
                {
                    flag_injection1 = true;
                    is_injection1_convergence = false;
                    is_injection_calculator1_convergence = false;
                }
            }

            is_reset = msg->s;

            //basecontrolへの代入
            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_base_control->is_restart = is_reset;
            msg_base_control->is_emergency = is_emergency;
            msg_base_control->is_move_autonomous = is_move_autonomous;
            msg_base_control->is_injection_autonomous = is_injection_autonomous;
            msg_base_control->injection_mec = injection_mec;

            //mainへ緊急を送る代入
            _candata_btn[0] = is_emergency;
            for(int i=0; i<msg_emergency->candlc; i++) msg_emergency->candata[i] = _candata_btn[i];

            //mainへ射出司令を送る代入
            _candata_btn[0] = flag_injection0;
            _candata_btn[1] = flag_injection1;
            for(int i=0; i<msg_injection->candlc; i++) msg_injection->candata[i] = _candata_btn[i];

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
            if(msg->g)_pub_canusb->publish(*msg_emergency);
            if(flag_injection0 || flag_injection1)_pub_canusb->publish(*msg_injection);
            if(robotcontrol_flag)
            {
                _pub_base_control->publish(*msg_base_control);
            }
            if(msg->s) _pub_scrn_pole->publish(*msg_scrn_pole_restart);
            if(flag_restart)
            {
                msg_base_control->is_restart = false;
                _pub_base_control->publish(*msg_base_control);
            }
        }

        void SmartphoneGamepad::callback_initial_state(const std_msgs::msg::String::SharedPtr msg)
        {
            initial_state = msg->data[0];
        }

        void SmartphoneGamepad::callback_state_num_ER(const std_msgs::msg::String::SharedPtr msg)
        {
            const unsigned char data[2] = {msg->data[0], msg->data[1]};
            command.state_num_ER(data, udp_port_state_num_er);
        }

        void SmartphoneGamepad::callback_state_num_RR(const std_msgs::msg::String::SharedPtr msg)
        {
            const unsigned char data[2] = {msg->data[0], msg->data[1]};
            command.state_num_RR(data, udp_port_state_num_rr);
        }

        void SmartphoneGamepad::callback_pole(const controller_interface_msg::msg::Pole::SharedPtr msg)
        {
            pole_a[0] = msg->a;
            pole_a[1] = msg->b;
            pole_a[2] = msg->c;
            pole_a[3] = msg->d;
            pole_a[4] = msg->e;
            pole_a[5] = msg->f;
            pole_a[6] = msg->g;
            pole_a[7] = msg->h;
            pole_a[8] = msg->i;
            pole_a[9] = msg->j;
            pole_a[10] = msg->k;
        }

        void SmartphoneGamepad::pole_integration()
        {
            auto msg_scrn_pole = std::make_shared<controller_interface_msg::msg::Pole>(); 
            unsigned char pole[11];
            pole[0] = static_cast<char>(pole_a[0]);
            pole[1] = static_cast<char>(pole_a[1]);
            pole[2] = static_cast<char>(pole_a[2]);
            pole[3] = static_cast<char>(pole_a[3]);
            pole[4] = static_cast<char>(pole_a[4]);
            pole[5] = static_cast<char>(pole_a[5]);
            pole[6] = static_cast<char>(pole_a[6]);
            pole[7] = static_cast<char>(pole_a[7]);
            pole[8] = static_cast<char>(pole_a[8]);
            pole[9] = static_cast<char>(pole_a[9]);
            pole[10] = static_cast<char>(pole_a[10]);

            msg_scrn_pole->a = pole_a[0];
            msg_scrn_pole->b = pole_a[1];
            msg_scrn_pole->c = pole_a[2];
            msg_scrn_pole->d = pole_a[3];
            msg_scrn_pole->e = pole_a[4];
            msg_scrn_pole->f = pole_a[5];
            msg_scrn_pole->g = pole_a[6];
            msg_scrn_pole->h = pole_a[7];
            msg_scrn_pole->i = pole_a[8];
            msg_scrn_pole->j = pole_a[9];
            msg_scrn_pole->k = pole_a[10];

            command.pole_ER(pole, udp_port_pole);
            command.pole_RR(pole, udp_port_pole);
            send.send(pole, sizeof(pole), IP_RR_PC, udp_port_pole_execution);

            _pub_scrn_pole->publish(*msg_scrn_pole);
        }

        void SmartphoneGamepad::_recv_callback()
        {
            if(joy_main.is_recved())
            {
                unsigned char data[16];
                _recv_joy_main(joy_main.data(data, sizeof(data)));
            }
            if(joy_sub.is_recved())
            {
                unsigned char data[16];
                _recv_joy_sub(joy_sub.data(data, sizeof(data)));
            }
            if(pole.is_recved())
            {
                unsigned char data[11];
                _recv_pole(pole.data(data, sizeof(data)));
            }
        }

        void SmartphoneGamepad::_recv_joy_main(const unsigned char data[16])
        {
            float values[4];
            memcpy(values, data, sizeof(float)*4);

            auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = 0x100;
            msg_linear->candlc = 8;

            auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = 0x101;
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

        void SmartphoneGamepad::_recv_joy_sub(const unsigned char data[16])
        {
            float values[4];
            memcpy(values, data, sizeof(float)*4);

            auto msg_l_elevation_velocity = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_l_elevation_velocity->canid = 0x210;
            msg_l_elevation_velocity->candlc = 8;

            auto msg_l_yaw = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_l_yaw->canid = 0x211;
            msg_l_yaw->candlc = 4;

            auto msg_r_elevation_velocity = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_r_elevation_velocity->canid = 0x212;
            msg_r_elevation_velocity->candlc = 8;

            auto msg_r_yaw = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_r_yaw->canid = 0x213;
            msg_r_yaw->candlc = 4;

            bool flag_injection_autonomous = false;

            uint8_t _candata_joy[8];

            if(is_injection_autonomous == false)
            {
                velPlanner_injection_v.vel(static_cast<double>(-values[0]));

                velPlanner_injection_v.cycle();

                if(injection_mec == 0)
                {
                    float_to_bytes(_candata_joy, defalt_pitch);
                    float_to_bytes(_candata_joy+4, static_cast<float>(velPlanner_injection_v.vel()) * manual_injection_max_vel);
                    for(int i=0; i<msg_l_elevation_velocity->candlc; i++) msg_l_elevation_velocity->candata[i] = _candata_joy[i];

                    float_to_bytes(_candata_joy, static_cast<float>(atan2(-values[2], values[3])));
                    for(int i=0; i<msg_l_yaw->candlc; i++) msg_l_yaw->candata[i] = _candata_joy[i];

                    _pub_canusb->publish(*msg_l_elevation_velocity);
                    _pub_canusb->publish(*msg_l_yaw);
                }
                if(injection_mec == 1)
                {
                    float_to_bytes(_candata_joy, defalt_pitch);
                    float_to_bytes(_candata_joy+4, static_cast<float>(velPlanner_injection_v.vel()) * manual_injection_max_vel);
                    for(int i=0; i<msg_r_elevation_velocity->candlc; i++) msg_r_elevation_velocity->candata[i] = _candata_joy[i];

                    float_to_bytes(_candata_joy, static_cast<float>(atan2(-values[2], values[3])));
                    for(int i=0; i<msg_r_yaw->candlc; i++) msg_r_yaw->candata[i] = _candata_joy[i];

                    _pub_canusb->publish(*msg_r_elevation_velocity);
                    _pub_canusb->publish(*msg_r_yaw);
                }
                flag_injection_autonomous = true;
            }
        }

        void SmartphoneGamepad::_recv_pole(const unsigned char data[11])
        {
            pole_a[0] = data[0];
            pole_a[1] = data[1];
            pole_a[2] = data[2];
            pole_a[3] = data[3];
            pole_a[4] = data[4];
            pole_a[5] = data[5];
            pole_a[6] = data[6];
            pole_a[7] = data[7];
            pole_a[8] = data[8];
            pole_a[9] = data[9];
            pole_a[10] = data[10];
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
            if(!msg->data) is_spline_convergence = true;
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
}
