#include "controller_interface_er/controller_interface_er_node.hpp"
#include <sys/time.h>
#include <sys/types.h>

using namespace utils;

namespace controller_interface
{
    #define BUFFER_NUM 3
    #define BUFSIZE 1024
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

        defalt_pitch(static_cast<float>(get_parameter("defalt_pitch").as_double())),
        manual_linear_max_vel(static_cast<float>(get_parameter("linear_max_vel").as_double())),
        manual_angular_max_vel(dtor(static_cast<float>(get_parameter("angular_max_vel").as_double()))),
        manual_injection_max_vel(dtor(static_cast<float>(get_parameter("injection_max_vel").as_double()))),
        defalt_restart_flag(get_parameter("defalt_restart_flag").as_bool()),
        defalt_emergency_flag(get_parameter("defalt_emergency_flag").as_bool()),
        defalt_move_autonomous_flag(get_parameter("defalt_move_autonomous_flag").as_bool()),
        defalt_injection_autonomous_flag(get_parameter("defalt_injection_autonomous_flag").as_bool()),
        defalt_injection_mec(get_parameter("defalt_injection_mec").as_int()),
        udp_port_main(get_parameter("udp_port_main").as_int()),
        udp_port_sub(get_parameter("udp_port_sub").as_int()),
        udp_timeout_ms(get_parameter("udp_timeout_ms").as_int())
        {
            const auto heartbeat_ms = this->get_parameter("heartbeat_ms").as_int();
            const auto move_injection_heteronomy_ms = this->get_parameter("move_injection_heteronomy_ms").as_int();

            //controllerからsub
            _sub_pad_main = this->create_subscription<controller_interface_msg::msg::SubPad>(
                "pad_main",
                _qos,
                std::bind(&SmartphoneGamepad::callback_pad_main, this, std::placeholders::_1)
            );

            _sub_pad_sub = this->create_subscription<controller_interface_msg::msg::SubPad>(
                "pad_sub",
                _qos,
                std::bind(&SmartphoneGamepad::callback_pad_sub, this, std::placeholders::_1)
            );

            _sub_scrn_pole_0 = this->create_subscription<std_msgs::msg::String>(
                "injection_pole_m0",
                _qos,
                std::bind(&SmartphoneGamepad::callback_scrn_pole, this, std::placeholders::_1)
            );

            _sub_scrn_pole_1 = this->create_subscription<std_msgs::msg::String>(
                "injection_pole_m1",
                _qos,
                std::bind(&SmartphoneGamepad::callback_scrn_pole, this, std::placeholders::_1)
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
            _pub_convergence = this->create_publisher<controller_interface_msg::msg::Convergence>("convergence" , _qos);
            _pub_scrn_string = this->create_publisher<std_msgs::msg::String>("scrn_pole", _qos);

            //各nodeへリスタートと手自動の切り替えをpub。
            _pub_base_control = this->create_publisher<controller_interface_msg::msg::BaseControl>("base_control",_qos);

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

            //上物と足回り手動周期(60hz)
            _move_injection_heteronomy = this->create_wall_timer(
                std::chrono::milliseconds(move_injection_heteronomy_ms),
                std::bind(&SmartphoneGamepad::callback_move_injection_heteronomy, this) 
            );

            //計画機
            velPlanner_linear_x.limit(limit_linear);
            velPlanner_linear_y.limit(limit_linear);
            velPlanner_angular_z.limit(limit_angular);
            velPlanner_injection_v.limit(limit_injection);

            //UDP
            sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            memset(&servaddr, 0, sizeof(servaddr));
            servaddr.sin_family = AF_INET;
            servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
            servaddr.sin_port = htons(udp_port_main);
            bind(sockfd, (struct sockaddr *) &servaddr, sizeof(servaddr));
            udp_thread_ = std::thread(&SmartphoneGamepad::callback_udp_main, this, sockfd);

            sockfd2 = socket(AF_INET, SOCK_DGRAM, 0);
            memset(&servaddr2, 0, sizeof(servaddr2));
            servaddr2.sin_family = AF_INET;
            servaddr2.sin_addr.s_addr = htonl(INADDR_ANY);
            servaddr2.sin_port = htons(udp_port_sub);
            bind(sockfd2, (struct sockaddr *) &servaddr2, sizeof(servaddr2));
            udp_thread_2 = std::thread(&SmartphoneGamepad::callback_udp_sub, this, sockfd2);
        }

        void SmartphoneGamepad::callback_pad_main(const controller_interface_msg::msg::SubPad::SharedPtr msg)
        {
            auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_restart->canid = 0x002;
            msg_restart->candlc = 0;

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = 0x000;
            msg_emergency->candlc = 1;

            auto msg_btn = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_btn->canid = 0x300;
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
                //RCLCPP_INFO(this->get_logger(), "a:%db:%dy:%dx:%dright:%ddown:%dleft:%dup:%d", msg->a, msg->b, msg->y, msg->x, msg->right, msg->down, msg->left, msg->up);
            }
            if(msg->g)_pub_canusb->publish(*msg_emergency);
            if(robotcontrol_flag)_pub_base_control->publish(*msg_base_control);
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

        void SmartphoneGamepad::callback_pad_sub(const controller_interface_msg::msg::SubPad::SharedPtr msg)
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
            msg_btn->canid = 0x300;
            msg_btn->candlc = 8;

            auto msg_scrn_pole_restart = std::make_shared<std_msgs::msg::String>(); 

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
                msg_scrn_pole_restart->data = "restart";
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
            if(robotcontrol_flag)_pub_base_control->publish(*msg_base_control);
            if(msg->s) _pub_scrn_string->publish(*msg_scrn_pole_restart);
            if(flag_restart)
            {
                msg_base_control->is_restart = false;
                _pub_base_control->publish(*msg_base_control);
            }
        }

        void SmartphoneGamepad::callback_scrn_pole(const std_msgs::msg::String::SharedPtr msg)
        {
            auto msg_scrn_pole = std::make_shared<std_msgs::msg::String>(); 
            if(msg->data == "A") msg_scrn_pole->data = "A";
            if(msg->data == "B") msg_scrn_pole->data = "B";
            if(msg->data == "C") msg_scrn_pole->data = "C";
            if(msg->data == "D") msg_scrn_pole->data = "D";
            if(msg->data == "E") msg_scrn_pole->data = "E";
            if(msg->data == "F") msg_scrn_pole->data = "F";
            if(msg->data == "G") msg_scrn_pole->data = "G";
            if(msg->data == "H") msg_scrn_pole->data = "H";
            if(msg->data == "I") msg_scrn_pole->data = "I";
            if(msg->data == "J") msg_scrn_pole->data = "J";
            if(msg->data == "K") msg_scrn_pole->data = "K";
            if(msg->data == "NotA") msg_scrn_pole->data = "NotA";
            if(msg->data == "NotB") msg_scrn_pole->data = "NotB";
            if(msg->data == "NotC") msg_scrn_pole->data = "NotC";
            if(msg->data == "NotD") msg_scrn_pole->data = "NotD";
            if(msg->data == "NotE") msg_scrn_pole->data = "NotE";
            if(msg->data == "NotF") msg_scrn_pole->data = "NotF";
            if(msg->data == "NotG") msg_scrn_pole->data = "NotG";
            if(msg->data == "NotH") msg_scrn_pole->data = "NotH";
            if(msg->data == "NotI") msg_scrn_pole->data = "NotI";
            if(msg->data == "NotJ") msg_scrn_pole->data = "NotJ";
            if(msg->data == "NotK") msg_scrn_pole->data = "NotK";
            _pub_scrn_string->publish(*msg_scrn_pole);
        }

        void SmartphoneGamepad::callback_udp_main(int sockfd)
        {
            char buffers[BUFFER_NUM][13];
            int cur_buf = 0;
            struct timeval tv;
            tv.tv_usec = udp_timeout_ms;

            while(rclcpp::ok())
            {
                clilen = sizeof(cliaddr);

                // ノンブロッキングモードでrecvfromを呼び出す
                fd_set read_fds;
                FD_ZERO(&read_fds);
                FD_SET(sockfd, &read_fds);
                int sel = select(sockfd + 1, &read_fds, NULL, NULL, &tv);
                if (sel == -1)
                {
                    perror("select");
                    continue;
                }
                else if (sel == 0)
                {
                    // タイムアウトした場合、再試行
                    continue;
                }

                // bufferに受信したデータが格納されている
                n = recvfrom(sockfd, buffers[cur_buf], BUFSIZE, 0, (struct sockaddr *) &cliaddr, &clilen);

                if (n < 0)
                {
                    perror("recvfrom");
                    continue;
                }

                if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) < 0) 
                {
                    perror("setsockopt");
                    continue;
                }

                std::memcpy(&analog_l_x_main, &buffers[cur_buf][0], sizeof(analog_l_x_main));
                std::memcpy(&analog_l_y_main, &buffers[cur_buf][4], sizeof(analog_l_y_main));
                std::memcpy(&analog_r_x_main, &buffers[cur_buf][8], sizeof(analog_r_x_main));
                std::memcpy(&analog_r_y_main, &buffers[cur_buf][12], sizeof(analog_r_y_main));
                cur_buf = (cur_buf + 1) % BUFFER_NUM;
            }
        }

        void SmartphoneGamepad::callback_udp_sub(int sockfd)
        {
            char buffers[BUFFER_NUM][13];
            int cur_buf = 0;
            struct timeval tv;
            tv.tv_usec = udp_timeout_ms;

            while(rclcpp::ok())
            {
                clilen = sizeof(cliaddr);

                // ノンブロッキングモードでrecvfromを呼び出す
                fd_set read_fds;
                FD_ZERO(&read_fds);
                FD_SET(sockfd, &read_fds);
                int sel = select(sockfd + 1, &read_fds, NULL, NULL, &tv);
                if (sel == -1)
                {
                    perror("select");
                    continue;
                }
                else if (sel == 0)
                {
                    // タイムアウトした場合、再試行
                    continue;
                }

                // bufferに受信したデータが格納されている
                n = recvfrom(sockfd, buffers[cur_buf], BUFSIZE, 0, (struct sockaddr *) &cliaddr, &clilen);

                if (n < 0)
                {
                    perror("recvfrom");
                    continue;
                }

                if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) < 0) 
                {
                    perror("setsockopt");
                    continue;
                }               

                std::memcpy(&analog_l_x_sub, &buffers[cur_buf][0], sizeof(analog_l_x_sub));
                std::memcpy(&analog_l_y_sub, &buffers[cur_buf][4], sizeof(analog_l_y_sub));
                std::memcpy(&analog_r_x_sub, &buffers[cur_buf][8], sizeof(analog_r_x_sub));
                std::memcpy(&analog_r_y_sub, &buffers[cur_buf][12], sizeof(analog_r_y_sub));   
                cur_buf = (cur_buf + 1) % BUFFER_NUM;
            }
        }

        void SmartphoneGamepad::callback_move_injection_heteronomy()
        {
            auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = 0x100;
            msg_linear->candlc = 8;

            auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = 0x101;
            msg_angular->candlc = 4;

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

            auto msg_gazebo = std::make_shared<geometry_msgs::msg::Twist>();

            uint8_t _candata_joy[8];

            bool flag_move_autonomous = false;
            bool flag_injection_autonomous = false;

            if(is_move_autonomous == false)
            {
                velPlanner_linear_x.vel(static_cast<double>(analog_l_y_main));//unityとロボットにおける。xとyが違うので逆にしている。
                velPlanner_linear_y.vel(static_cast<double>(analog_l_x_main));
                velPlanner_angular_z.vel(static_cast<double>(analog_r_x_main));

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
                _pub_gazebo->publish(*msg_gazebo);

                flag_move_autonomous = true;
            }
            if(is_injection_autonomous == false)
            {
                velPlanner_injection_v.vel(static_cast<double>(analog_l_x_sub));

                velPlanner_injection_v.cycle();

                if(injection_mec == 0)
                {
                    float_to_bytes(_candata_joy, defalt_pitch);
                    float_to_bytes(_candata_joy+4, static_cast<float>(velPlanner_injection_v.vel()) * manual_injection_max_vel);
                    for(int i=0; i<msg_l_elevation_velocity->candlc; i++) msg_l_elevation_velocity->candata[i] = _candata_joy[i];

                    float_to_bytes(_candata_joy, static_cast<float>(atan2(-analog_r_x_sub, analog_r_y_sub)));
                    for(int i=0; i<msg_l_yaw->candlc; i++) msg_l_yaw->candata[i] = _candata_joy[i];

                    _pub_canusb->publish(*msg_l_elevation_velocity);
                    _pub_canusb->publish(*msg_l_yaw);
                }
                if(injection_mec == 1)
                {
                    float_to_bytes(_candata_joy, defalt_pitch);
                    float_to_bytes(_candata_joy+4, static_cast<float>(velPlanner_injection_v.vel()) * manual_injection_max_vel);
                    for(int i=0; i<msg_r_elevation_velocity->candlc; i++) msg_r_elevation_velocity->candata[i] = _candata_joy[i];

                    float_to_bytes(_candata_joy, static_cast<float>(atan2(-analog_r_x_sub, analog_r_y_sub)));
                    for(int i=0; i<msg_r_yaw->candlc; i++) msg_r_yaw->candata[i] = _candata_joy[i];

                    _pub_canusb->publish(*msg_r_elevation_velocity);
                    _pub_canusb->publish(*msg_r_yaw);
                }
                flag_injection_autonomous = true;
            }
            if(is_move_autonomous == true || is_injection_autonomous == true) 
            {
                if(flag_move_autonomous == true)
                {
                    float_to_bytes(_candata_joy, 0);
                    for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];
                    for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];

                    _pub_canusb->publish(*msg_linear);
                    _pub_canusb->publish(*msg_angular);
                }
                if(flag_injection_autonomous == true)
                {
                    float_to_bytes(_candata_joy, 0);
                    for(int i=0; i<msg_l_elevation_velocity->candlc; i++) msg_l_elevation_velocity->candata[i] = _candata_joy[i];
                    for(int i=0; i<msg_l_yaw->candlc; i++) msg_l_yaw->candata[i] = _candata_joy[i];
                    for(int i=0; i<msg_r_elevation_velocity->candlc; i++) msg_r_elevation_velocity->candata[i] = _candata_joy[i];
                    for(int i=0; i<msg_r_yaw->candlc; i++) msg_r_yaw->candata[i] = _candata_joy[i];

                    _pub_canusb->publish(*msg_l_elevation_velocity);
                    _pub_canusb->publish(*msg_l_yaw);
                    _pub_canusb->publish(*msg_r_elevation_velocity);
                    _pub_canusb->publish(*msg_r_yaw);
                }
            }
        }

        void SmartphoneGamepad::callback_main(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg)
        {
            //mainから射出可能司令のsub。それぞれをconvergenceの適当なところに入れてpub。上物の収束状況。
            auto msg_injectioncommnd = std::make_shared<controller_interface_msg::msg::Convergence>();
            uint8_t _candata[2];
            for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
            msg_injectioncommnd->injection0 = static_cast<bool>(_candata[0]);
            msg_injectioncommnd->injection1 = static_cast<bool>(_candata[1]);
            _pub_convergence->publish(*msg_injectioncommnd);
        }

        void SmartphoneGamepad::callback_spline(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //spline_pidから足回り収束のsub。onvergenceの適当なところに入れてpub。足回りの収束状況。
            auto msg_spline_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            is_spline_convergence = msg->data;
            msg_spline_convergence->spline_convergence = is_spline_convergence;
            _pub_convergence->publish(*msg_spline_convergence);
        }

        void SmartphoneGamepad::callback_injection_calculator_0(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //injection_calculatorから上モノ指令値計算収束のsub。onvergenceの適当なところに入れてpub。上物の指令値の収束情報。
            auto msg_injection_calculator0_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            is_injection_calculator0_convergence = msg->data;
            msg_injection_calculator0_convergence->injection_calculator0 = is_injection_calculator0_convergence;
            _pub_convergence->publish(*msg_injection_calculator0_convergence);
        }

        void SmartphoneGamepad::callback_injection_calculator_1(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //injection_calculatorから上モノ指令値計算収束のsub。onvergenceの適当なところに入れてpub。上物の指令値の収束情報。7
            auto msg_injection_calculator1_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            is_injection_calculator1_convergence = msg->data;
            msg_injection_calculator1_convergence->injection_calculator1 = is_injection_calculator1_convergence;
            _pub_convergence->publish(*msg_injection_calculator1_convergence);
        }
}
