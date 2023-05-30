#include "sequencer/sequencer_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "utilities/can_utils.hpp"
#include "utilities/utils.hpp"
#include <fstream>
#include <boost/format.hpp>

using namespace std;
using namespace utils;

namespace sequencer{

Sequencer::Sequencer(const rclcpp::NodeOptions &options) : Sequencer("", options) {}

Sequencer::Sequencer(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("sequencer_node", name_space, options),
can_movable_id(get_parameter("canid.movable").as_int()),
can_digital_button_id(get_parameter("canid.sub_digital_button").as_int()),
can_inject_id(get_parameter("canid.inject").as_int()),
can_cancel_inject_id(get_parameter("canid.cancel_inject").as_int()),
can_calculatable_id(get_parameter("canid.calculatable").as_int()),

first_inject_state(get_parameter("first_inject_state").as_string()),
second_inject_state(get_parameter("second_inject_state").as_string()),

socket_robot_state(get_parameter("port.robot_state").as_int()),

aimable_poles_atA_file_path(ament_index_cpp::get_package_share_directory("main_executor")+"/config/"+"/sequencer/"+"aimable_poles_atA.cfg"),
aimable_poles_atB_file_path(ament_index_cpp::get_package_share_directory("main_executor")+"/config/"+"/sequencer/"+"aimable_poles_atB.cfg"),
aimable_poles_atC_file_path(ament_index_cpp::get_package_share_directory("main_executor")+"/config/"+"/sequencer/"+"aimable_poles_atC.cfg"),
aimable_poles_atD_file_path(ament_index_cpp::get_package_share_directory("main_executor")+"/config/"+"/sequencer/"+"aimable_poles_atD.cfg")
{
    _subscription_base_control = this->create_subscription<controller_interface_msg::msg::BaseControl>(
        "pub_base_control",
        _qos,
        std::bind(&Sequencer::_subscriber_callback_base_control, this, std::placeholders::_1)
    );
    _subscription_convergence = this->create_subscription<controller_interface_msg::msg::Convergence>(
        "pub_convergence",
        _qos,
        std::bind(&Sequencer::_subscriber_callback_convergence, this, std::placeholders::_1)
    );
    _subscription_movable = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_"+ (boost::format("%x") % can_movable_id).str(),
        _qos,
        std::bind(&Sequencer::_subscriber_callback_movable, this, std::placeholders::_1)
    );
    _subscription_calculatable = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_"+ (boost::format("%x") % can_calculatable_id).str(),
        _qos,
        std::bind(&Sequencer::_subscriber_callback_calculatable, this, std::placeholders::_1)
    );
    _subscription_injected = this->create_subscription<controller_interface_msg::msg::Injection>(
        "injection_mech",
        _qos,
        std::bind(&Sequencer::_subscriber_callback_injection, this, std::placeholders::_1)
    );
    _subscription_pole = this->create_subscription<std_msgs::msg::String>(
        "injection_pole",
        _qos,
        std::bind(&Sequencer::_subscriber_callback_pole, this, std::placeholders::_1)
    );
    _subscription_move_progress = this->create_subscription<std_msgs::msg::Float64>(
        "move_progress",
        _qos,
        std::bind(&Sequencer::_subscriber_callback_move_progress, this, std::placeholders::_1)
    );

    _socket_timer = this->create_wall_timer(
        std::chrono::milliseconds(this->get_parameter("interval_ms").as_int()),
        [this] { _recv_callback(); }
    );

    publisher_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    publisher_pole_m0 = this->create_publisher<std_msgs::msg::String>("injection_pole_m0", _qos);
    publisher_pole_m1 = this->create_publisher<std_msgs::msg::String>("injection_pole_m1", _qos);
    publisher_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);
    publisher_rings = this->create_publisher<controller_interface_msg::msg::Injection>("current_rings", _qos);
    publisher_inject_state = this->create_publisher<std_msgs::msg::String>("inject_state", _qos);
}

void Sequencer::_subscriber_callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
    if(msg->is_restart){
        current_inject_state = "";
        current_pickup_state = "";
        // initial_state = msg->initial_state;
        pick_assistant = false;
        current_move_progress = 0.0;
        current_rings = {0,0};
    }
    judge_convergence.spline_convergence = msg->is_move_autonomous;
    judge_convergence.injection0= msg->is_injection_autonomous;
    judge_convergence.injection1= msg->is_injection_autonomous;
}

void Sequencer::_subscriber_callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg){
    // cout << "回収状態 : " << current_pickup_state << endl;
    if((current_pickup_state == "L0" || current_pickup_state == "L1") && current_move_progress > 0.7){
        if(!pick_assistant){
            auto msg_pick_assistant = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_pick_assistant->canid = can_digital_button_id;
            msg_pick_assistant->candlc = 8;

            msg_pick_assistant->candata[2] = true; //回収補助機構
            publisher_can->publish(*msg_pick_assistant);
            RCLCPP_INFO(this->get_logger(), "回収補助機構展開");

            pick_assistant = true;
        }
        if(msg->spline_convergence || !judge_convergence.spline_convergence){
            // 回収もしくは回収・装填
            auto msg_pickup = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_pickup->canid = can_digital_button_id;
            msg_pickup->candlc = 8;

            if(current_rings[0] > 0 || current_rings[1] > 0 || current_pickup_state == "L1"){
                msg_pickup->candata[5] = true; // 回収
                RCLCPP_INFO(this->get_logger(), "リングの回収");
            }
            else{
                msg_pickup->candata[7] = true; // 回収・装填
                RCLCPP_INFO(this->get_logger(), "リングの回収及び装填");
            }

            publisher_can->publish(*msg_pickup);
            current_pickup_state = "";

            // 射出可能リング数の更新
            current_rings = {current_rings[0]+max_rings, current_rings[1]+max_rings};
            auto msg_rings = std::make_shared<controller_interface_msg::msg::Injection>();
            msg_rings->injectable_rings[0] = current_rings[0];
            msg_rings->injectable_rings[1] = current_rings[1];
            publisher_rings->publish(*msg_rings);
        }
    }
    else if(current_inject_state != ""){
        auto msg_inject = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_inject->canid = can_inject_id;
        msg_inject->candlc = 2;
        auto msg_rings = std::make_shared<controller_interface_msg::msg::Injection>();

        if(judge_convergence.injection0 && msg->injection_calculator0 && msg->injection0 && inject_flag[0] && (msg->spline_convergence || !judge_convergence.spline_convergence)){
            inject_flag[0] = false;
            msg_inject->candata[0] = true;
            publisher_can->publish(*msg_inject);
            msg_rings->injectable_rings[0] = --current_rings[0];
        }
        if(judge_convergence.injection1 && msg->injection_calculator1 && msg->injection1 && inject_flag[1] && (msg->spline_convergence || !judge_convergence.spline_convergence)){
            inject_flag[1] = false;
            msg_inject->candata[1] = true;
            publisher_can->publish(*msg_inject);
            msg_rings->injectable_rings[1] = --current_rings[1];
        }
        publisher_rings->publish(*msg_rings);
    }
}

void Sequencer::_subscriber_callback_movable(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    auto msg_move_node = std::make_shared<std_msgs::msg::String>();
    msg_move_node->data = current_inject_state;
    publisher_move_node->publish(*msg_move_node);

    RCLCPP_INFO(this->get_logger(), "移動可能指令受信");
}

void Sequencer::_subscriber_callback_calculatable(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){

}

void Sequencer::_subscriber_callback_injection(const controller_interface_msg::msg::Injection::SharedPtr msg){
    if(msg->is_release_mech[0]){
        is_mech_locking[0] = false;
    }
    if(msg->is_release_mech[1]){
        is_mech_locking[1] = false;
    }
}

void Sequencer::_subscriber_callback_pole(const std_msgs::msg::String::SharedPtr msg){
    auto msg_pole = std::make_shared<std_msgs::msg::String>();

    // 射出機構の優先順位を算出
    array<int,2> mech_priority = {11,11};
    int count = 0;
    for(const auto &pole : aimable_poles_m0){
        if(pole == msg->data){
            mech_priority[0] = count;
            break;
        }
        count++;
    }
    count = 0;
    for(const auto &pole : aimable_poles_m1){
        if(pole == msg->data){
            mech_priority[1] = count;
            break;
        }
        count++;
    }

    // 狙っている射出機構がロック状態であればそれを射出させる
    if(mech_priority[0] < 11 && msg->data == aiming_pole[0] && is_mech_locking[0]){
        is_mech_locking[0] = true;
        aiming_pole[0] = msg->data;
        inject_flag[0] = true;
        msg_pole->data = msg->data;
        publisher_pole_m0->publish(*msg_pole);
        return;
    }
    else if(mech_priority[1] < 11 && msg->data == aiming_pole[1] && is_mech_locking[1]){
        is_mech_locking[1] = true;
        aiming_pole[1] = msg->data;
        inject_flag[1] = true;
        msg_pole->data = msg->data;
        publisher_pole_m1->publish(*msg_pole);
        return;
    }

    // 狙っている射出機構がない場合
    if(!is_mech_locking[0] && mech_priority[0] < 11 && (mech_priority[0] <= mech_priority[1] || is_mech_locking[1])){  // 優先順位が等しい場合は機構0を優先する
        is_mech_locking[0] = true;
        aiming_pole[0] = msg->data;
        inject_flag[0] = true;
        msg_pole->data = msg->data;
        publisher_pole_m0->publish(*msg_pole);
        return;
    }
    else if(!is_mech_locking[1] && mech_priority[1] < 11){
        is_mech_locking[1] = true;
        aiming_pole[1] = msg->data;
        inject_flag[1] = true;
        msg_pole->data = msg->data;
        publisher_pole_m1->publish(*msg_pole);
        return;
    }

}

void Sequencer::_subscriber_callback_move_progress(const std_msgs::msg::Float64::SharedPtr msg){
    current_move_progress = msg->data;
}

void Sequencer::_recv_callback(){
    if(socket_robot_state.is_recved()){
        unsigned char data[2];
        _recv_robot_state(socket_robot_state.data(data, sizeof(data)));
    }
}

void Sequencer::_recv_robot_state(const unsigned char data[2]){
    const string state = {static_cast<char>(data[0]), static_cast<char>(data[1])};

    auto msg_pickup_preparation = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_pickup_preparation->canid = can_digital_button_id;
    msg_pickup_preparation->candlc = 8;

    auto msg_move_node = std::make_shared<std_msgs::msg::String>();

    if(state=="L0"){
        msg_pickup_preparation->candata[6] = true; //左回収準備
        publisher_can->publish(*msg_pickup_preparation);
        RCLCPP_INFO(this->get_logger(), "左方回収準備");
        current_inject_state = first_inject_state;
        cout << first_inject_state << "に射出状態を移行" << endl;
    }
    else if(state=="L1"){
        msg_pickup_preparation->candata[4] = true; //右回収準備
        publisher_can->publish(*msg_pickup_preparation);
        RCLCPP_INFO(this->get_logger(), "右方回収準備");
        current_inject_state = second_inject_state;
        cout << second_inject_state << "に射出状態を移行" << endl;
    }

    if(state=="L0" || state=="L1"){
        current_pickup_state = state;
        msg_move_node->data = state;
        publisher_move_node->publish(*msg_move_node);
        pick_assistant = false;
        current_move_progress = 0.0;

        cancel_inject(true, true);
        // return;
    }
    else{
        current_inject_state = state[0];
    }

    // 射出状態を送信
    auto msg_state = std::make_shared<std_msgs::msg::String>();
    msg_state->data = current_inject_state;
    publisher_inject_state->publish(*msg_state);

    // 各状態での射出可能ポールのリストアップ
    string aimable_poles_file_path;
    if(current_inject_state=="A") aimable_poles_file_path = aimable_poles_atA_file_path;
    else if(current_inject_state=="B") aimable_poles_file_path = aimable_poles_atB_file_path;
    else if(current_inject_state=="C") aimable_poles_file_path = aimable_poles_atC_file_path;
    else if(current_inject_state=="D") aimable_poles_file_path = aimable_poles_atD_file_path;

    ifstream ifs(aimable_poles_file_path);
    if(ifs){
        aimable_poles_m0.clear();
        aimable_poles_m1.clear();
    }
    string str;
    int mech_num = 0;
    while(getline(ifs, str)){
        string token;
        istringstream stream(str);
        int count = 0;
        while(getline(stream, token, ' ')){   //スペース区切り
            if(count==0 && token=="#") break;
            else if(count==0) mech_num++;

            if(mech_num==1) aimable_poles_m0.push_back(token);
            else if(mech_num==2) aimable_poles_m1.push_back(token);
            count++;
        }
    }

    // プリント
    for(const auto pole: aimable_poles_m0){
        cout << pole << " ";
    }
    cout << " : 機構0" << endl;
    for(const auto pole: aimable_poles_m1){
        cout << pole << " ";
    }
    cout << " : 機構1" << endl;

    is_mech_locking[0] = false;
    is_mech_locking[1] = false;

    if(current_pickup_state != "L0" && current_pickup_state != "L1"){
        msg_move_node->data = current_inject_state;
        publisher_move_node->publish(*msg_move_node);
        cancel_inject(true, true);
    }
}

void Sequencer::cancel_inject(const bool mech0, const bool mech1){
    auto msg_cancel_inject = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_cancel_inject->canid = can_cancel_inject_id;
    msg_cancel_inject->candlc = 2;
    msg_cancel_inject->candata[0] = mech0;   //機構0
    msg_cancel_inject->candata[1] = mech1;   //機構1
    publisher_can->publish(*msg_cancel_inject);
}

}  // namespace sequencer
