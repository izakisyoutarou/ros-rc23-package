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

socket_robot_state(get_parameter("port.robot_state").as_int())

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

    _socket_timer = this->create_wall_timer(
        std::chrono::milliseconds(this->get_parameter("interval_ms").as_int()),
        [this] { _recv_callback(); }
    );

    publisher_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    publisher_pole_m0 = this->create_publisher<std_msgs::msg::String>("injection_pole_m0", _qos);
    publisher_pole_m1 = this->create_publisher<std_msgs::msg::String>("injection_pole_m1", _qos);
    publisher_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);
    publisher_rings = this->create_publisher<controller_interface_msg::msg::Injection>("current_rings", _qos);
}

void Sequencer::_subscriber_callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
    if(msg->is_restart){
        current_inject_state = msg->initial_state;
        current_pickup_state = msg->initial_state;
        initial_state = msg->initial_state;
    }
    judge_convergence.spline_convergence = msg->is_move_autonomous;
}

void Sequencer::_subscriber_callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg){
    if(current_pickup_state == "L0" || current_pickup_state == "L1"){
        if(msg->spline_convergence || !judge_convergence.spline_convergence){
            current_pickup_state = "";

            current_poles = {max_poles, max_poles};
            auto msg_rings = std::make_shared<controller_interface_msg::msg::Injection>();
            msg_rings->injectable_poles_m0 = current_poles[0];
            msg_rings->injectable_poles_m1 = current_poles[1];
            publisher_rings->publish(*msg_rings);
        }
    }
    else if(current_inject_state != initial_state){
        auto msg_inject = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_inject->canid = can_inject_id;
        msg_inject->candlc = 2;

        if(msg->injection_calculator0 && msg->injection0 && (msg->spline_convergence || !judge_convergence.spline_convergence)){
            msg_inject->candata[0] = true;
            publisher_can->publish(*msg_inject);
        }
        if(msg->injection_calculator1 && msg->injection1 && (msg->spline_convergence || !judge_convergence.spline_convergence)){
            msg_inject->candata[1] = true;
            publisher_can->publish(*msg_inject);
        }
    }
}

void Sequencer::_subscriber_callback_movable(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    auto msg_move_node = std::make_shared<std_msgs::msg::String>();
    msg_move_node->data = current_inject_state;
    publisher_move_node->publish(*msg_move_node);

    RCLCPP_INFO(this->get_logger(), "移動可能指令受信");
}

void Sequencer::_subscriber_callback_injection(const controller_interface_msg::msg::Injection::SharedPtr msg){
    if(msg->is_release_m0){

    }
    if(msg->is_release_m1){

    }
    if(msg->is_decrement_pole_m0){

    }
    if(msg->is_decrement_pole_m1){

    }
}

void Sequencer::_subscriber_callback_pole(const std_msgs::msg::String::SharedPtr msg){

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
    }
    else if(state=="L1"){
        msg_pickup_preparation->candata[4] = true; //右回収準備
        publisher_can->publish(*msg_pickup_preparation);
        RCLCPP_INFO(this->get_logger(), "右方回収準備");
    }

    if(state=="L0" || state=="L1"){
        current_pickup_state = state;
        msg_move_node->data = state;
        publisher_move_node->publish(*msg_move_node);

        cancel_inject(true, true);
        return;
    }

    /***** 回収特殊状態(L0,L1)入力時はこの先は実行しない *****/
    current_inject_state = state[0];

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
