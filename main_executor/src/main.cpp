#include <rclcpp/rclcpp.hpp>
#include <rcl/rcl.h>
#include "socketcan_interface/socketcan_interface_node.hpp"
#include "mcl_2d/mcl_2d_node.hpp"
#include "controller_interface/controller_interface_node.hpp"
#include "spline_pid/spline_pid_node.hpp"
#include <iostream>

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    // auto socketcan_node = std::make_shared<socketcan_interface::SocketcanInterface>(nodes_option);
    auto mcl_2d_node = std::make_shared<mcl_2d::Mcl2D>(nodes_option);
    auto controller_node = std::make_shared<controller_interface::ControllerInterface>(nodes_option);
    auto spline_pid_node = std::make_shared<spline_pid::SplinePid>(nodes_option);

    // exec.add_node(socketcan_node);
    exec.add_node(mcl_2d_node);
    exec.add_node(controller_node);
    exec.add_node(spline_pid_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
