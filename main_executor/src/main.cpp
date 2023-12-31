#include <rclcpp/rclcpp.hpp>
#include "socketcan_interface/socketcan_interface_node.hpp"
#include "mcl_2d/mcl_2d_node.hpp"
#include "controller_interface/controller_interface_node.hpp"
#include "spline_pid/spline_pid_node.hpp"
#include "injection_interface/injection_interface_node.hpp"
#include "injection_param_calculator/injection_param_calculator.hpp"
#include "RANSAC_localization/RANSAC_localization.hpp"
#include "sequencer/sequencer_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    auto socketcan_node = std::make_shared<socketcan_interface::SocketcanInterface>(nodes_option);
    auto RANSAC_localization = std::make_shared<self_localization::RANSACLocalization>(nodes_option);
    auto sequencer = std::make_shared<sequencer::Sequencer>(nodes_option);
    // auto mcl_2d_node = std::make_shared<mcl_2d::Mcl2D>(nodes_option);
    auto controller_node = std::make_shared<controller_interface::SmartphoneGamepad>(nodes_option);
    auto injection_interface_node0 = std::make_shared<injection_interface::InjectionInterface>(nodes_option, 0);
    auto injection_interface_node1 = std::make_shared<injection_interface::InjectionInterface>(nodes_option, 1);
    auto injection_param_calclator_node0 = std::make_shared<injection_param_calculator::InjectionParamCalculator>(nodes_option, 0);
    auto injection_param_calclator_node1 = std::make_shared<injection_param_calculator::InjectionParamCalculator>(nodes_option, 1);
    auto spline_pid_node = std::make_shared<spline_pid::SplinePid>(nodes_option);

    exec.add_node(socketcan_node);
    exec.add_node(RANSAC_localization);
    exec.add_node(sequencer);
    // exec.add_node(mcl_2d_node);
    exec.add_node(spline_pid_node);
    exec.add_node(controller_node);
    exec.add_node(injection_interface_node0);
    exec.add_node(injection_interface_node1);
    exec.add_node(injection_param_calclator_node0);
    exec.add_node(injection_param_calclator_node1);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
