#include "controller_interface/super_command.hpp"
#include <rclcpp/rclcpp.hpp>

super_command::super_command(){}

void super_command::state_num(const char* data, const char* dest_ip, int dest_port)
{
    int data_len = strlen(data);
    send_sequencer.send(data, data_len, dest_ip, dest_port);
}

void super_command::pole(const char* data, const char* dest_ip, int dest_port)
{
    int data_len = strlen(data);
    send_sequencer.send(data, data_len, dest_ip, dest_port);
}