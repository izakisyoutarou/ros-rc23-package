#include "controller_interface/super_command.hpp"
#include <rclcpp/rclcpp.hpp>

#define IP_ER_PC "192.168.1.2"
#define IP_RR_PC "192.168.1.3"

super_command::super_command(){}

void super_command::state_num_ER(const unsigned char* data, int dest_port)
{
    int data_len = 2;
    const char* dest_ip = IP_ER_PC;
    send_sequencer.send(data, data_len, dest_ip, dest_port);
}

void super_command::state_num_RR(const unsigned char* data, int dest_port)
{
    int data_len = 2;
    const char* dest_ip = IP_RR_PC;
    send_sequencer.send(data, data_len, dest_ip, dest_port);
}

void super_command::pole_ER(const unsigned char* data, int dest_port)
{
    int data_len = 11;
    const char* dest_ip = IP_ER_PC;
    send_sequencer.send(data, data_len, dest_ip, dest_port);
}

void super_command::pole_RR(const unsigned char* data, int dest_port)
{
    int data_len = 11;
    const char* dest_ip = IP_RR_PC;
    send_sequencer.send(data, data_len, dest_ip, dest_port);
}