#include "controller_interface/super_command.hpp"
#include <rclcpp/rclcpp.hpp>

#define IP_ER_PC "192.168.1.2"
#define IP_RR_PC "192.168.1.3"

super_command::super_command(){}
//ERの状態をコントローラに送る
//send_udpクラスのsendto関数に送るデータをsuper_commandクラスのsend関数で定義している
void super_command::state_num_ER(const unsigned char* data, const string dest_ip, int dest_port)
{
    int data_len = 2;
    send_sequencer.send(data, data_len, dest_ip, dest_port);
}
//RRの状態をコントローラに送る
//send_udpクラスのsendto関数に送るデータをsuper_commandクラスのsend関数で定義している
void super_command::state_num_RR(const unsigned char* data, const string dest_ip, int dest_port)
{
    int data_len = 2;
    send_sequencer.send(data, data_len, dest_ip, dest_port);
}