#pragma once

#include <string>
#include "send_udp.hpp"

class super_command
{
    public:
        super_command();

        void state_num(const char* data, const char* dest_ip, int dest_port);
        void pole(const char* data, const char* dest_ip, int dest_port);

    private:
        send_udp send_sequencer;

};