#pragma once

#include <string>
#include "send_udp.hpp"

class super_command
{
    public:
        super_command();

        void state_num_ER(const unsigned char* data, int dest_port);
        void state_num_RR(const unsigned char* data, int dest_port);
        void pole_ER(const unsigned char* data, int dest_port);
        void pole_RR(const unsigned char* data, int dest_port);

    private:
        send_udp send_sequencer;

};