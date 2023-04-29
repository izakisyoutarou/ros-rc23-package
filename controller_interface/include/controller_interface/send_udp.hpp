#pragma once

class send_udp
{
    public:
        send_udp();
        void send(const char* data, int data_len, const char* dest_ip, int dest_port);

    private:
};