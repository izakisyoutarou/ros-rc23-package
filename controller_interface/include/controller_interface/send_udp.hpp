#pragma once
#include <string>

using namespace std;
class send_udp
{
    public:
        send_udp();
        void send(const unsigned char* data, int data_len, const string dest_ip, int dest_port);

    private:
};