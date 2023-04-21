#pragma once
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>//memcpyのため
#include <thread>

class udp
{
    public:
        udp(int udp_port_main, int udp_port_sub);
        
        void callback_udp_main(int sockfd);
        void callback_udp_sub(int sockfd);

        float analog_l_x_main(){return l_x_main;}
        float analog_l_y_main(){return l_y_main;}
        float analog_r_x_main(){return r_x_main;}
        float analog_r_y_main(){return r_y_main;}
        float analog_l_x_sub(){return l_x_sub;}
        float analog_l_y_sub(){return l_y_sub;}
        float analog_r_x_sub(){return r_x_sub;}
        float analog_r_y_sub(){return r_y_sub;}

    private:
        float l_x_main = 0.0f;
        float l_y_main = 0.0f;
        float r_x_main = 0.0f;
        float r_y_main = 0.0f;

        float l_x_sub = 0.0f;
        float l_y_sub = 0.0f;
        float r_x_sub = 0.0f;
        float r_y_sub = 0.0f;

        int udp_timeout_ms = 17;

        int sockfd_main, main;
        socklen_t clilen_main;
        char* buffer_main = new char[16];
        struct sockaddr_in servaddr_main, cliaddr_main;
        std::thread udp_thread_main;

        int sockfd_sub, sub;
        socklen_t clilen_sub;
        char* buffer_sub = new char[16];
        struct sockaddr_in servaddr_sub, cliaddr_sub;
        std::thread udp_thread_sub;
};