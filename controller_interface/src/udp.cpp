#include "controller_interface/udp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sys/time.h>
#include <sys/types.h>

#define BUFFER_NUM 3
#define BUFSIZE 1024
#define ER_IP "192.168.1.4"

udp::udp(int udp_port_main, int udp_port_sub/*, int udp_port_er*/)
{
    //UDP
    sockfd_main = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&servaddr_main, 0, sizeof(servaddr_main));
    servaddr_main.sin_family = AF_INET;
    servaddr_main.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr_main.sin_port = htons(udp_port_main);
    bind(sockfd_main, (struct sockaddr *) &servaddr_main, sizeof(servaddr_main));
    udp_thread_main = std::thread(&udp::callback_udp_main, this, sockfd_main);

    sockfd_sub = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&servaddr_sub, 0, sizeof(servaddr_sub));
    servaddr_sub.sin_family = AF_INET;
    servaddr_sub.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr_sub.sin_port = htons(udp_port_sub);
    bind(sockfd_sub, (struct sockaddr *) &servaddr_sub, sizeof(servaddr_sub));
    udp_thread_sub = std::thread(&udp::callback_udp_sub, this, sockfd_sub);

    // sockfd_er_pole = socket(AF_INET, SOCK_DGRAM, 0);
    // memset(&servaddr_er_pole, 0, sizeof(servaddr_er_pole));
    // servaddr_er_pole.sin_family = AF_INET;
    // servaddr_er_pole.sin_addr.s_addr = htonl(INADDR_ANY);
    // servaddr_er_pole.sin_port = htons(udp_port_er);
    // bind(sockfd_er_pole, (struct sockaddr *) &servaddr_er_pole, sizeof(servaddr_er_pole));
    // udp_thread_er_pole = std::thread(&udp::callback_udp_er_pole, this, sockfd_er_pole);
}

void udp::callback_udp_main(int sockfd)
{
    char buffers[BUFFER_NUM][13];
    int cur_buf = 0;
    struct timeval tv;
    tv.tv_usec = udp_timeout_ms;

    while(rclcpp::ok())
    {
        clilen_main = sizeof(cliaddr_main);

        // ノンブロッキングモードでrecvfromを呼び出す
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(sockfd, &read_fds);
        int sel = select(sockfd + 1, &read_fds, NULL, NULL, &tv);
        if (sel == -1)
        {
            perror("select");
            continue;
        }
        else if (sel == 0)
        {
            // タイムアウトした場合、再試行
            continue;
        }

        // bufferに受信したデータが格納されている
        main = recvfrom(sockfd, buffers[cur_buf], BUFSIZE, 0, (struct sockaddr *) &cliaddr_main, &clilen_main);

        if (main < 0)
        {
            perror("recvfrom");
            continue;
        }

        if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) < 0) 
        {
            perror("setsockopt");
            continue;
        }

        std::memcpy(&l_x_main, &buffers[cur_buf][0], sizeof(l_x_main));
        std::memcpy(&l_y_main, &buffers[cur_buf][4], sizeof(l_y_main));
        std::memcpy(&r_x_main, &buffers[cur_buf][8], sizeof(r_x_main));
        std::memcpy(&r_y_main, &buffers[cur_buf][12], sizeof(r_y_main));
        cur_buf = (cur_buf + 1) % BUFFER_NUM;
    }
}

void udp::callback_udp_sub(int sockfd)
{
    char buffers[BUFFER_NUM][13];
    int cur_buf = 0;
    struct timeval tv;
    tv.tv_usec = udp_timeout_ms;

    while(rclcpp::ok())
    {
        clilen_sub = sizeof(cliaddr_sub);

        // ノンブロッキングモードでrecvfromを呼び出す
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(sockfd, &read_fds);
        int sel = select(sockfd + 1, &read_fds, NULL, NULL, &tv);
        if (sel == -1)
        {
            perror("select");
            continue;
        }
        else if (sel == 0)
        {
            // タイムアウトした場合、再試行
            continue;
        }

        // bufferに受信したデータが格納されている
        sub = recvfrom(sockfd, buffers[cur_buf], BUFSIZE, 0, (struct sockaddr *) &cliaddr_sub, &clilen_sub);

        if (sub < 0)
        {
            perror("recvfrom");
            continue;
        }

        if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) < 0) 
        {
            perror("setsockopt");
            continue;
        }               

        std::memcpy(&l_x_sub, &buffers[cur_buf][0], sizeof(l_x_sub));
        std::memcpy(&l_y_sub, &buffers[cur_buf][4], sizeof(l_y_sub));
        std::memcpy(&r_x_sub, &buffers[cur_buf][8], sizeof(r_x_sub));
        std::memcpy(&r_y_sub, &buffers[cur_buf][12], sizeof(r_y_sub));   
        cur_buf = (cur_buf + 1) % BUFFER_NUM;
    }
}

// void udp::callback_udp_er_pole(int sockfd)
// {
//     char buffers[BUFSIZE];
//     int cur_buf = 0;
//     struct timeval tv;
//     tv.tv_usec = udp_timeout_ms;

//     while(rclcpp::ok())
//     {
//         clilen_sub = sizeof(cliaddr_sub);

//         // ノンブロッキングモードでrecvfromを呼び出す
//         fd_set read_fds;
//         FD_ZERO(&read_fds);
//         FD_SET(sockfd, &read_fds);
//         int sel = select(sockfd + 1, &read_fds, NULL, NULL, &tv);
//         if (sel == -1)
//         {
//             perror("select");
//             continue;
//         }
//         else if (sel == 0)
//         {
//             // タイムアウトした場合、再試行
//             continue;
//         }

//         // bufferに受信したデータが格納されている
//         sub = recvfrom(sockfd, buffers[cur_buf], BUFSIZE, 0, (struct sockaddr *) &cliaddr_sub, &clilen_sub);

//         if (sub < 0)
//         {
//             perror("recvfrom");
//             continue;
//         }

//         if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) < 0) 
//         {
//             perror("setsockopt");
//             continue;
//         }               

//         std::memcpy(&l_x_sub, &buffers[cur_buf][0], sizeof(l_x_sub));
//         std::memcpy(&l_y_sub, &buffers[cur_buf][4], sizeof(l_y_sub));
//         std::memcpy(&r_x_sub, &buffers[cur_buf][8], sizeof(r_x_sub));
//         std::memcpy(&r_y_sub, &buffers[cur_buf][12], sizeof(r_y_sub));   
//         cur_buf = (cur_buf + 1) % BUFFER_NUM;
//     }
// }