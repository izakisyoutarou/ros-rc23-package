#include "controller_interface/send_udp.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>//memcpyのため

send_udp::send_udp(){}

void send_udp::send(const unsigned char* data, int data_len, const char* dest_ip, int dest_port)
{
    int sockfd;
    struct sockaddr_in servaddr;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(dest_ip);
    servaddr.sin_port = htons(dest_port);

    int n = sendto(sockfd, data, data_len, 0, (const struct sockaddr *) &servaddr, sizeof(servaddr));
    if (n < 0) {
        perror("sendto");
    }
    close(sockfd);
}