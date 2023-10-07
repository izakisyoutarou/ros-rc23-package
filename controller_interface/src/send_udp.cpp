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
//データの値、その量、ipアドレス、ポートなどの値を格納
void send_udp::send(const unsigned char* data, int data_len, const string dest_ip, int dest_port)
{
    int sockfd;
    struct sockaddr_in servaddr;
    //通信のエンドポイントを作成
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    //メモリに指定バイト数分の値をセットすることができる
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(dest_ip.c_str());
    servaddr.sin_port = htons(dest_port);
    //sendtoで中に入っている引数をコントローラに送る
    int n = sendto(sockfd, data, data_len, 0, (const struct sockaddr *) &servaddr, sizeof(servaddr));
    if (n < 0) {
        //データの値が0より小さいときはエラーを吐くようにする
        perror("sendto");
    }
    close(sockfd);
}