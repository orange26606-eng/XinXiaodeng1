#define _POSIX_C_SOURCE 200809L
#include "robot_udp.h"
#include "imu_udp.h"
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
int robot_udp_open(struct robot_udp *u,const char *bind_address,const char *peer_address)
{
    memset(u,0,sizeof(*u));u->fd=-1;
    struct sockaddr_in address={.sin_family=AF_INET,.sin_port=htons(ROBOT_UDP_PORT)};
    u->filter_peer=strcmp(peer_address,"any")!=0;
    if(inet_pton(AF_INET,bind_address,&address.sin_addr)!=1 ||
       (u->filter_peer && inet_pton(AF_INET,peer_address,&u->peer)!=1)) { errno=EINVAL;return -1; }
    int fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if(fd<0) return -1;
    if(bind(fd,(struct sockaddr *)&address,sizeof(address))) {
        int saved=errno;close(fd);errno=saved;return -1;
    }
    u->fd=fd;
    fprintf(stderr,"Robot UDP bind=%s:%u peer=%s\n",bind_address,ROBOT_UDP_PORT,peer_address);
    return 0;
}
int robot_udp_drain(struct robot_udp *u,struct robot_model *m)
{
    for(int count=0;count<64;++count) {
        uint8_t packet[ROBOT_PACKET_SIZE];
        struct sockaddr_in from;socklen_t length=sizeof(from);
        ssize_t n=recvfrom(u->fd,packet,sizeof(packet),MSG_TRUNC,(struct sockaddr *)&from,&length);
        if(n<0) {
            if(errno==EINTR) { --count;continue; }
            if(errno==EAGAIN || errno==EWOULDBLOCK) return count;
            return -1;
        }
        if(u->filter_peer && from.sin_addr.s_addr!=u->peer.s_addr) { ++u->ignored;continue; }
        (void)robot_model_push(m,packet,(size_t)n,imu_now_ms());
    }
    return 64;
}
