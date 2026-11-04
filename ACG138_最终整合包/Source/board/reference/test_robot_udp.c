#define _POSIX_C_SOURCE 200809L
#include "robot_udp.h"
#include <arpa/inet.h>
#include <assert.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
static void send_packet(int fd,const void *p,size_t n)
{
    struct sockaddr_in to={.sin_family=AF_INET,.sin_port=htons(ROBOT_UDP_PORT)};
    assert(inet_pton(AF_INET,"127.0.0.1",&to.sin_addr)==1);
    assert(sendto(fd,p,n,0,(struct sockaddr *)&to,sizeof(to))==(ssize_t)n);
}
int main(void)
{
    struct robot_udp udp;struct robot_model model={0};
    assert(robot_udp_open(&udp,"127.0.0.1","127.0.0.2")==0);
    int good=socket(AF_INET,SOCK_DGRAM,0),other=socket(AF_INET,SOCK_DGRAM,0);
    assert(good>=0 && other>=0);
    struct sockaddr_in from={.sin_family=AF_INET};
    assert(inet_pton(AF_INET,"127.0.0.2",&from.sin_addr)==1);
    assert(bind(good,(struct sockaddr *)&from,sizeof(from))==0);
    unsigned char packet[33]={'R','B','T','1',1,0,0,2,6};
    uint32_t crc=robot_crc32(packet,28);
    for(unsigned i=0;i<4;++i)packet[28+i]=(unsigned char)(crc>>(8*i));
    send_packet(other,packet,32);send_packet(good,packet,32);
    struct pollfd pfd={.fd=udp.fd,.events=POLLIN};assert(poll(&pfd,1,1000)==1);
    assert(robot_udp_drain(&udp,&model)==2 && udp.ignored==1 && model.received==1);
    assert(model.mm[0]==4500 && model.error[1]==2 && model.error[2]==6);
    send_packet(good,packet,33);assert(poll(&pfd,1,1000)==1);
    assert(robot_udp_drain(&udp,&model)==1 && model.bad_packet);
    close(good);close(other);close(udp.fd);
    puts("PASS robot UDP: real loopback datagrams, peer filtering, truncation and model update");
    return 0;
}
