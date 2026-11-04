#ifndef ROBOT_UDP_H
#define ROBOT_UDP_H
#include "robot_protocol.h"
#include <netinet/in.h>
struct robot_udp { int fd,filter_peer; struct in_addr peer; uint64_t ignored; };
int robot_udp_open(struct robot_udp *udp,const char *bind_address,const char *peer_address);
int robot_udp_drain(struct robot_udp *udp,struct robot_model *model);
#endif
