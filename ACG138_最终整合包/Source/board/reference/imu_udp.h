#ifndef IMU_UDP_H
#define IMU_UDP_H
#include "imu_protocol.h"
#include <netinet/in.h>
struct imu_udp {
    int fd, filter_peer;
    struct in_addr peer;
    uint64_t ignored_peers;
};
uint64_t imu_now_ms(void);
int imu_udp_open(struct imu_udp *udp, const char *bind_address,
                 const char *peer_address, unsigned port);
/* 非阻塞排空最多64个报文，返回-1表示套接字错误。 */
int imu_udp_drain(struct imu_udp *udp, struct imu_model *model);
#endif
