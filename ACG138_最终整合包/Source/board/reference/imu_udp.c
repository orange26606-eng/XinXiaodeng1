#define _POSIX_C_SOURCE 200809L
#include "imu_udp.h"
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

uint64_t imu_now_ms(void)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now)) { perror("CLOCK_MONOTONIC"); return 0; }
    return (uint64_t)now.tv_sec * 1000u + (uint64_t)now.tv_nsec / 1000000u;
}

int imu_udp_open(struct imu_udp *udp, const char *bind_address,
                 const char *peer_address, unsigned port)
{
    memset(udp, 0, sizeof(*udp));
    udp->fd = -1;
    struct sockaddr_in address = {.sin_family = AF_INET, .sin_port = htons((uint16_t)port)};
    if (!port || port > 65535 || inet_pton(AF_INET, bind_address, &address.sin_addr) != 1) {
        fprintf(stderr, "Invalid bind IPv4 address or UDP port\n"); errno = EINVAL; return -1;
    }
    udp->filter_peer = strcmp(peer_address, "any") != 0;
    if (udp->filter_peer && inet_pton(AF_INET, peer_address, &udp->peer) != 1) {
        fprintf(stderr, "Invalid peer IPv4 address; use any to disable peer filter\n");
        errno = EINVAL; return -1;
    }
    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;
    /* UDP接收器独占绑定。 */
    if (bind(fd, (struct sockaddr *)&address, sizeof(address))) {
        int saved = errno; close(fd); errno = saved; return -1;
    }
    udp->fd = fd;
    fprintf(stderr, "IMU UDP bind=%s:%u peer=%s\n", bind_address, port, peer_address);
    return 0;
}

int imu_udp_drain(struct imu_udp *udp, struct imu_model *model)
{
    int count = 0;
    while (count < 64) {
        uint8_t packet[IMU_PACKET_SIZE];
        struct sockaddr_in sender;
        socklen_t sender_length = sizeof(sender);
        /* Linux的MSG_TRUNC在缓冲不足时仍报告完整报文长度。 */
        ssize_t length = recvfrom(udp->fd, packet, sizeof(packet), MSG_TRUNC,
                                  (struct sockaddr *)&sender, &sender_length);
        if (length < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return count;
            return -1;
        }
        ++count;
        if (udp->filter_peer && sender.sin_addr.s_addr != udp->peer.s_addr) {
            ++udp->ignored_peers;
            continue;
        }
        imu_model_push(model, packet, (size_t)length, imu_now_ms());
    }
    return count;
}
