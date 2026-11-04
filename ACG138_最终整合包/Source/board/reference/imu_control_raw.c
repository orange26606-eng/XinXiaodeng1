/* 仅用于Linux准备检查，此源码尚未针对真实网络设备运行。 */
#define _GNU_SOURCE
#include "imu_control_raw.h"
#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

static int choose_interface(char name[IMU_INTERFACE_NAME_BYTES])
{
    DIR *directory = opendir("/sys/class/net");
    if (!directory) return -1;
    struct dirent *entry;
    unsigned matches = 0;
    while ((entry = readdir(directory))) {
        if (entry->d_name[0] == '.' || strlen(entry->d_name) >= IMU_INTERFACE_NAME_BYTES) continue;
        char path[PATH_MAX], resolved[PATH_MAX];
        snprintf(path, sizeof(path), "/sys/class/net/%s/device", entry->d_name);
        if (realpath(path, resolved) && strstr(resolved, "fe1c0000.ethernet")) {
            memcpy(name, entry->d_name, strlen(entry->d_name) + 1); ++matches;
        }
    }
    closedir(directory);
    if (matches != 1) {
        fprintf(stderr, "Expected one fe1c0000.ethernet interface, found %u; use --interface NAME\n", matches);
        errno = ENODEV; return -1;
    }
    return 0;
}

int imu_control_raw_open(struct imu_control_raw *raw, const char *interface_name,
    const char *source_ipv4, const char *destination_ipv4)
{
    memset(raw, 0, sizeof(*raw)); raw->fd = -1;
    if (inet_pton(AF_INET, source_ipv4, raw->source_ipv4) != 1 ||
        inet_pton(AF_INET, destination_ipv4, raw->destination_ipv4) != 1) {
        errno = EINVAL; return -1;
    }
    const uint8_t fpga_mac[6] = {0x00,0x0A,0x35,0x01,0xFE,0xC0};
    memcpy(raw->destination_mac, fpga_mac, 6);
    if (interface_name && *interface_name) {
        if (strlen(interface_name) >= sizeof(raw->interface_name)) { errno = EINVAL; return -1; }
        memcpy(raw->interface_name, interface_name, strlen(interface_name) + 1);
    } else if (choose_interface(raw->interface_name)) return -1;
    /* 协议设为0，使仅发送套接字不接收相机报文；sendto显式提供ETH_P_IP，不执行ARP查询。
 */
    int fd = socket(AF_PACKET, SOCK_RAW | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        if (errno == EPERM || errno == EACCES)
            fprintf(stderr, "Raw RGMII control needs root or CAP_NET_RAW; no automatic sudo is used.\n");
        return -1;
    }
    struct ifreq request = {0};
    memcpy(request.ifr_name, raw->interface_name, strlen(raw->interface_name) + 1);
    if (ioctl(fd, SIOCGIFINDEX, &request)) goto failed;
    raw->interface_index = request.ifr_ifindex;
    if (ioctl(fd, SIOCGIFHWADDR, &request)) goto failed;
    memcpy(raw->source_mac, request.ifr_hwaddr.sa_data, 6);
    if (ioctl(fd, SIOCGIFFLAGS, &request)) goto failed;
    if (!(request.ifr_flags & IFF_UP)) { errno = ENETDOWN; goto failed; }
    raw->fd = fd;
    fprintf(stderr, "Control interface=%s FPGA MAC=00:0A:35:01:FE:C0\n", raw->interface_name);
    return 0;
failed:
    { int saved = errno; close(fd); errno = saved; return -1; }
}

int imu_control_raw_send(struct imu_control_raw *raw, const uint8_t payload[32])
{
    uint8_t frame[IMU_CONTROL_ETH_BYTES];
    if (imu_control_build_ethernet(frame, raw->source_mac, raw->destination_mac,
                                  raw->source_ipv4, raw->destination_ipv4, payload) != sizeof(frame)) {
        errno = EINVAL; return -1;
    }
    struct sockaddr_ll address = {0};
    address.sll_family = AF_PACKET; address.sll_protocol = htons(ETH_P_IP);
    address.sll_ifindex = raw->interface_index; address.sll_halen = 6;
    memcpy(address.sll_addr, raw->destination_mac, 6);
    ssize_t sent;
    do { sent = sendto(raw->fd, frame, sizeof(frame), 0,
                      (struct sockaddr *)&address, sizeof(address)); } while (sent < 0 && errno == EINTR);
    if (sent != (ssize_t)sizeof(frame)) { if (sent >= 0) errno = EIO; return -1; }
    return 0;
}

void imu_control_raw_close(struct imu_control_raw *raw)
{ if (raw->fd >= 0) { close(raw->fd); raw->fd = -1; } }

int imu_control_ack_socket(const char *bind_address)
{
    struct sockaddr_in address = {.sin_family = AF_INET, .sin_port = htons(IMU_CONTROL_PORT)};
    if (inet_pton(AF_INET, bind_address, &address.sin_addr) != 1) { errno = EINVAL; return -1; }
    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;
    if (bind(fd, (struct sockaddr *)&address, sizeof(address))) {
        int saved = errno; close(fd); errno = saved; return -1;
    }
    return fd;
}

int imu_control_ack_drain(int fd, const char *peer_address, struct imu_control_session *session)
{
    struct in_addr peer;
    if (inet_pton(AF_INET, peer_address, &peer) != 1) { errno = EINVAL; return -1; }
    int became_ready = 0;
    for (unsigned count = 0; count < 64; ++count) {
        uint8_t payload[32]; struct sockaddr_in sender;
        socklen_t length = sizeof(sender);
        ssize_t received = recvfrom(fd, payload, sizeof(payload), MSG_TRUNC,
                                   (struct sockaddr *)&sender, &length);
        if (received < 0) {
            if (errno == EINTR) { --count; continue; }
            if (errno == EAGAIN || errno == EWOULDBLOCK) return became_ready;
            return -1;
        }
        if (sender.sin_addr.s_addr != peer.s_addr || sender.sin_port != htons(IMU_CONTROL_PORT)) continue;
        became_ready |= imu_control_session_receive(session, payload, (size_t)received);
    }
    return became_ready;
}

int imu_control_drop_angles(int fd)
{
    uint8_t discard[64];
    /* 限制每轮处理量，避免连续错误流量阻塞界面；正常50Hz队列通常会提前排空。
 */
    for (unsigned count = 0; count < 4096; ++count) {
        ssize_t received = recv(fd, discard, sizeof(discard), MSG_TRUNC);
        if (received < 0) {
            if (errno == EINTR) { --count; continue; }
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
            return -1;
        }
    }
    errno = EOVERFLOW; return -1;
}
