/* Linux传输准备实现，主机测试只调用纯报文构造函数。 */
#ifndef IMU_CONTROL_RAW_H
#define IMU_CONTROL_RAW_H
#include "imu_iic_control.h"
#define IMU_INTERFACE_NAME_BYTES 16
struct imu_control_raw {
    int fd, interface_index;
    char interface_name[IMU_INTERFACE_NAME_BYTES];
    uint8_t source_mac[6], destination_mac[6], source_ipv4[4], destination_ipv4[4];
};
int imu_control_raw_open(struct imu_control_raw *raw, const char *interface_name,
    const char *source_ipv4, const char *destination_ipv4);
int imu_control_raw_send(struct imu_control_raw *raw, const uint8_t payload[32]);
void imu_control_raw_close(struct imu_control_raw *raw);
/* UDP5007回执套接字与仅发送AF_PACKET描述符相互独立。 */
int imu_control_ack_socket(const char *bind_address);
/* 排空回执，接受READY时返回1，否则返回0，套接字错误返回-1。 */
int imu_control_ack_drain(int fd, const char *peer_address,
                         struct imu_control_session *session);
/* 开始新的READY数据流前丢弃排队的旧角度报文。 */
int imu_control_drop_angles(int fd);
#endif
