#define _POSIX_C_SOURCE 200809L
#include "imu_udp.h"
#include <errno.h>
#include <inttypes.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static volatile sig_atomic_t running = 1;
static void stop(int signal_number) { (void)signal_number; running = 0; }

int main(int argc, char **argv)
{
    const char *bind_address = "192.168.0.3", *peer = "192.168.0.2";
    unsigned port = IMU_UDP_PORT;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--bind") && i + 1 < argc) bind_address = argv[++i];
        else if (!strcmp(argv[i], "--peer") && i + 1 < argc) peer = argv[++i];
        else if (!strcmp(argv[i], "--port") && i + 1 < argc) {
            char *end; unsigned long number = strtoul(argv[++i], &end, 10);
            if (*end || number < 1 || number > 65535) return 2;
            port = (unsigned)number;
        } else {
            fprintf(stderr, "Usage: %s [--bind IPv4] [--peer IPv4|any] [--port 5006]\n", argv[0]);
            return strcmp(argv[i], "--help") ? 2 : 0;
        }
    }
    signal(SIGINT, stop); signal(SIGTERM, stop);
    struct imu_udp udp;
    if (imu_udp_open(&udp, bind_address, peer, port)) { perror("IMU UDP open"); return 1; }
    struct imu_model model;
    imu_model_init(&model, imu_now_ms());
    uint64_t last_print = 0;
    int result = 0;
    while (running) {
        struct pollfd fd = {.fd = udp.fd, .events = POLLIN};
        int ready = poll(&fd, 1, 100);
        if (ready < 0) { if (errno == EINTR) continue; perror("UDP poll"); result = 1; break; }
        if ((fd.revents & (POLLERR | POLLHUP | POLLNVAL)) ||
            ((fd.revents & POLLIN) && imu_udp_drain(&udp, &model) < 0)) {
            perror("UDP receive"); result = 1; break;
        }
        uint64_t now = imu_now_ms();
        if (now - last_print >= 200) {
            fprintf(stdout, "state=%d status=%u seq=%08" PRIX32
                    " roll=%+.2f pitch=%+.2f yaw=%+.2f DEG good=%" PRIu64
                    " iic_error=%" PRIu64 " bad=%" PRIu64 " missing=%" PRIu64
                    " order=%" PRIu64 " ignored_peer=%" PRIu64 "\n",
                    imu_model_state(&model, now), model.status, model.sequence,
                    model.degrees[0], model.degrees[1], model.degrees[2], model.good_count,
                    model.error_count, model.invalid_count, model.missing_count,
                    model.order_count, udp.ignored_peers);
            fflush(stdout); last_print = now;
        }
    }
    close(udp.fd);
    return result;
}
