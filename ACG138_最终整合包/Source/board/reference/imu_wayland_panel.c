/* 独立Wayland客户端，不打开视频设备或修改网络。1.1.1加入真实连接文字和中文标题，初始化、终端确认、UDP及校准流程保持不变。

 */
#define _GNU_SOURCE
#include "imu_canvas.h"
#include "imu_udp.h"
#include "imu_control_raw.h"
#include "imu_terminal_flow.h"
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <inttypes.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/random.h>
#include <termios.h>
#include <wayland-client.h>
#include "xdg-shell-client-protocol.h"
#include "viewporter-client-protocol.h"
#include "xdg-output-client-protocol.h"

struct panel_output {
    struct wl_output *wl;
    struct zxdg_output_v1 *xdg;
    char name[80];
};
struct panel_buffer { struct wl_buffer *wl; uint32_t *pixels; int busy; };
static struct panel_output outputs[16];
static unsigned output_count;
static struct wl_compositor *compositor;
static struct wl_shm *shared_memory;
static struct xdg_wm_base *shell;
static struct wp_viewporter *viewporter;
static struct zxdg_output_manager_v1 *output_manager;
static struct wp_viewport *viewport;
static int configured, destination_width = IMU_CANVAS_WIDTH;
static int destination_height = IMU_CANVAS_HEIGHT;
static volatile sig_atomic_t running = 1;
static void stop(int number) { (void)number; running = 0; }
static void output_geometry(void *data, struct wl_output *output, int32_t x, int32_t y,
    int32_t width, int32_t height, int32_t subpixel, const char *make,
    const char *model, int32_t transform)
{ (void)data; (void)output; (void)x; (void)y; (void)width; (void)height;
  (void)subpixel; (void)make; (void)model; (void)transform; }
static void output_mode(void *data, struct wl_output *output, uint32_t flags,
    int32_t width, int32_t height, int32_t refresh)
{ (void)data; (void)output; (void)flags; (void)width; (void)height; (void)refresh; }
static void output_done(void *data, struct wl_output *output) { (void)data; (void)output; }
static void output_scale(void *data, struct wl_output *output, int32_t scale)
{ (void)data; (void)output; (void)scale; }
static void output_name(void *data, struct wl_output *output, const char *name)
{ (void)output; snprintf(((struct panel_output *)data)->name, 80, "%s", name); }
static void output_description(void *data, struct wl_output *output, const char *description)
{ (void)data; (void)output; (void)description; }
static const struct wl_output_listener output_listener = {
    .geometry = output_geometry, .mode = output_mode, .done = output_done,
    .scale = output_scale, .name = output_name, .description = output_description
};
static void logical_position(void *data, struct zxdg_output_v1 *output, int32_t x, int32_t y)
{ (void)data; (void)output; (void)x; (void)y; }
static void logical_size(void *data, struct zxdg_output_v1 *output, int32_t w, int32_t h)
{ (void)data; (void)output; (void)w; (void)h; }
static void logical_done(void *data, struct zxdg_output_v1 *output) { (void)data; (void)output; }
static void logical_name(void *data, struct zxdg_output_v1 *output, const char *name)
{ (void)output; snprintf(((struct panel_output *)data)->name, 80, "%s", name); }
static void logical_description(void *data, struct zxdg_output_v1 *output, const char *description)
{ (void)data; (void)output; (void)description; }
static const struct zxdg_output_v1_listener logical_listener = {
    .logical_position = logical_position, .logical_size = logical_size,
    .done = logical_done, .name = logical_name, .description = logical_description
};
static void ping(void *data, struct xdg_wm_base *base, uint32_t serial)
{ (void)data; xdg_wm_base_pong(base, serial); }
static const struct xdg_wm_base_listener shell_listener = {.ping = ping};
static void global(void *data, struct wl_registry *registry, uint32_t id,
                    const char *interface, uint32_t version)
{
    (void)data;
    if (!strcmp(interface, "wl_compositor"))
        compositor = wl_registry_bind(registry, id, &wl_compositor_interface, version < 4 ? version : 4);
    else if (!strcmp(interface, "wl_shm"))
        shared_memory = wl_registry_bind(registry, id, &wl_shm_interface, 1);
    else if (!strcmp(interface, "xdg_wm_base")) {
        shell = wl_registry_bind(registry, id, &xdg_wm_base_interface, 1);
        xdg_wm_base_add_listener(shell, &shell_listener, NULL);
    } else if (!strcmp(interface, "wp_viewporter"))
        viewporter = wl_registry_bind(registry, id, &wp_viewporter_interface, 1);
    else if (!strcmp(interface, "zxdg_output_manager_v1"))
        output_manager = wl_registry_bind(registry, id, &zxdg_output_manager_v1_interface, version < 3 ? version : 3);
    else if (!strcmp(interface, "wl_output") && version >= 2 && output_count < 16) {
        struct panel_output *output = &outputs[output_count++];
        output->wl = wl_registry_bind(registry, id, &wl_output_interface, version < 4 ? version : 4);
        wl_output_add_listener(output->wl, &output_listener, output);
    }
}
static void removed(void *data, struct wl_registry *registry, uint32_t id)
{ (void)data; (void)registry; (void)id; }
static const struct wl_registry_listener registry_listener = {.global = global, .global_remove = removed};
static void top_configure(void *data, struct xdg_toplevel *top, int32_t width,
                          int32_t height, struct wl_array *states)
{
    (void)data; (void)top; (void)states;
    if (width > 0 && width <= 16384) destination_width = width;
    if (height > 0 && height <= 16384) destination_height = height;
}
static void top_close(void *data, struct xdg_toplevel *top) { (void)data; (void)top; running = 0; }
static const struct xdg_toplevel_listener top_listener = {.configure = top_configure, .close = top_close};
static void surface_configure(void *data, struct xdg_surface *surface, uint32_t serial)
{
    (void)data;
    xdg_surface_ack_configure(surface, serial);
    wp_viewport_set_destination(viewport, destination_width, destination_height);
    xdg_surface_set_window_geometry(surface, 0, 0, destination_width, destination_height);
    configured = 1;
}
static const struct xdg_surface_listener surface_listener = {.configure = surface_configure};
static void released(void *data, struct wl_buffer *buffer)
{ (void)buffer; ((struct panel_buffer *)data)->busy = 0; }
static const struct wl_buffer_listener buffer_listener = {.release = released};

static int make_buffer(struct panel_buffer *buffer)
{
    const size_t size = (size_t)IMU_CANVAS_WIDTH * IMU_CANVAS_HEIGHT * 4;
    int fd = memfd_create("fpga-imu-panel", MFD_CLOEXEC);
    if (fd < 0) return -1;
    if (ftruncate(fd, (off_t)size)) { int saved = errno; close(fd); errno = saved; return -1; }
    buffer->pixels = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (buffer->pixels == MAP_FAILED) { buffer->pixels = NULL; close(fd); return -1; }
    struct wl_shm_pool *pool = wl_shm_create_pool(shared_memory, fd, (int32_t)size);
    if (!pool) { close(fd); errno = ENOMEM; return -1; }
    buffer->wl = wl_shm_pool_create_buffer(pool, 0, IMU_CANVAS_WIDTH,
                    IMU_CANVAS_HEIGHT, IMU_CANVAS_WIDTH * 4, WL_SHM_FORMAT_XRGB8888);
    if (!buffer->wl) { wl_shm_pool_destroy(pool); close(fd); errno = ENOMEM; return -1; }
    wl_buffer_add_listener(buffer->wl, &buffer_listener, buffer);
    wl_shm_pool_destroy(pool); close(fd);
    return 0;
}

/* 此时尚未创建表面、顶层窗口、共享缓冲或可见窗口，只处理不可见启动预检和Wayland探测。
 */
static int terminal_calibration(struct wl_display *display, struct imu_udp *udp,
    int ack_fd, struct imu_control_raw *raw, const char *peer, struct imu_terminal_flow *flow)
{
    while (running && !imu_terminal_flow_may_open_window(flow) && !imu_terminal_flow_failed(flow)) {
        while (wl_display_prepare_read(display) != 0)
            if (wl_display_dispatch_pending(display) < 0) return -1;
        if (!running) { wl_display_cancel_read(display); return -1; }
        int want_write = 0;
        if (wl_display_flush(display) < 0) {
            if (errno == EAGAIN) want_write = 1;
            else { wl_display_cancel_read(display); return -1; }
        }
        struct pollfd fds[4] = {
            {.fd = wl_display_get_fd(display), .events = (short)(POLLIN | (want_write ? POLLOUT : 0))},
            {.fd = udp->fd, .events = POLLIN},
            {.fd = ack_fd, .events = POLLIN},
            {.fd = STDIN_FILENO, .events = POLLIN}
        };
        int ready = poll(fds, 4, 50);
        if (ready < 0) {
            wl_display_cancel_read(display);
            if (errno == EINTR) continue;
            perror("Terminal calibration poll"); return -1;
        }
        if (fds[0].revents & POLLIN) {
            if (wl_display_read_events(display) < 0) return -1;
        } else wl_display_cancel_read(display);
        if (!running) return -1;
        if ((fds[0].revents | fds[1].revents | fds[2].revents) & (POLLERR | POLLHUP | POLLNVAL)) return -1;
        if (wl_display_dispatch_pending(display) < 0 || !running) return -1;
        /* 同次轮询收到断开和READY时，优先处理断开。 */
        if (fds[3].revents & (POLLERR | POLLHUP | POLLNVAL)) {
            imu_terminal_flow_disconnect(flow);
            fprintf(stderr, "Terminal disconnected; no attitude window opened.\n"); return -1;
        }
        /* 先处理早期输入再处理ACK；WAIT切换后清空旧行及不完整输入。 */


        if (fds[3].revents & POLLIN) {
            char input[128]; ssize_t count = read(STDIN_FILENO, input, sizeof(input));
            if (!count) {
                imu_terminal_flow_disconnect(flow);
                fprintf(stderr, "Terminal EOF; no attitude window opened.\n"); return -1;
            }
            if (count < 0) {
                if (errno != EINTR && errno != EAGAIN) {
                    imu_terminal_flow_disconnect(flow); perror("Terminal read"); return -1;
                }
            } else if (imu_terminal_flow_input(flow, input, (size_t)count, imu_now_ms())) {
                fprintf(stderr, "User reported IMU D3 stopped blinking; requesting FPGA READY.\n");
            }
        }
        if (!running) return -1;
        if (fds[2].revents & POLLIN) {
            if (imu_control_ack_drain(ack_fd, peer, &flow->control) < 0) {
                perror("Control ACK receive"); imu_control_session_io_failed(&flow->control); return -1;
            }
        }
        if (imu_terminal_flow_prompt_needed(flow)) {
            if (tcflush(STDIN_FILENO, TCIFLUSH)) { perror("Flush early terminal input"); return -1; }
            if (!running) return -1;
            (void)imu_terminal_flow_after_input_flush(flow);
            fprintf(stderr,
                "\n九轴算法已写入并读回确认，校准开始命令已获得 IIC ACK。\n"
                "请保持 IMU 水平静止，观察 IMU 模块 D3 绿色灯（不是 FPGA 开发板 D3）。\n"
                "亲眼确认 D3 停止闪烁后，输入 y 并回车；收到 READY 后才打开姿态角窗口。\n"
                "IMU D3 stopped blinking? Type y then Enter: ");
            if (fflush(stderr)) { perror("Terminal prompt"); return -1; }
        }
        if ((fds[1].revents & POLLIN) && imu_control_drop_angles(udp->fd)) {
            perror("Discarding queued pre-window angle packets"); return -1;
        }
        if (!running || flow->stopped) return -1;
        uint8_t command[32];
        if (imu_terminal_flow_next_tx(flow, imu_now_ms(), command) && running &&
            imu_control_raw_send(raw, command)) {
            perror("Raw FPGA command send"); imu_control_session_io_failed(&flow->control); return -1;
        }
    }
    if (!running || flow->stopped) return -1;
    if (imu_terminal_flow_failed(flow)) {
        if (flow->control.timed_out) fprintf(stderr, "Control timeout; no attitude window opened.\n");
        else if (flow->control.busy) fprintf(stderr, "FPGA session BUSY; use the printed previous token to resume.\n");
        else fprintf(stderr, "FPGA control FAILED error=%u; no attitude window opened.\n", flow->control.error);
        return -1;
    }
    return imu_terminal_flow_may_open_window(flow) ? 0 : -1;
}

static void usage(const char *program)
{
    fprintf(stderr, "Usage: %s [--bind IPv4] [--peer IPv4] [--port 5006]\n"
            "       [--interface NAME] [--fullscreen OUTPUT] [--list-outputs]\n"
            "       [--resume-token 0x12345678] (explicit interrupted-session recovery)\n"
            "Default: interactive terminal setup and y+Enter first; window only after READY.\n", program);
}

int main(int argc, char **argv)
{
    const char *bind_address = "192.168.0.3", *peer = "192.168.0.2", *fullscreen = NULL;
    const char *interface_name = NULL;
    int resume = 0; uint32_t resume_token = 0;
    unsigned port = IMU_UDP_PORT;
    int list_only = 0, result = 1;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--bind") && i + 1 < argc) bind_address = argv[++i];
        else if (!strcmp(argv[i], "--peer") && i + 1 < argc) peer = argv[++i];
        else if (!strcmp(argv[i], "--interface") && i + 1 < argc) interface_name = argv[++i];
        else if (!strcmp(argv[i], "--resume-token") && i + 1 < argc) {
            char *end; errno = 0; unsigned long value = strtoul(argv[++i], &end, 0);
            if (errno || *end || end == argv[i] || value > UINT32_MAX) { usage(argv[0]); return 2; }
            resume = 1; resume_token = (uint32_t)value;
        }
        else if (!strcmp(argv[i], "--fullscreen") && i + 1 < argc) fullscreen = argv[++i];
        else if (!strcmp(argv[i], "--list-outputs")) list_only = 1;
        else if (!strcmp(argv[i], "--port") && i + 1 < argc) {
            char *end; unsigned long number = strtoul(argv[++i], &end, 10);
            if (*end || !number || number > 65535) { usage(argv[0]); return 2; }
            port = (unsigned)number;
        } else { usage(argv[0]); return strcmp(argv[i], "--help") ? 2 : 0; }
    }
    if (!list_only && !isatty(STDIN_FILENO)) {
        fprintf(stderr, "An interactive RK terminal is required; no calibration request sent.\n"); return 1;
    }
    signal(SIGINT, stop); signal(SIGTERM, stop);
    struct imu_udp udp = {.fd = -1};
    struct imu_control_raw raw = {.fd = -1};
    int ack_fd = -1;
    struct panel_buffer buffers[2] = {{0}, {0}};
    struct wl_surface *surface = NULL;
    struct xdg_surface *xdg_surface = NULL;
    struct xdg_toplevel *top = NULL;
    struct wl_display *display = wl_display_connect(NULL);
    if (!display) { perror("Wayland connect (check XDG_RUNTIME_DIR/WAYLAND_DISPLAY)"); return 1; }
    struct wl_registry *registry = wl_display_get_registry(display);
    if (!registry) { wl_display_disconnect(display); return 1; }
    wl_registry_add_listener(registry, &registry_listener, NULL);
    if (wl_display_roundtrip(display) < 0 || wl_display_roundtrip(display) < 0) goto cleanup;
    if (output_manager) {
        for (unsigned i = 0; i < output_count; ++i) {
            outputs[i].xdg = zxdg_output_manager_v1_get_xdg_output(output_manager, outputs[i].wl);
            zxdg_output_v1_add_listener(outputs[i].xdg, &logical_listener, &outputs[i]);
        }
        if (wl_display_roundtrip(display) < 0) goto cleanup;
    }
    struct wl_output *target = NULL;
    for (unsigned i = 0; i < output_count; ++i) {
        fprintf(stderr, "Wayland output=%s\n", outputs[i].name[0] ? outputs[i].name : "(unnamed)");
        if (fullscreen && !strcmp(outputs[i].name, fullscreen)) target = outputs[i].wl;
    }
    if (list_only) { result = 0; goto cleanup; }
    if (!compositor || !shared_memory || !shell || !viewporter || (fullscreen && !target)) {
        fprintf(stderr, "Required Wayland protocol or requested output missing\n"); goto cleanup;
    }
    uint32_t token = 0;
    size_t token_bytes = 0;
    while (!resume && token_bytes < sizeof(token)) {
        ssize_t got = getrandom((uint8_t *)&token + token_bytes, sizeof(token) - token_bytes, 0);
        if (got < 0) { if (errno == EINTR) continue; perror("control random token"); goto cleanup; }
        if (!got) { fprintf(stderr, "Cannot create control token\n"); goto cleanup; }
        token_bytes += (size_t)got;
    }
    if (resume) token = resume_token;
    struct imu_terminal_flow flow;
    imu_terminal_flow_init(&flow, token, resume, imu_now_ms());
    fprintf(stderr, "IMU session token=0x%08" PRIX32 " mode=%s\n", token,
            resume ? "EXPLICIT RESUME (no new calibration if already READY)" : "NEW CALIBRATION");
    fprintf(stderr, "Initializing nine-axis algorithm and starting Accel/Gyro calibration.\n"
                    "Keep IMU flat and still. Early input is ignored; wait for the y+Enter prompt.\n");
    ack_fd = imu_control_ack_socket(bind_address);
    if (ack_fd < 0) { perror("Control ACK bind UDP5007"); goto cleanup; }
    if (imu_control_raw_open(&raw, interface_name, bind_address, peer)) {
        perror("Raw FPGA control (root/CAP_NET_RAW required)"); goto cleanup;
    }
    if (imu_udp_open(&udp, bind_address, peer, port)) { perror("IMU UDP open"); goto cleanup; }
    if (terminal_calibration(display, &udp, ack_fd, &raw, peer, &flow)) goto cleanup;
    if (!running || !imu_terminal_flow_may_open_window(&flow)) goto cleanup;
    if (imu_control_drop_angles(udp.fd)) { perror("Discard queued angles before window"); goto cleanup; }
    imu_canvas_set_connection(raw.interface_name, bind_address, peer, port);
    close(ack_fd); ack_fd = -1; imu_control_raw_close(&raw);
    fprintf(stderr, "FPGA READY acknowledged. Opening the separate attitude window.\n");
    if (!running) goto cleanup;
    surface = wl_compositor_create_surface(compositor);
    if (!surface) { fprintf(stderr, "Cannot create Wayland surface\n"); goto cleanup; }
    viewport = wp_viewporter_get_viewport(viewporter, surface);
    xdg_surface = xdg_wm_base_get_xdg_surface(shell, surface);
    if (!viewport || !xdg_surface) { fprintf(stderr, "Cannot allocate Wayland surface objects\n"); goto cleanup; }
    xdg_surface_add_listener(xdg_surface, &surface_listener, NULL);
    top = xdg_surface_get_toplevel(xdg_surface);
    if (!top) { fprintf(stderr, "Cannot allocate Wayland toplevel\n"); goto cleanup; }
    xdg_toplevel_add_listener(top, &top_listener, NULL);
    xdg_toplevel_set_title(top, "FPGA IMU 姿态调试");
    xdg_toplevel_set_app_id(top, "acg138-imu-panel");
    xdg_toplevel_set_min_size(top, IMU_CANVAS_WIDTH / 2, IMU_CANVAS_HEIGHT / 2);
    if (fullscreen) xdg_toplevel_set_fullscreen(top, target);
    wl_surface_commit(surface);
    if (make_buffer(&buffers[0]) || make_buffer(&buffers[1])) { perror("Wayland shared buffer"); goto cleanup; }
    struct imu_model model;
    imu_model_init(&model, imu_now_ms());
    uint64_t last_draw = 0;
    result = 0;
    while (running) {
        while (wl_display_prepare_read(display) != 0)
            if (wl_display_dispatch_pending(display) < 0) { result = 1; goto cleanup; }
        int want_write = 0;
        if (wl_display_flush(display) < 0) {
            if (errno == EAGAIN) want_write = 1;
            else { wl_display_cancel_read(display); result = 1; break; }
        }
        struct pollfd fds[2] = {
            {.fd = wl_display_get_fd(display), .events = (short)(POLLIN | (want_write ? POLLOUT : 0))},
            {.fd = udp.fd, .events = POLLIN}
        };
        int ready = poll(fds, 2, 50);
        if (ready < 0) {
            wl_display_cancel_read(display);
            if (errno == EINTR) continue;
            perror("Wayland/UDP poll"); result = 1; break;
        }
        if (fds[0].revents & POLLIN) {
            if (wl_display_read_events(display) < 0) { result = 1; break; }
        } else wl_display_cancel_read(display);
        if ((fds[0].revents | fds[1].revents) & (POLLERR | POLLHUP | POLLNVAL)) { result = 1; break; }
        if (wl_display_dispatch_pending(display) < 0) { result = 1; break; }
        if (!running) break; /* 窗口关闭事件不发送校准START命令。 */
        if (fds[1].revents & POLLIN) {
            if (imu_udp_drain(&udp, &model) < 0) { perror("IMU UDP receive"); result = 1; break; }
        }
        uint64_t now = imu_now_ms();
        if (configured && now - last_draw >= 50) {
            for (unsigned i = 0; i < 2; ++i) if (!buffers[i].busy) {
                imu_canvas_draw(buffers[i].pixels, &model, now);
                buffers[i].busy = 1;
                wl_surface_attach(surface, buffers[i].wl, 0, 0);
                wl_surface_damage(surface, 0, 0, INT_MAX, INT_MAX);
                wl_surface_commit(surface);
                last_draw = now; break;
            }
        }
    }
cleanup:
    if (ack_fd >= 0) close(ack_fd);
    imu_control_raw_close(&raw);
    if (udp.fd >= 0) close(udp.fd);
    for (unsigned i = 0; i < 2; ++i) {
        if (buffers[i].wl) wl_buffer_destroy(buffers[i].wl);
        if (buffers[i].pixels) munmap(buffers[i].pixels, (size_t)IMU_CANVAS_WIDTH * IMU_CANVAS_HEIGHT * 4);
    }
    if (top) xdg_toplevel_destroy(top);
    if (xdg_surface) xdg_surface_destroy(xdg_surface);
    if (viewport) wp_viewport_destroy(viewport);
    if (surface) wl_surface_destroy(surface);
    for (unsigned i = 0; i < output_count; ++i) {
        if (outputs[i].xdg) zxdg_output_v1_destroy(outputs[i].xdg);
        wl_output_destroy(outputs[i].wl);
    }
    if (output_manager) zxdg_output_manager_v1_destroy(output_manager);
    if (viewporter) wp_viewporter_destroy(viewporter);
    if (shell) xdg_wm_base_destroy(shell);
    if (shared_memory) wl_shm_destroy(shared_memory);
    if (compositor) wl_compositor_destroy(compositor);
    wl_registry_destroy(registry);
    wl_display_disconnect(display);
    return result;
}
