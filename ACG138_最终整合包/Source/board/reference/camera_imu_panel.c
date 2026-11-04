/* 由独立IMU面板扩展的Wayland客户端，保留相机与姿态布局及原报文格式。版本依次增加中文标题、真实连接标记、IMU静置等待与新鲜数据启动检查、V4L2预览、采集重启前释放缓冲，以及最新检测和参考航向显示，图像保持等比例适配。





 */
#define _GNU_SOURCE
#include "dashboard_canvas.h"
#include "dashboard_video.h"
#include "panel_outputs.h"
#include "imu_udp.h"
#include "../phone_export.h"
#include "../odometry_runtime.h"
#include "imu_control_raw.h"
#include "imu_auto_start.h"
#include "robot_udp.h"
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
#include <wayland-client.h>
#include <arpa/inet.h>
#include "xdg-shell-client-protocol.h"
#include "viewporter-client-protocol.h"
#include "xdg-output-client-protocol.h"

struct panel_buffer { struct wl_buffer *wl; uint32_t *pixels; int busy; };
static struct panel_outputs output_state;
static struct wl_compositor *compositor;
static struct wl_shm *shared_memory;
static struct xdg_wm_base *shell;
static struct wp_viewporter *viewporter;
static struct wp_viewport *viewport;
static int configured, destination_width = DASHBOARD_WIDTH;
static int destination_height = DASHBOARD_HEIGHT;
static volatile sig_atomic_t running = 1;
static void stop(int number) { (void)number; running = 0; }
static const char *video_ready_file, *video_released_file, *video_device;
static int video_release_acknowledged;
static uint64_t video_opened_ms, video_error_ms;

/* 文件位于本会话mktemp私有目录。生产者退出前删除ready标记，管理器等到released后才启动新写入端，避免现存MMAP采集阻止S_FMT；IMU自动启动也使用同一确认流程。


 */
static int video_generation_sync(struct dashboard_video *video, int allow_open,
                                 uint64_t now_ms)
{
    int ready = 1;
    if (video_ready_file && access(video_ready_file, F_OK)) {
        if (errno != ENOENT) { perror("Video ready marker"); return -1; }
        ready = 0;
    }
    if (!ready) {
producer_paused:
        if (video && video->fd >= 0) dashboard_video_close(video);
        video_error_ms = 0;
        if (!video_release_acknowledged && video_released_file) {
            int fd = open(video_released_file, O_CREAT | O_WRONLY | O_TRUNC | O_CLOEXEC | O_NOFOLLOW, 0600);
            if (fd < 0) { perror("Video release marker"); return -1; }
            if (close(fd)) { perror("Close video release marker"); return -1; }
            video_release_acknowledged = 1;
            fprintf(stderr, "Camera capture released; waiting for the next batch.\n");
        }
        return 0;
    }
    if (video_release_acknowledged && video_released_file &&
        unlink(video_released_file) && errno != ENOENT) {
        perror("Clear video release marker"); return -1;
    }
    video_release_acknowledged = 0;
    if (video_error_ms) {
        if (now_ms - video_error_ms > 7000) {
            fprintf(stderr, "Camera capture failed while its producer remained active.\n"); return -1;
        }
        return 0;
    }
    if (allow_open && video && video->fd < 0) {
        if (dashboard_video_open(video, video_device, DASHBOARD_VIDEO_WIDTH, DASHBOARD_VIDEO_HEIGHT)) {
            if (video_ready_file && access(video_ready_file, F_OK) && errno == ENOENT) goto producer_paused;
            return -1;
        }
        video_opened_ms = now_ms;
    }
    return 0;
}
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
    else panel_outputs_global(&output_state, registry, id, interface, version);
}
static void removed(void *data, struct wl_registry *registry, uint32_t id)
{ (void)data; (void)registry; panel_outputs_remove(&output_state, id); }
static const struct wl_registry_listener registry_listener = {.global = global, .global_remove = removed};
/* 切换输出时重定向现有窗口，采集与IMU会话独立运行。 */

static void output_reselect(struct xdg_toplevel *top, struct wl_surface *surface)
{
    struct panel_output_selection selection;
    if (!panel_outputs_update(&output_state, &selection)) return;
    if (selection.windowed) {
        xdg_toplevel_unset_fullscreen(top);
        fprintf(stderr, "Wayland display mode=windowed\n");
    } else if (selection.wl) {
        xdg_toplevel_set_fullscreen(top, selection.wl);
        fprintf(stderr, "Wayland display selected=%s id=%u policy=%s (same camera/IMU session)\n",
                *selection.name ? selection.name : "(unnamed)", selection.id,
                output_state.mode ? output_state.mode : "auto");
    } else {
        xdg_toplevel_unset_fullscreen(top);
        fprintf(stderr, "Wayland display waiting: policy=%s; camera/IMU session remains active\n",
                output_state.mode ? output_state.mode : "auto");
    }
    wl_surface_commit(surface);
}
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
    const size_t size = (size_t)DASHBOARD_WIDTH * DASHBOARD_HEIGHT * 4;
    int fd = memfd_create("fpga-camera-imu-panel", MFD_CLOEXEC);
    if (fd < 0) return -1;
    if (ftruncate(fd, (off_t)size)) { int saved = errno; close(fd); errno = saved; return -1; }
    buffer->pixels = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (buffer->pixels == MAP_FAILED) { buffer->pixels = NULL; close(fd); return -1; }
    struct wl_shm_pool *pool = wl_shm_create_pool(shared_memory, fd, (int32_t)size);
    if (!pool) { close(fd); errno = ENOMEM; return -1; }
    buffer->wl = wl_shm_pool_create_buffer(pool, 0, DASHBOARD_WIDTH,
                    DASHBOARD_HEIGHT, DASHBOARD_WIDTH * 4, WL_SHM_FORMAT_XRGB8888);
    if (!buffer->wl) { wl_shm_pool_destroy(pool); close(fd); errno = ENOMEM; return -1; }
    wl_buffer_add_listener(buffer->wl, &buffer_listener, buffer);
    wl_shm_pool_destroy(pool); close(fd);
    return 0;
}

static void usage(const char *program)
{
    fprintf(stderr, "Usage: %s --video /dev/video61 [--bind IPv4] [--peer IPv4] [--port 5016]\n"
            "       [--interface NAME] [--fullscreen auto|OUTPUT | --windowed] [--list-outputs]\n"
            "       [--video-ready-file PATH --video-released-file PATH]\n"
            "       [--detections PATH] (latest complete YOLO log result)\n"
            "       [--yolo-fps-file PATH] (overrides YOLO_FPS_FILE environment)\n"
            "       [--resume-token 0x12345678] (explicit interrupted-session recovery)\n"
            "Display default: auto, HDMI first then DSI; hotplug keeps the same session.\n"
            "Default: RK gyro bias calibration, keep still about 2 seconds; no IMC1 requests.\n", program);
}

int main(int argc, char **argv)
{
    const char *bind_address = "192.168.0.3", *peer = "192.168.0.2", *fullscreen = "auto";
    const char *interface_name = NULL, *video_path = NULL;
    const char *detections_path = NULL;
    const char *yolo_fps_path = getenv("YOLO_FPS_FILE");
    int resume = 0; uint32_t resume_token = 0;
    unsigned port = 5016;
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
        else if (!strcmp(argv[i], "--video") && i + 1 < argc) video_path = argv[++i];
        else if (!strcmp(argv[i], "--detections") && i + 1 < argc) detections_path = argv[++i];
        else if (!strcmp(argv[i], "--yolo-fps-file") && i + 1 < argc) yolo_fps_path = argv[++i];
        else if (!strcmp(argv[i], "--video-ready-file") && i + 1 < argc) video_ready_file = argv[++i];
        else if (!strcmp(argv[i], "--video-released-file") && i + 1 < argc) video_released_file = argv[++i];
        else if (!strcmp(argv[i], "--fullscreen") && i + 1 < argc) fullscreen = argv[++i];
        else if (!strcmp(argv[i], "--windowed")) fullscreen = "windowed";
        else if (!strcmp(argv[i], "--list-outputs")) list_only = 1;
        else if (!strcmp(argv[i], "--port") && i + 1 < argc) {
            char *end; unsigned long number = strtoul(argv[++i], &end, 10);
            if (*end || !number || number > 65535) { usage(argv[0]); return 2; }
            port = (unsigned)number;
        } else { usage(argv[0]); return strcmp(argv[i], "--help") ? 2 : 0; }
    }
    if (!!video_ready_file != !!video_released_file) { usage(argv[0]); return 2; }
    if (!list_only && (!video_path || strncmp(video_path, "/dev/video", 10))) {
        fprintf(stderr, "Specify --video /dev/video61 (YOLO) or /dev/video60 (ISP).\n"); return 2;
    }
    video_device = video_path;
    panel_outputs_init(&output_state, fullscreen);
    if (!list_only && !isatty(STDIN_FILENO)) {
        fprintf(stderr, "Use the existing interactive RK launcher.\n"); return 1;
    }
    signal(SIGINT, stop); signal(SIGTERM, stop);
    
    struct robot_udp robot_udp = {.fd = -1};
    struct robot_model robot = {0};
    
    
    struct dashboard_video video = {.fd = -1};
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
    /* xdg-output名称可能晚于get_xdg_output请求到达。 */
    if (wl_display_roundtrip(display) < 0) goto cleanup;
    for (unsigned i = 0; i < PANEL_OUTPUT_LIMIT; ++i) if (output_state.entries[i].present)
        fprintf(stderr, "Wayland output=%s\n", output_state.entries[i].name[0] ? output_state.entries[i].name : "(unnamed)");
    if (list_only) { result = 0; goto cleanup; }
    if (!compositor || !shared_memory || !shell || !viewporter) {
        fprintf(stderr, "Required Wayland protocol missing\n"); goto cleanup;
    }
    (void)interface_name;(void)resume;(void)resume_token;
    if(port!=5016){fprintf(stderr,"MPU9250 integrated receiver requires --port 5016\n");goto cleanup;}
    if(robot_udp_open(&robot_udp,bind_address,peer)){perror("Robot UDP open");goto cleanup;}
    if(odom_runtime_start(bind_address,peer)){
        fprintf(stderr,"MPU9250 receiver unavailable; camera/phone/control retained, pose waiting.\n");
    }
    fprintf(stderr,"MPU9250: keep vehicle still for gyro bias calibration (about 2s of new data). No old IMC1 handshake.\n");
    if (video_generation_sync(&video, 1, imu_now_ms())) goto cleanup;
    fprintf(stderr, "MPU9250 passive receiver started. Opening camera/attitude window; calibration status is independent.\n");
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
    xdg_toplevel_set_title(top, "FPGA 双摄像头与 IMU 姿态");
    xdg_toplevel_set_app_id(top, "acg138-camera-imu-panel");
    xdg_toplevel_set_min_size(top, DASHBOARD_WIDTH / 2, DASHBOARD_HEIGHT / 2);
    output_reselect(top, surface);
    wl_surface_commit(surface);
    if (make_buffer(&buffers[0]) || make_buffer(&buffers[1])) { perror("Wayland shared buffer"); goto cleanup; }
    struct imu_model model;imu_model_init(&model,imu_now_ms());
    struct dashboard_heading heading;
    dashboard_heading_init(&heading);
    struct dashboard_rate rate;
    dashboard_rate_init(&rate, model.good_count, imu_now_ms());
    struct dashboard_detections detections = {0};
    struct dashboard_yolo_fps yolo_fps = {0};
    uint64_t yolo_fps_poll_ms = 0;
    if(yolo_fps_path && !*yolo_fps_path)yolo_fps_path=NULL;
    uint64_t detection_poll_ms = 0;
    uint64_t last_draw = 0, video_poll_ms = 0, window_start = imu_now_ms(), last_log = 0;
    result = 0;
    while (running) {
        if (video_generation_sync(&video, 1, imu_now_ms())) { result = 1; break; }
        while (wl_display_prepare_read(display) != 0)
            if (wl_display_dispatch_pending(display) < 0) { result = 1; goto cleanup; }
        int want_write = 0;
        if (wl_display_flush(display) < 0) {
            if (errno == EAGAIN) want_write = 1;
            else { wl_display_cancel_read(display); result = 1; break; }
        }
        struct pollfd fds[4] = {
            {.fd = wl_display_get_fd(display), .events = (short)(POLLIN | (want_write ? POLLOUT : 0))},
            {.fd = -1, .events = 0},
            {.fd = video.fd, .events = (short)(imu_now_ms() - video_poll_ms >= 33 ? POLLIN : 0)},
            {.fd = robot_udp.fd, .events = POLLIN}
        };
        uint64_t poll_now=imu_now_ms();
        uint64_t next_video=video.fd>=0?video_poll_ms+33:poll_now+10;
        uint64_t next_draw=configured?last_draw+33:poll_now+10;
        uint64_t next_due=next_video<next_draw?next_video:next_draw;
        int wait_ms=next_due<=poll_now?1:(next_due-poll_now>10?10:(int)(next_due-poll_now));
        int ready = poll(fds, 4, wait_ms);
        if (ready < 0) {
            wl_display_cancel_read(display);
            if (errno == EINTR) continue;
            perror("Wayland/UDP/video poll"); result = 1; break;
        }
        if (fds[0].revents & POLLIN) {
            if (wl_display_read_events(display) < 0) { result = 1; break; }
        } else wl_display_cancel_read(display);
        if ((fds[0].revents | fds[1].revents) & (POLLERR | POLLHUP | POLLNVAL)) { result = 1; break; }
        if (wl_display_dispatch_pending(display) < 0) { result = 1; break; }
        if (!running) break; /* 窗口关闭事件不发送校准START命令。 */
        output_reselect(top, surface);
        odom_runtime_imu_snapshot(&model);
        if(fds[3].revents & (POLLERR | POLLHUP | POLLNVAL)) { result=1;break; }
        if(fds[3].revents & POLLIN) {
            if(robot_udp_drain(&robot_udp,&robot)<0) { perror("Robot UDP receive");result=1;break; }
        }
        uint64_t now = imu_now_ms();
        if (dashboard_heading_update(&heading, &model, now))
            fprintf(stderr, "HEADING reference_yaw_deg=%+.4f; initial direction=+X, positive=CCW\n",
                    heading.reference_degrees);
        dashboard_rate_update(&rate, model.good_count, now);
        if (detections_path && now - detection_poll_ms >= 100) {
            (void)dashboard_detections_read(detections_path, &detections);
            detection_poll_ms = now;
        }
        if(yolo_fps_path && now-yolo_fps_poll_ms>=100) {
            (void)dashboard_yolo_fps_read(yolo_fps_path,&yolo_fps);
            yolo_fps_poll_ms=now;
        }
        if (video_generation_sync(&video, 1, now)) { result = 1; break; }
        if (video.fd >= 0 && (fds[2].revents & (POLLERR | POLLHUP | POLLNVAL))) {
            fprintf(stderr, "Camera writer unavailable; waiting briefly for its batch supervisor.\n");
            dashboard_video_close(&video); video_error_ms = now;
        }
        if (video.fd >= 0 && now - video_poll_ms >= 33) {
            video_poll_ms = now;
            if (dashboard_video_update(&video, now) < 0) {
                perror("Camera preview receive"); dashboard_video_close(&video); video_error_ms = now;
            }
        }
        if (video.fd >= 0 && !video.frames && now - video_opened_ms > 10000) {
            fprintf(stderr, "Camera produced no usable frame within 10 seconds; inspect isp/yolo logs.\n"); result = 1; break;
        }
        if (now - last_log >= 5000) {
            fprintf(stderr, "DASHBOARD camera_frames=%" PRIu64 " camera_age_ms=%" PRIu64 " imu_good=%" PRIu64 "\n",
                video.frames, video.frames ? now-video.last_frame_ms : now-window_start, model.good_count);
            fprintf(stderr,"ROBOT UDP5008 accepted=%" PRIu64 " rejected=%" PRIu64
                    " ignored_peer=%" PRIu64 " mode=%s ERR=%u%u%u\n",
                    robot.received,robot.rejected,robot_udp.ignored,
                    robot_drive_mode(&robot,now)==ROBOT_MODE_UNKNOWN?"WAIT":robot.auto_mode?"AUTO":"MANUAL",
                    (unsigned)robot.error[0],(unsigned)robot.error[1],(unsigned)robot.error[2]);
            last_log = now;
        }
        phone_export_state(&model, &robot, now);
        if (configured && now - last_draw >= 33) {
            for (unsigned i = 0; i < 2; ++i) if (!buffers[i].busy) {
                dashboard_canvas_draw(buffers[i].pixels, &model, video.pixels, video.frames, video.last_frame_ms, imu_now_ms(),
                    &rate, detections_path ? &detections : NULL, &heading, &robot,
                    yolo_fps_path ? &yolo_fps : NULL);
                struct odom_state odometry;odom_runtime_snapshot(&odometry);
                dashboard_odometry_draw(buffers[i].pixels,&odometry,imu_now_ms());
                buffers[i].busy = 1;
                wl_surface_attach(surface, buffers[i].wl, 0, 0);
                wl_surface_damage(surface, 0, 0, INT_MAX, INT_MAX);
                wl_surface_commit(surface);
                last_draw = now; break;
            }
        }
    }
cleanup:
    odom_runtime_stop();
    if (robot_udp.fd >= 0) close(robot_udp.fd);
    dashboard_video_close(&video);

    for (unsigned i = 0; i < 2; ++i) {
        if (buffers[i].wl) wl_buffer_destroy(buffers[i].wl);
        if (buffers[i].pixels) munmap(buffers[i].pixels, (size_t)DASHBOARD_WIDTH * DASHBOARD_HEIGHT * 4);
    }
    if (top) xdg_toplevel_destroy(top);
    if (xdg_surface) xdg_surface_destroy(xdg_surface);
    if (viewport) wp_viewport_destroy(viewport);
    if (surface) wl_surface_destroy(surface);
    panel_outputs_finish(&output_state);
    if (viewporter) wp_viewporter_destroy(viewporter);
    if (shell) xdg_wm_base_destroy(shell);
    if (shared_memory) wl_shm_destroy(shared_memory);
    if (compositor) wl_compositor_destroy(compositor);
    wl_registry_destroy(registry);
    wl_display_disconnect(display);
    return result;
}
