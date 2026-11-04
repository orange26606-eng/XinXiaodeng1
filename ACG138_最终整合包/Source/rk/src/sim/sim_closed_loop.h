#ifndef ACG_SIM_CLOSED_LOOP_DRAFT_H
#define ACG_SIM_CLOSED_LOOP_DRAFT_H
/* 闭环仿真内部接口，与公开 ABI 和报文格式分开。由调用方在单线程中推进，不访问硬件、套接字或电机，也不使用实际时间等待。 */
#include "lab_sim.h"
#include "lab_mapping.h"
#ifdef __cplusplus
extern "C" {
#endif
#define SIM_CL_DRAFT_VERSION 1u
#define SIM_CL_PENDING_MAX 512u
#define SIM_CL_SCAN_BUFFERS 32u
#define SIM_CL_OBSTACLES_MAX 7u

enum sim_cl_scene {
    SIM_CL_ROOM=1, SIM_CL_OBSTACLE=2, SIM_CL_NARROW=3,
    SIM_CL_DEAD_END=4, SIM_CL_UNREACHABLE=5, SIM_CL_CORRIDOR=6
};
typedef struct sim_cl sim_cl;
typedef struct sim_cl_box { double min_x_m,min_y_m,max_x_m,max_y_m; } sim_cl_box;
typedef struct sim_cl_config {
    lab_sim_config sensors; /* 由控制命令驱动运动，忽略预设path、speed及yaw_rate。 */
    uint32_t scene,origin_epoch,command_source_id,integration_step_us;
    double vehicle_width_m,vehicle_length_m;
    double max_linear_m_s,max_angular_rad_s,max_wheel_m_s;
    double acceleration_m_s2,braking_m_s2,angular_acceleration_rad_s2,angular_braking_rad_s2;
    double wheel_response_tau_s,traction_left,traction_right;
    uint64_t max_command_valid_us;
    uint64_t scan_delay_us,encoder_delay_us,imu_delay_us;
    uint32_t drop_encoder_every,drop_imu_every;
    /* 可冻结单个编码器通道而不改真实运动，-1关闭故障注入。 */
    int32_t freeze_encoder;
    uint64_t freeze_start_us,freeze_duration_us;
} sim_cl_config;
typedef struct sim_cl_command {
    lab_context context;
    uint32_t source_id,sequence;
    uint64_t generated_us,valid_until_us;
    double linear_m_s,angular_rad_s;
} sim_cl_command;
typedef struct sim_cl_stats {
    uint64_t now_us,finish_us,commands_accepted,commands_rejected,commands_clamped,command_expirations;
    uint64_t events_generated,events_delivered,scans_generated,scans_dropped,poses_dropped;
    uint64_t encoders_dropped,imus_dropped,collisions;
    uint32_t pending_events,pending_high_water,scan_buffers_high_water;
    double wheel_left_m_s,wheel_right_m_s;
    size_t allocated_bytes;
} sim_cl_stats;
/* 仅用于评估，不送入SLAM或带噪导航。 */
typedef struct sim_cl_evaluation {
    lab_truth truth;
    lab_sim_metrics odometry;
    double minimum_clearance_m;
    uint64_t first_collision_us;
    lab_truth first_collision_pose;
    sim_cl_command first_collision_command;
    int has_collision;
} sim_cl_evaluation;

void sim_cl_config_default(sim_cl_config *config);
lab_status sim_cl_open(sim_cl **out,const sim_cl_config *config);
/* 内部地图世界适配器复制原栅格和几何，仅用于理想模式。未知和地图边界结束射线但不返回回波；碰撞检查未知及占用不小于65的格。起点、位姿、IMU及扫描使用同一导入坐标系。


 */
lab_status sim_cl_open_map(sim_cl **out,const sim_cl_config *config,const lab_map_view *map,
    double start_x_m,double start_y_m,double start_yaw_rad);
/* 只接受当前上下文和数据源的递增序号，生成时刻不晚于now且now早于有效期，生存期不超上限，不排队未来命令。到期瞬间将虚拟目标和轮速清零，普通零命令则按制动配置减速。


 */
lab_status sim_cl_submit(sim_cl *sim,const sim_cl_command *command);
/* 绝对仿真时刻单调递增，步长大于0且不超integration_step_us。继续推进前先排空可见事件，否则返回LAB_AGAIN且不改状态。延迟事件在received_us不晚于now后才可见，运行时长后冻结运动，可继续到finish_us排空尾部。


 */
lab_status sim_cl_step(sim_cl *sim,uint64_t target_us);
/* 输出归调用方持有，AGAIN无可见事件，EOF表示尾部排空。 */
lab_status sim_cl_next(sim_cl *sim,lab_event *out);
void sim_cl_get_stats(const sim_cl *sim,sim_cl_stats *out);
void sim_cl_get_evaluation(const sim_cl *sim,sim_cl_evaluation *out);
/* 场景几何只供评估或明确标识的已知地图NAV_IDEAL_TEST，不作为联合SLAM导航测试地图。
 */
lab_status sim_cl_scene_boxes(const sim_cl *sim,sim_cl_box *boxes,size_t capacity,size_t *required);
void sim_cl_close(sim_cl *sim);
#ifdef __cplusplus
}
#endif
#endif
