`include "r6_motion_policy.vh"

// 增加认证代理电机命令通路和独立500ms看门狗。
//-------------------------------


// 日期：2026-10-12 18:26
// 模块：camera_dual
// 功能: ACG138双目与IMU顶层接口及同域UART诊断。
// 版本：1.2.1
// 版本1.1.0：引出IMU的P10 I2C接口与四个固定3.3V状态灯。
// 版本1.2.0：增加板内RGMII接收端口及IIC校准命令通路。
//                   v1.2.1----UART诊断改用与AF状态相同的PLL50MHz域，消除直接跨时钟域状态采样。
//-------------------------------

module camera_dual(
    input clk50m,
    input [4:0] key_n,
    input [3:0] enc_a,enc_b,
    output [3:0] motor_in1,motor_in2,motor_pwm,
    input rgmii_rx_clk, input rgmii_rx_ctl, input [3:0] rgmii_rxd,
    inout imu_sda, imu_scl,
    output [3:0] imu_led,
    output uart_tx,
    input front_rx,left_rx,right_rx,
    output camera_diag_tx,
    output [7:0] ultrasonic_led,
    output rgmii_tx_clk,
    output [3:0] rgmii_txd,
    output rgmii_tx_ctl,
    output cam1_pwdn,
    output cam1_rst_n,
    output cam1_xclk,
    output cam1_scl,
    inout cam1_sda,
    input cam1_pclk,
    input cam1_href,
    input cam1_vsync,
    input [7:0] cam1_data,
    output cam2_pwdn,
    output cam2_rst_n,
    output cam2_xclk,
    output cam2_scl,
    inout cam2_sda,
    input cam2_pclk,
    input cam2_href,
    input cam2_vsync,
    input [7:0] cam2_data
);
reg [20:0] startup = 0;
always @(posedge clk50m) if (!startup[20]) startup <= startup + 1'b1;
wire reset_n = startup[20];
wire core_reset1, core_reset2;
wire init_done;
wire [15:0] af_status1,af_status2;
wire camera_diag_clk,camera_diag_rst_n;
wire [111:0] robot_snapshot;
wire [127:0] odom_positions;wire [3:0] odom_ready;wire [7:0] odom_ab;wire [31:0] odom_invalid;
wire remote_rx_valid,remote_rx_ready,remote_valid,remote_reset_n;
wire [79:0] remote_rx_data,remote_data;
imu_bundle_cdc #(.WIDTH(80)) phone_command_cdc(
 .src_clk(rgmii_rx_clk),.src_rst_n(remote_reset_n),.src_valid(remote_rx_valid),.src_ready(remote_rx_ready),
 .src_data(remote_rx_data),.dst_clk(clk50m),.dst_rst_n(remote_reset_n),
 .dst_valid(remote_valid),.dst_ready(1'b1),.dst_data(remote_data));
wire imu_calibration_wait;
wire nav_rx_en,nav_rx_er,nav_status_valid,nav_status_ready,nav_claimed,nav_armed;
wire[47:0]nav_raw_sonar;wire[95:0]nav_raw_sequence,nav_raw_age_us;wire[2:0]nav_raw_available,nav_raw_zero;
 wire[11:0]nav_sensor_flags;wire[63:0]nav_effective_duty;wire[7:0]nav_execution_flags;
 wire[1:0]nav_front_sequence;wire[14:0]nav_sonar_diag;wire[2:0]nav_pending;wire[7:0]nav_motion_intent;wire[7:0]nav_rx_data;wire[767:0]nav_status_data;wire[63:0]nav_targets,nav_measured;
wire nav_blocked,nav_ready,nav_local_stop,nav_fault;wire[3:0]nav_fault_mask;wire[1:0]nav_mode;
wire[55:0]nav_diagnostic;
nav_control_link #(.CONFIG_ID(`R6_CONFIG_ID)) navigation_link(
 .rx_clk(rgmii_rx_clk),.clk(clk50m),.rst_n(remote_reset_n),.gmii_rxen(nav_rx_en),.gmii_rxer(nav_rx_er),.gmii_rxd(nav_rx_data),
 .raw_sonar(nav_raw_sonar),.raw_sequence(nav_raw_sequence),.raw_age_us(nav_raw_age_us),.raw_available(nav_raw_available),.raw_zero(nav_raw_zero),
 .effective_duty(nav_effective_duty),.execution_flags(nav_execution_flags),.sonar_fresh(nav_sensor_flags[2:0]),.sonar_near(nav_sensor_flags[5:3]),.sonar_pending(nav_sensor_flags[8:6]),.sonar_observation_block(nav_sensor_flags[11:9]),
 .front_nonzero_sequence(nav_front_sequence),.sonar_age_diag(nav_sonar_diag),.observation_pending(nav_pending),.motion_blocked(nav_blocked),.ready(nav_ready),.local_stop(nav_local_stop),.fault(nav_fault),.measured_rpm_centi(nav_measured),
 .encoder_counts(odom_positions),.encoder_valid(odom_ready),.fault_mask(nav_fault_mask),.diagnostic(nav_diagnostic),
 .claimed(nav_claimed),.armed(nav_armed),.targets(nav_targets),.motion_intent(nav_motion_intent),.owner_mode(nav_mode),
 .status_valid(nav_status_valid),.status_ready(nav_status_ready),.status_payload(nav_status_data));
wire [7:0] robot_status_led;
wire [2:0] init_status;
assign cam1_rst_n = reset_n & core_reset1;
assign cam2_rst_n = reset_n & core_reset2;
ov5640x2_udp_rgmii core (
.nav_rx_en(nav_rx_en),.nav_rx_er(nav_rx_er),.nav_rx_data(nav_rx_data),.nav_status_valid(nav_status_valid),.nav_status_ready(nav_status_ready),.nav_status_data(nav_status_data),
.remote_valid(remote_rx_valid),.remote_ready(remote_rx_ready),.remote_data(remote_rx_data),.remote_reset_n(remote_reset_n),
.robot_snapshot(robot_snapshot),.odom_ab(odom_ab),.odom_invalid(odom_invalid),.odom_positions(odom_positions),.odom_ready(odom_ready),.odom_reset_key(key_n[4]),
.imu_calibration_wait(imu_calibration_wait),
.rgmii_rx_clk(rgmii_rx_clk),.rgmii_rx_ctl(rgmii_rx_ctl),.rgmii_rxd(rgmii_rxd),
.imu_sda(imu_sda),.imu_scl(imu_scl),.imu_led(imu_led),
.diag_clk(camera_diag_clk),.diag_reset_n(camera_diag_rst_n),
.af_status1(af_status1),.af_status2(af_status2),
.camera_status(init_status),.clk(clk50m),.rst_n(reset_n),.eth_gtxc(rgmii_tx_clk),.eth_txd(rgmii_txd),.eth_txen(rgmii_tx_ctl),
.eth_mdc(),.eth_mdio(),.eth_rst_n(),.led(init_done),
.camera1_sclk(cam1_scl),.camera1_sdat(cam1_sda),.camera1_pclk(cam1_pclk),.camera1_href(cam1_href),.camera1_vsync(cam1_vsync),.camera1_data(cam1_data),.camera1_xclk(cam1_xclk),.camera1_pwdn(cam1_pwdn),.camera1_rst_n(core_reset1),
.camera2_sclk(cam2_scl),.camera2_sdat(cam2_sda),.camera2_pclk(cam2_pclk),.camera2_href(cam2_href),.camera2_vsync(cam2_vsync),.camera2_data(cam2_data),.camera2_xclk(cam2_xclk),.camera2_pwdn(cam2_pwdn),.camera2_rst_n(core_reset2)
);
camera_diag diag(.af1(af_status1),.af2(af_status2),.clk(camera_diag_clk),.reset_n(camera_diag_rst_n),.init_done(init_status),
.pclk1(cam1_pclk),.href1(cam1_href),.vsync1(cam1_vsync),
.pclk2(cam2_pclk),.href2(cam2_href),.vsync2(cam2_vsync),.uart_tx(camera_diag_tx));
robot_control #(.NAV_CONTROL(1)) ultrasonic(.nav_sensor_flags(nav_sensor_flags),.nav_raw_sonar(nav_raw_sonar),.nav_raw_sequence(nav_raw_sequence),.nav_raw_age_us(nav_raw_age_us),.nav_raw_available(nav_raw_available),.nav_raw_zero(nav_raw_zero),.nav_effective_duty(nav_effective_duty),.nav_execution_flags(nav_execution_flags),.nav_front_sequence(nav_front_sequence),.nav_diagnostic(nav_diagnostic),.nav_claimed(nav_claimed),.nav_armed(nav_armed),.nav_rpm(nav_targets),.nav_intent(nav_motion_intent),.nav_sonar_diag(nav_sonar_diag),.nav_pending(nav_pending),.nav_blocked(nav_blocked),.nav_ready(nav_ready),.nav_local_stop(nav_local_stop),.nav_fault(nav_fault),.nav_fault_mask(nav_fault_mask),.nav_measured(nav_measured),.odom_ab(odom_ab),.odom_invalid(odom_invalid),.odom_positions(odom_positions),.odom_ready(odom_ready),.remote_valid(remote_valid),.remote_data(remote_data),.telemetry_data(robot_snapshot),.key_n(key_n),.enc_a(enc_a),.enc_b(enc_b),.motor_in1(motor_in1),.motor_in2(motor_in2),.motor_pwm(motor_pwm),.clk(clk50m),.front_rx(front_rx),.left_rx(left_rx),.right_rx(right_rx),.host_tx(uart_tx),.led(robot_status_led));
imu_wait_leds waiting_leds(.clk(camera_diag_clk),.rst_n(camera_diag_rst_n),
 .waiting(imu_calibration_wait),.normal_led(robot_status_led),.led(ultrasonic_led));
endmodule
