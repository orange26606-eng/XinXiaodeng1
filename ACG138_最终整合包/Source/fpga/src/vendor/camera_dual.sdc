# 时钟及CDC约束，时间单位为ns。
# 板外同步接口延迟需按实际器件和走线设置。

create_clock -name clk50m -period 20.000 [get_ports {clk50m}]
create_clock -name cam1_pclk -period 10.000 [get_ports {cam1_pclk}]
create_clock -name cam2_pclk -period 10.000 [get_ports {cam2_pclk}]
create_clock -name rgmii_rx_clk -period 8.000 [get_ports {rgmii_rx_clk}]
set_max_delay -from [get_pins {core/controller_top/cache1/dc_fifo/fifo_inst/Equal.wptr_*_s0/Q}] -to [get_pins {core/controller_top/cache1/dc_fifo/fifo_inst/Equal.rq1_wptr_*/DI*}] 8
set_false_path -hold -from [get_pins {core/controller_top/cache1/dc_fifo/fifo_inst/Equal.wptr_*_s0/Q}] -to [get_pins {core/controller_top/cache1/dc_fifo/fifo_inst/Equal.rq1_wptr_*/DI*}]
set_false_path -from [get_pins {diag/u_cam1_snapshot/request_toggle_s2/Q}] -to [get_pins {diag/u_cam1_snapshot/request_meta_s0/D}]
set_false_path -from [get_pins {diag/u_cam1_snapshot/acknowledge_toggle_s0/Q}] -to [get_pins {diag/u_cam1_snapshot/acknowledge_meta_s0/D}]
set_max_delay -from [get_pins {diag/u_cam1_snapshot/source_hold_*_s0/Q}] -to [get_pins {diag/u_cam1_snapshot/dst_data_*_s0/D}] 20
set_false_path -hold -from [get_pins {diag/u_cam1_snapshot/source_hold_*_s0/Q}] -to [get_pins {diag/u_cam1_snapshot/dst_data_*_s0/D}]
set_max_delay -from [get_pins {core/controller_top/cache2/dc_fifo/fifo_inst/Equal.wptr_*_s0/Q}] -to [get_pins {core/controller_top/cache2/dc_fifo/fifo_inst/Equal.rq1_wptr_*/DI*}] 8
set_false_path -hold -from [get_pins {core/controller_top/cache2/dc_fifo/fifo_inst/Equal.wptr_*_s0/Q}] -to [get_pins {core/controller_top/cache2/dc_fifo/fifo_inst/Equal.rq1_wptr_*/DI*}]
set_false_path -from [get_pins {diag/u_cam2_snapshot/request_toggle_s2/Q}] -to [get_pins {diag/u_cam2_snapshot/request_meta_s0/D}]
set_false_path -from [get_pins {diag/u_cam2_snapshot/acknowledge_toggle_s0/Q}] -to [get_pins {diag/u_cam2_snapshot/acknowledge_meta_s0/D}]
set_max_delay -from [get_pins {diag/u_cam2_snapshot/source_hold_*_s0/Q}] -to [get_pins {diag/u_cam2_snapshot/dst_data_*_s0/D}] 20
set_false_path -hold -from [get_pins {diag/u_cam2_snapshot/source_hold_*_s0/Q}] -to [get_pins {diag/u_cam2_snapshot/dst_data_*_s0/D}]
set_false_path -from [get_pins {core/camera_init1/Init_Done_s1/Q core/camera_init2/Init_Done_s1/Q}] -to [get_pins {core/camera_ready_sync_0_s0/D core/Camera_ETH_Formator1/init_done_sync_0_s0/D core/Camera_ETH_Formator2/init_done_sync_0_s0/D}]
set_false_path -from [get_pins {core/controller_top/controller/pair_fault_s0/Q}] -to [get_pins {diag/init_meta_2_s0/D}]
set_false_path -to [get_pins {core/cfg_reset_sync_*/CLEAR core/tx_reset_sync_*/CLEAR core/controller_top/controller_reset_sync_*/CLEAR core/Camera_ETH_Formator1/pclk_reset_sync_*/CLEAR core/Camera_ETH_Formator2/pclk_reset_sync_*/CLEAR diag/reset1_sync_*/CLEAR diag/reset2_sync_*/CLEAR diag/u_cam1_snapshot/dst_reset_sync_*/CLEAR}]
set_false_path -to [get_pins {core/controller_top/cache1/dc_fifo/fifo_inst/reset_r_0_s0/PRESET core/controller_top/cache1/dc_fifo/fifo_inst/reset_r_1_s0/PRESET core/controller_top/cache2/dc_fifo/fifo_inst/reset_r_0_s0/PRESET core/controller_top/cache2/dc_fifo/fifo_inst/reset_r_1_s0/PRESET}]
set_false_path -to [get_pins {phone_command_cdc/src_reset_sync_0_s0/CLEAR phone_command_cdc/src_reset_sync_1_s0/CLEAR}]
set_max_delay -from [get_pins {core/controller_top/cache1/dc_fifo/fifo_inst/Equal.rptr_*_s0/Q}] -to [get_pins {core/controller_top/cache1/dc_fifo/fifo_inst/Equal.wq1_rptr_*_s0/D}] 8
set_false_path -hold -from [get_pins {core/controller_top/cache1/dc_fifo/fifo_inst/Equal.rptr_*_s0/Q}] -to [get_pins {core/controller_top/cache1/dc_fifo/fifo_inst/Equal.wq1_rptr_*_s0/D}]
set_max_delay -from [get_pins {core/controller_top/cache2/dc_fifo/fifo_inst/Equal.rptr_*_s0/Q}] -to [get_pins {core/controller_top/cache2/dc_fifo/fifo_inst/Equal.wq1_rptr_*_s0/D}] 8
set_false_path -hold -from [get_pins {core/controller_top/cache2/dc_fifo/fifo_inst/Equal.rptr_*_s0/Q}] -to [get_pins {core/controller_top/cache2/dc_fifo/fifo_inst/Equal.wq1_rptr_*_s0/D}]
set_max_delay -from [get_pins {core/controller_top/cache1/dc_fifo/fifo_inst/rbin_num_13_s0/Q}] -to [get_pins {core/controller_top/cache1/dc_fifo/fifo_inst/Equal.wq1_rptr_13_s0/D}] 8
set_false_path -hold -from [get_pins {core/controller_top/cache1/dc_fifo/fifo_inst/rbin_num_13_s0/Q}] -to [get_pins {core/controller_top/cache1/dc_fifo/fifo_inst/Equal.wq1_rptr_13_s0/D}]
set_false_path -from [get_pins {core/u_robot_mailbox/request_toggle_s*/Q}] -to [get_pins {core/u_robot_mailbox/request_meta_s0/D}]
set_false_path -from [get_pins {core/u_robot_mailbox/acknowledge_toggle_s0/Q}] -to [get_pins {core/u_robot_mailbox/acknowledge_meta_s0/D}]
set_max_delay -from [get_pins {core/u_robot_mailbox/source_hold_*_s0/Q}] -to [get_pins {core/u_robot_mailbox/dst_data_*_s0/D}] 8
set_false_path -hold -from [get_pins {core/u_robot_mailbox/source_hold_*_s0/Q}] -to [get_pins {core/u_robot_mailbox/dst_data_*_s0/D}]
set_false_path -to [get_pins {phone_command_cdc/dst_reset_sync_*/CLEAR}]
set_false_path -from [get_pins {phone_command_cdc/request_toggle_s2/Q}] -to [get_pins {phone_command_cdc/request_meta_s0/D}]
set_false_path -from [get_pins {phone_command_cdc/acknowledge_toggle_s0/Q}] -to [get_pins {phone_command_cdc/acknowledge_meta_s0/D}]
set_max_delay -from [get_pins {phone_command_cdc/source_hold_*_s0/Q}] -to [get_pins {phone_command_cdc/dst_data_*_s0/D}] 20
set_false_path -hold -from [get_pins {phone_command_cdc/source_hold_*_s0/Q}] -to [get_pins {phone_command_cdc/dst_data_*_s0/D}]
set_false_path -from [get_pins {core/u_mpu9250/mail/request_toggle_s*/Q}] -to [get_pins {core/u_mpu9250/mail/request_meta_s0/D}]
set_false_path -from [get_pins {core/u_mpu9250/mail/acknowledge_toggle_s*/Q}] -to [get_pins {core/u_mpu9250/mail/acknowledge_meta_s0/D}]
set_max_delay -from [get_pins {core/u_mpu9250/mail/source_hold_*_s0/Q}] -to [get_pins {core/u_mpu9250/mail/dst_data_*_s0/D}] 8
set_false_path -hold -from [get_pins {core/u_mpu9250/mail/source_hold_*_s0/Q}] -to [get_pins {core/u_mpu9250/mail/dst_data_*_s0/D}]
set_false_path -from [get_pins {navigation_link/crossing/request_toggle_s*/Q}] -to [get_pins {navigation_link/crossing/request_meta_s0/D}]
set_false_path -from [get_pins {navigation_link/crossing/acknowledge_toggle_s*/Q}] -to [get_pins {navigation_link/crossing/acknowledge_meta_s0/D}]
set_max_delay -from [get_pins {navigation_link/crossing/source_hold_*_s0/Q}] -to [get_pins {navigation_link/crossing/dst_data_*_s0/D}] 20
set_false_path -hold -from [get_pins {navigation_link/crossing/source_hold_*_s0/Q}] -to [get_pins {navigation_link/crossing/dst_data_*_s0/D}]
set_false_path -from [get_pins {core/nav_mail/request_toggle_s*/Q}] -to [get_pins {core/nav_mail/request_meta_s0/D}]
set_false_path -from [get_pins {core/nav_mail/acknowledge_toggle_s*/Q}] -to [get_pins {core/nav_mail/acknowledge_meta_s0/D}]
set_max_delay -from [get_pins {core/nav_mail/source_hold_*_s0/Q}] -to [get_pins {core/nav_mail/dst_data_*_s0/D}] 8
set_false_path -hold -from [get_pins {core/nav_mail/source_hold_*_s0/Q}] -to [get_pins {core/nav_mail/dst_data_*_s0/D}]
set_false_path -from [get_pins {core/u_mpu9250/attitude/mail/request_toggle_s*/Q}] -to [get_pins {core/u_mpu9250/attitude/mail/request_meta_s0/D}]
set_false_path -from [get_pins {core/u_mpu9250/attitude/mail/acknowledge_toggle_s*/Q}] -to [get_pins {core/u_mpu9250/attitude/mail/acknowledge_meta_s0/D}]
set_max_delay -from [get_pins {core/u_mpu9250/attitude/mail/source_hold_*_s0/Q}] -to [get_pins {core/u_mpu9250/attitude/mail/dst_data_*_s0/D}] 8
set_false_path -hold -from [get_pins {core/u_mpu9250/attitude/mail/source_hold_*_s0/Q}] -to [get_pins {core/u_mpu9250/attitude/mail/dst_data_*_s0/D}]
