set task_dir [file dirname [file normalize [info script]]]
cd $task_dir
open_project phone_robot.gprj
set_option -include_path [list [file join $task_dir src]]
set_option -enable_dsrm 1
set_option -use_cpu_as_gpio 1
set_option -synthesis_tool gowinsynthesis
set_option -timing_driven 1
set_option -place_option 4
set_option -route_option 2
set_option -top_module camera_dual
set_option -output_base_name phone_robot_final_auto_nearstop
run all
