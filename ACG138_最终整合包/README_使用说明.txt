FPGA多传感器采集与实时运动控制系统
2026-11-04

FPGA实现双路OV5640采集、MPU9250采集与六轴姿态解算、三路超声波处理、四轮编码器测速、PI闭环调速、PWM输出及自动避障。硬件控制链路配有跨时钟域缓存、以太网分包仲裁、状态回传和命令看门狗。
配套应用提供视觉识别、地图处理、目标任务及手机显示与控制。

阅读顺序
Docs/项目说明.txt介绍硬件功能、资源利用、模块入口和系统分工。
Source/fpga/phone_robot.gprj打开完整FPGA工程，Source/fpga/build.tcl执行构建。
FPGA/phone_robot_final_auto_nearstop.fs为已完成布局布线的配套位流。
Docs/使用说明.txt给出传输、烧录、启动与验证步骤。

交付内容
FPGA：配套位流。
Source/fpga：Verilog源码、引脚、时钟约束和ROM初始化文件。
Source其他目录：协同应用源码、测试与构建工具。
acg138-final-auto-nearstop-20261104：板端运行目录，包含程序、模型、配置和运行库。
Android：配套APK67。
LICENSES：第三方许可与来源记录。

在本目录打开PowerShell，将BoardIP改为板子当前Wi-Fi地址。旧会话先在其目录执行sh begin stop，同名安装目录备份后再上传。
$BoardIP = '板子当前IP'
scp -O -r '.\acg138-final-auto-nearstop-20261104' "root@${BoardIP}:/root/"
ssh "root@$BoardIP"

在板端执行
cd /root/acg138-final-auto-nearstop-20261104
sh fix-permissions.sh
sha256sum -c SHA256SUMS.txt
LIDAR_DEVICE=/dev/ttyUSB0 ROBOT_OUTPUT=HDMI-A-2 sh begin drive

状态、诊断与停止
sh begin status
sh collect-diagnostics.sh
sh begin stop

FPGA在Gowin Programmer中加载配套FS，手机安装Android目录的APK67。文件哈希相同时可继续使用已安装版本。CFG编号相同不足以确认位流相同。
运行文件的校验清单为SHA256SUMS.txt。自动模式入口修复见Docs/自动模式修复说明.txt。

源码注释与第三方文件说明见Docs/源码说明.txt。布局布线和时序报告见Docs/布局布线报告/使用说明.txt。
