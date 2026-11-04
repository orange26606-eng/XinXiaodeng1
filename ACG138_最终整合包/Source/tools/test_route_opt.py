"""检查布线优化相关模块的等价性。"""
from pathlib import Path
import hashlib,json,os,shutil,subprocess
from dev_paths import build_path
ROOT=Path(__file__).resolve().parents[1]
STAGE=build_path("route-optimization-tests")
for folder in ["fpga/src", "fpga/src/motion", "fpga/src/ultrasonic", "tests/route_opt"]:
    for p in (ROOT/folder).glob("*.v"):
        shutil.copy2(p,STAGE/p.name)
shutil.copy2(ROOT/"fpga/src/raw/controller.v",STAGE/"controller.v")
shutil.copy2(ROOT/"fpga/src/imu/imu_bundle_cdc.v",STAGE/"imu_bundle_cdc.v")
shutil.copy2(ROOT/"fpga/src/r6_motion_policy.vh",STAGE/"r6_motion_policy.vh")
(STAGE/"src/motion").mkdir(parents=True,exist_ok=True)
for p in (ROOT/"fpga/src/motion").glob("status_*.hex"):
    shutil.copy2(p,STAGE/"src/motion"/p.name)
for p in (ROOT/"tests/route_opt/fixtures").glob("*.hex"):
    shutil.copy2(p,STAGE/p.name)
for p in (ROOT/"tests/route_opt/fixtures").glob("*.v"):
    s=p.read_text(encoding="utf8")
    name=p.stem
    original={"robot_status_before":"robot_status","nav_command_rx_before":"nav_command_rx_gmii","phone_command_rx_before":"phone_command_rx_gmii","controller_before":"controller","robot_motion_before":"robot_motion","robot_udp_tx_before":"robot_udp_message_tx_gmii"}[name]
    s=s.replace("module "+original,"module "+name,1)
    if original=="robot_motion":
        s=s.replace("module robot_wheel #(","module robot_wheel_before #(").replace("robot_wheel #(","robot_wheel_before #(")
    (STAGE/p.name).write_text(s,encoding="utf8")

suites=[
 ("tb_route_udp_tx",["robot_udp_message_tx_gmii.v","robot_udp_tx_before.v","tb_route_udp_tx.v"]),
 ("tb_route_status_link",["nav_control_link.v","nav_command_rx_gmii.v","imu_bundle_cdc.v","nav_velocity_guard.v","nav_status_payload.v","tb_route_status_link.v"]),
 ("tb_route_age",["tb_route_age.v"]),
 ("tb_route_motion",["robot_motion.v","robot_motion_before.v","robot_auto_sonar.v","robot_feedback_guard.v","tb_route_motion.v"]),
 ("tb_route_controller",["controller.v","controller_before.v","tb_route_controller.v"]),
 ("tb_route_receivers",["nav_command_rx_gmii.v","nav_command_rx_before.v","phone_command_rx_gmii.v","phone_command_rx_before.v","tb_route_receivers.v"]),
]
for feedback,near in [(0,0),(0,1),(1,0),(1,1)]:
    suites.append((f"tb_route_status_{feedback}{near}",["robot_status.v","robot_status_before.v","XR_bin32_bcd.v","uart_tx.v","tb_route_status.v"]))
if os.environ.get("ROUTE_TEST_ONLY"):
    suites=[s for s in suites if s[0]==os.environ["ROUTE_TEST_ONLY"]]
tools=Path(os.environ.get("VIVADO_BIN","D:/tools/vivado1/Vivado/2017.3/bin"))
env=dict(os.environ);env.setdefault("PROCESSOR_ARCHITECTURE","AMD64")
results=[]
for top,files in suites:
    if not (STAGE/files[-1]).exists():
        continue
    commands=[
      ["cmd.exe","/d","/c",str(tools/"xvlog.bat"),"--sv",*files],
      ["cmd.exe","/d","/c",str(tools/"xelab.bat"),top,"-s",top,"--timescale","1ns/1ps","--mt","off"],
      ["cmd.exe","/d","/c",str(tools/"xsim.bat"),top,"-R"],
    ]
    logs=[]
    for command in commands:
        run=subprocess.run(command,cwd=STAGE,env=env,capture_output=True,text=True,encoding="utf8",errors="replace",timeout=240)
        logs.append(run.stdout+run.stderr)
        (ROOT/"validation"/(top+".log")).write_text("\n".join(logs),encoding="utf8")
        if run.returncode:
            raise RuntimeError(top+"\n"+logs[-1][-6000:])
    assert "PASS" in logs[-1] and "FAIL" not in logs[-1],logs[-1]
    summary=[x for x in logs[-1].splitlines() if "PASS" in x]
    print(top,*summary,flush=True)
    results.append(dict(top=top,status="PASS",summary=summary))
proof=dict(status="PASS",tests=results,hardware_started=False,motor_commands_sent=False,
    sources={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
             for folder in ["fpga", "tests/route_opt"] for p in (ROOT/folder).rglob("*") if p.is_file()})
name="route-rtl"+("-"+os.environ["ROUTE_TEST_ONLY"] if os.environ.get("ROUTE_TEST_ONLY") else "")+".json"
(ROOT/"validation"/name).write_text(json.dumps(proof,ensure_ascii=False,indent=2),encoding="utf8")
