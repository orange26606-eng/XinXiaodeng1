"""无设备运行测距状态机及运动隔离RTL测试。"""
from pathlib import Path
import hashlib,json,os,shutil,subprocess
from dev_paths import build_path

R=Path(__file__).resolve().parents[1]
B=build_path('final-auto-rtl')
V=R/'validation';V.mkdir(exist_ok=True)
IVERILOG=os.environ.get('IVERILOG','D:/tools/IcarusVerilog/iverilog/bin/iverilog.exe')
VVP=os.environ.get('VVP','D:/tools/IcarusVerilog/iverilog/bin/vvp.exe')
for folder in ['fpga/src','fpga/src/motion','fpga/src/ultrasonic','tests/auto_sonar']:
    for p in (R/folder).glob('*.v'):shutil.copy2(p,B/p.name)
shutil.copy2(R/'fpga/src/r6_motion_policy.vh',B/'r6_motion_policy.vh')
(B/'src/motion').mkdir(parents=True,exist_ok=True)
for p in (R/'fpga/src/motion').glob('status_*.hex'):shutil.copy2(p,B/'src/motion'/p.name)
base=R/'tests/auto_sonar/fixtures/robot_motion_baseline_original.v'
s=base.read_text(encoding='utf8').replace('module robot_motion #(','module robot_motion_baseline #(').replace('module robot_wheel #(','module robot_wheel_baseline #(').replace('robot_wheel #(','robot_wheel_baseline #(')
(B/'robot_motion_baseline.v').write_text(s,encoding='utf8')
suite=[(name,['robot_auto_sonar.v',source])for name,source in [
 ('tb_nearstop_fsm','tb_nearstop_fsm.v'),('tb_nearstop_context','tb_nearstop_fsm.v'),
 ('tb_auto_channel','tb_auto_channel.v'),('tb_auto_history','tb_auto_history.v'),
 ('tb_auto_directional','tb_auto_directional.v'),('tb_turnfirst_wrap','tb_turnfirst_wrap.v'),
 ('tb_turnfirst_fsm','tb_turnfirst_fsm.v')]]
suite += [('tb_nearstop_system',['robot_auto_sonar.v','robot_motion.v','robot_feedback_guard.v','tb_nearstop_system.v'])]
if(B/'tb_merge_isolation.v').exists():suite += [('tb_merge_isolation',['robot_auto_sonar.v','robot_motion.v','robot_motion_baseline.v','robot_feedback_guard.v','tb_merge_isolation.v'])]
if(B/'tb_merge_uart.v').exists():suite += [('tb_merge_uart',[p.name for folder in ['fpga/src/motion','fpga/src/ultrasonic']for p in(R/folder).glob('*.v')]+['nav_rpm_feedback.v','nav_pi_pipeline.v','tb_merge_uart.v'])]
suite += [('tb_merge_remote',['phone_remote_control.v','tb_merge_remote.v'])]
if os.environ.get('AUTO_TEST_ONLY'):suite=[s for s in suite if s[0]==os.environ['AUTO_TEST_ONLY']]
T=Path(os.environ.get('VIVADO_BIN','D:/tools/vivado1/Vivado/2017.3/bin'))
env=os.environ.copy();env.setdefault('PROCESSOR_ARCHITECTURE','AMD64')
results=[]
for top,files in suite:
    output=B/(top+'.vvp');logs=[]
    commands=[['cmd.exe','/d','/c',str(T/'xvlog.bat'),'--sv',*files],
              ['cmd.exe','/d','/c',str(T/'xelab.bat'),top,'-s',top,'--timescale','1ns/1ps','--mt','off'],
              ['cmd.exe','/d','/c',str(T/'xsim.bat'),top,'-R']]
    for cmd in commands:
        p=subprocess.run(cmd,cwd=B,env=env,capture_output=True,text=True,encoding='utf8',errors='replace',timeout=180)
        logs.append(p.stdout+p.stderr)
        if p.returncode:
            (V/(top+'.log')).write_text('\n'.join(logs),encoding='utf8')
            raise RuntimeError(top+'\n'+logs[-1][-5000:])
    assert 'PASS' in logs[-1] and 'FAIL' not in logs[-1],(top,logs[-1])
    (V/(top+'.log')).write_text('\n'.join(logs),encoding='utf8')
    lines=[x for x in logs[-1].splitlines()if'PASS'in x];print(top,*lines,flush=True)
    results.append(dict(top=top,result='PASS',summary=lines))
proof=dict(status='PASS',tests=results,motor_commands_sent=False,hardware_tested=False,
 sources={p.relative_to(R).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for folder in ['fpga/src/motion','fpga/src/ultrasonic','tests/auto_sonar']for p in(R/folder).rglob('*.v')})
name='auto-rtl-'+os.environ['AUTO_TEST_ONLY']+'.json' if os.environ.get('AUTO_TEST_ONLY') else 'auto-rtl.json'
(V/name).write_text(json.dumps(proof,ensure_ascii=False,indent=2),encoding='utf8')
