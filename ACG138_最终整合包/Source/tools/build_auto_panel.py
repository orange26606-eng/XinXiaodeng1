"""构建现有相机面板，仅补充AUTO状态文字。"""
from pathlib import Path
import hashlib,json,os,re,shutil,subprocess
from dev_paths import build_path,zig_path
R=Path(__file__).resolve().parents[1];B=build_path('final-auto-camera-panel')
for folder in ['board','rk/src','rk/include','rk/third_party']:shutil.copytree(R/folder,B/folder,dirs_exist_ok=True)
shutil.copy2(R/'rk/libwayland-client.so',B/'rk/libwayland-client.so')
board=B/'board';app=B/'rk/src/app';way=B/'rk/third_party/wayland'
reference=['camera_imu_panel.c','panel_outputs.c','panel_output_policy.c','robot_protocol.c','robot_udp.c','dashboard_canvas.c','dashboard_yolo_fps.c','dashboard_heading.c','dashboard_video.c','dashboard_rate.c','dashboard_detections.c','imu_udp.c','imu_protocol.c','imu_iic_control.c','imu_control_raw.c','imu_auto_start.c']
svc=['lab_real_mpusvc.c','lab_real_mpusvc_file.c','lab_real_mpusvc_posix.c','lab_real_mpusvc_panel_client.c','lab_real_mpusvc_vendor_odom.c','lab_real_mpusvc_vendor_fusion.c','lab_real_mpusvc_vendor_adapter.c']
sources=[board/'reference'/n for n in reference]+[board/'phone_export.c']+[app/n for n in svc]+[B/'rk/src/common/lab_types.c',B/'rk/src/common/lab_codec.c']+[way/n for n in ['xdg-shell-protocol.c','viewporter-protocol.c','xdg-output-protocol.c']]
glyphs=(board/'reference/imu_font_data.h').read_text(encoding='utf8');start=glyphs.index('static const struct imu_glyph imu_glyphs_3[]');end=glyphs.index('};',start)
codes={int(x,0)for x in re.findall(r'\{\s*(0x[0-9a-fA-F]+|\d+)\s*,',glyphs[start:end])}
for text in ['自动模式：FPGA · 测距与方向','等待测距','正常前进','左弧线','右弧线','左转','右转','恢复前进','后退','等待方向','近障停车','恢复方向','等待编码器']:assert all(ord(c)in codes for c in text),text
inc=[x for d in [board/'reference',board,way,B/'rk/include',app]for x in ['-I',d]]
inputs={p.relative_to(B).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()for p in sources}
env=dict(os.environ,TMP=str(B),TEMP=str(B),ZIG_GLOBAL_CACHE_DIR=str(B/'global'),ZIG_LOCAL_CACHE_DIR=str(B/'cache'))
exe=B/'camera-imu-panel-phone';cmd=[zig_path(),'cc','-target','aarch64-linux-gnu.2.28','-std=c11','-O2','-UNDEBUG','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-missing-field-initializers',*inc,*sources,'-L',B/'rk','-lwayland-client','-pthread','-lm','-ldl','-o',exe]
p=subprocess.run(list(map(str,cmd)),cwd=B,env=env,capture_output=True,text=True,encoding='utf8',errors='replace');assert p.returncode==0,p.stdout+p.stderr
out=R/'build/camera-imu-panel-phone';shutil.copy2(exe,out)
for rel,digest in inputs.items():assert hashlib.sha256((R/rel).read_bytes()).hexdigest()==digest,rel
(R/'validation/auto-camera-panel-build.json').write_text(json.dumps(dict(status='PASS',target='aarch64-linux-gnu.2.28',sources=inputs,artifact='build/camera-imu-panel-phone',sha256=hashlib.sha256(out.read_bytes()).hexdigest(),AUTO_text_only=True,font_coverage_verified=True,hardware_tested=False),indent=2),encoding='utf8')
print('PASS ARM64 camera panel AUTO text; existing font and capture code retained')
