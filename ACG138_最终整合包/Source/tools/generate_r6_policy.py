"""单一R6距离/指纹策略生成，C和Verilog必须一致。"""
from pathlib import Path
import json
import sys
r=Path(__file__).resolve().parents[1]
d=json.loads((r/'policy/r6-motion.json').read_text(encoding='utf8'))
assert 0<d['front_emergency_mm']<d['front_forward_stop_mm']<d['front_near_mm']<d['front_observe_mm']
c=['/* Generated from policy/r6-motion.json. Do not hand edit. */','#ifndef ACG_R6_MOTION_POLICY_H','#define ACG_R6_MOTION_POLICY_H','#include <stdint.h>',f'#define R6_CONFIG_ID UINT32_C(0x{d["config_id_hex"]})']
vh=['// Generated from policy/r6-motion.json. Do not hand edit.','`ifndef ACG_R6_MOTION_POLICY_VH','`define ACG_R6_MOTION_POLICY_VH',f'`define R6_CONFIG_ID 32\'h{d["config_id_hex"]}']
c.append(f'#define R6_COMMAND_TTL_MS {d["command_ttl_ms"]}u')
vh.append(f'`define R6_COMMAND_TTL_MS {d["command_ttl_ms"]}')
for k in ['software_policy','front_observe_mm','front_near_mm','front_forward_stop_mm','front_emergency_mm','front_release_mm','front_release_samples','side_turn_mm','side_straight_emergency_mm','body_buffer_mm','reverse_max_ms','action_geometry_budget_us','cycle_geometry_budget_us']:
 c.append(f'#define R6_{k.upper()} {d[k]}u')
 if k.startswith('front_') or k=='side_turn_mm':vh.append(f'`define R6_{k.upper()} {d[k]}')
for k in ['max_linear_m_s','reverse_speed_m_s','reverse_target_m','reverse_hard_m','reverse_min_fit_m','brake_m_s2']:
 c.append(f'#define R6_{k.upper()} {d[k]}')
c.append('#define R6_SPEED_BAND_COUNT '+str(len(d['speed_bands']))+'u')
c.append('#define R6_SPEED_BANDS {'+','.join('{'+str(mm)+','+str(speed)+'}' for mm,speed in d['speed_bands'])+'}')
c+=['enum r6_check_result {R6_CHECK_CLEAR=1,R6_CHECK_OCCUPIED=0,R6_CHECK_UNOBSERVED=-1,R6_CHECK_STALE=-2,R6_CHECK_POSE_INVALID=-3,R6_CHECK_BUDGET=-4,R6_CHECK_WIRE_REJECT=-5,R6_CHECK_INVALID=-6};','#endif']
vh.append('`endif')
(r/'rk/include/r6_motion_policy.h').write_text('\n'.join(c)+'\n',encoding='utf8')
if '--c-only' not in sys.argv:
 (r/'fpga/src/r6_motion_policy.vh').write_text('\n'.join(vh)+'\n',encoding='utf8')
print('Generated R6 C/Verilog policy, config=0x'+d['config_id_hex'])
