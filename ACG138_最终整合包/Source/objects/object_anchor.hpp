#pragma once
#include <cmath>
#include <cstdint>
namespace obj {
struct AnchorLimits {double area_min=.45,area_max=2.2,aspect_tol=.40,position_claim_m=.35;};
struct ObjectAnchor {int class_id=-1;double area=0,aspect=0,world_x=0,world_y=0;bool world_valid=false;uint64_t locked_us=0,epoch=0;};
// 几何检查使用已确认的世界坐标或连续里程计坐标。
// 不接受相机光学坐标；此处不运行描述子网络。
inline const char* anchor_geometry(const ObjectAnchor&a,int cls,double area,double aspect,const AnchorLimits&c){
 if(cls!=a.class_id)return "CLASS_MISMATCH";
 if(!std::isfinite(area)||!std::isfinite(aspect)||!std::isfinite(a.area)||!std::isfinite(a.aspect)||area<=0||aspect<=0||a.area<=0||a.aspect<=0)return "INVALID_GEOMETRY";
 // 面积随距离平方变化，初始面积仅用于诊断，
 // 关联逐次检查尺度、运动及不可变外观。
 if(std::fabs(aspect/a.aspect-1)>c.aspect_tol+1e-9)return "ASPECT_OUTSIDE_ANCHOR";
 return nullptr;
}
inline const char* anchor_claim(const ObjectAnchor&a,int cls,double area,double aspect,bool world_valid,double x,double y,const AnchorLimits&c){
 if(const char*r=anchor_geometry(a,cls,area,aspect,c))return r;
 if(!a.world_valid||!world_valid||!std::isfinite(a.world_x)||!std::isfinite(a.world_y)||!std::isfinite(x)||!std::isfinite(y))return "WORLD_POSITION_REQUIRED_FOR_RECLAIM";
 if(std::hypot(x-a.world_x,y-a.world_y)>c.position_claim_m+1e-9)return "WORLD_POSITION_OUTSIDE_ANCHOR";
 return nullptr;
}
}
