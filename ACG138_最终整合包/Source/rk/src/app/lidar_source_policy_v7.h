#ifndef LIDAR_SOURCE_POLICY_V7_H
#define LIDAR_SOURCE_POLICY_V7_H
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <cstdlib>

/* C1采集策略按5kHz、8至12圈每秒规格配置。达到8Hz时标称每圈625点，10Hz时为500点；这是密度与延迟取舍，不改变硬件量程或回波质量。

 */
namespace lidar_source_v7 {
struct Request { double hz; bool valid; bool explicit_value; };
inline Request request(const char *text) {
    if(!text)return {8.,true,false};
    char *end=nullptr;
    double hz=std::strtod(text,&end);
    if(!*text || !end || *end || !std::isfinite(hz) || (hz!=0. && (hz<8. || hz>12.)))
        return {0.,false,true};
    return {hz,true,true};
}
inline bool c1_model(unsigned model) {return (model>>4)==4;}
inline unsigned rpm(double hz) {return unsigned(std::lround(hz*60.));}
struct Tune {
    bool pending=false,attempted=false,sdk_accepted=false;
    unsigned requested_rpm=0;
    Tune(Request r,unsigned model,bool rpm_supported) {
        pending=r.valid && r.hz>0. && c1_model(model) && rpm_supported;
        requested_rpm=pending?rpm(r.hz):0;
    }
    bool take_request(unsigned &out) {
        if(!pending)return false;
        pending=false;attempted=true;out=requested_rpm;return true;
    }
};
struct Timing {
    uint64_t previous_first=0,interval_us=0;
    unsigned intervals=0;
    /* 按独立来源时间计数，重复或倒退时间保留为旧扫描。 */

    bool observe(uint64_t first) {
        if(!first || (previous_first && first<=previous_first))return false;
        if(previous_first) {
            uint64_t delta=first-previous_first;
            if(delta<=500000){interval_us+=delta;++intervals;}
        }
        previous_first=first;return true;
    }
    double hz() const {return interval_us?1e6*double(intervals)/double(interval_us):0.;}
    void reset_window() {interval_us=0;intervals=0;}
};
inline bool duration(uint64_t first,uint64_t returned,size_t count,double sample_us,uint32_t &span) {
    if(!count || count>8192 || !std::isfinite(sample_us) || sample_us<=0.)return false;
    const double d=(count-1)*sample_us;
    if(!first || first>returned || !std::isfinite(d) || d<1. || d>250000. || d>double(returned-first))return false;
    span=uint32_t(std::ceil(d));return true;
}
struct Counts {unsigned valid=0,weak=0,quality_zero=0,range_zero=0,out_of_range=0;};
inline void count(Counts &c,uint32_t distance_q2,unsigned quality) {
    /* quality=0表示无效；质量统计与原始LS01输出分开。 */

    if(!quality){++c.quality_zero;return;}
    if(!distance_q2){++c.range_zero;return;}
    if(distance_q2<200 || distance_q2>48000){++c.out_of_range;return;}
    ++c.valid;if(quality==1)++c.weak;
}
}
#endif
