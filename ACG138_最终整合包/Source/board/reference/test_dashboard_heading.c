/* 测试航向零点、回绕、过期及故障保留与会话重置；期望向量由基本方位几何独立给出。
 */
#include "dashboard_heading.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
static unsigned checks;
#define CHECK(c) do { ++checks; if(!(c)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c); exit(1); } } while(0)
#define CLOSE(a,b) CHECK(fabs((a)-(b)) < 1e-9)
static void sample(struct imu_model *m, double yaw, uint64_t now)
{
    m->have_angles=m->seen_packet=m->have_sequence=1;
    m->state=IMU_LIVE; m->degrees[2]=yaw;
    m->last_packet_ms=m->last_good_ms=now;
    ++m->good_count; ++m->sequence;
}
static void vector(const struct dashboard_heading *h,double x,double y)
{
    double a=123,b=456;
    CHECK(dashboard_heading_vector(h,&a,&b)); CLOSE(a,x); CLOSE(b,y);
}
int main(void)
{
    struct imu_model m; struct dashboard_heading h;
    imu_model_init(&m,1000); dashboard_heading_init(&h);
    double x=123,y=456;
    CHECK(!dashboard_heading_update(&h,&m,1000)); CHECK(!h.ready);
    CHECK(!dashboard_heading_vector(&h,&x,&y)); CLOSE(x,123); CLOSE(y,456);
    sample(&m,73.5,2000);
    CHECK(!dashboard_heading_update(&h,&m,3000)); CHECK(!h.ready); /* 字段：stale */
    CHECK(!dashboard_heading_update(&h,&m,1999)); CHECK(!h.ready); /* 字段：future */
    m.state=IMU_DEVICE_ERROR;
    CHECK(!dashboard_heading_update(&h,&m,2000)); CHECK(!h.ready);
    sample(&m,NAN,2010); CHECK(!dashboard_heading_update(&h,&m,2010));
    sample(&m,INFINITY,2020); CHECK(!dashboard_heading_update(&h,&m,2020));
    sample(&m,73.5,2030);
    CHECK(dashboard_heading_update(&h,&m,2030)); CLOSE(h.reference_degrees,73.5);
    CLOSE(h.relative_degrees,0); vector(&h,1,0);
    sample(&m,163.5,2050); CHECK(!dashboard_heading_update(&h,&m,2050));
    CLOSE(h.relative_degrees,90); vector(&h,0,1);
    sample(&m,-16.5,2070); dashboard_heading_update(&h,&m,2070);
    CLOSE(h.relative_degrees,-90); vector(&h,0,-1);
    sample(&m,-106.5,2090); dashboard_heading_update(&h,&m,2090);
    CLOSE(h.relative_degrees,-180); vector(&h,-1,0);
    sample(&m,433.5,2110); dashboard_heading_update(&h,&m,2110);
    CLOSE(h.relative_degrees,0); vector(&h,1,0);
    sample(&m,103.5,2130); dashboard_heading_update(&h,&m,2130);
    CLOSE(h.relative_degrees,30);
    for(int state=IMU_DEVICE_ERROR; state<=IMU_STALE; ++state) {
        sample(&m,12,2150); m.state=(enum imu_state)state;
        CHECK(!dashboard_heading_update(&h,&m,2150)); CLOSE(h.relative_degrees,30);
        CLOSE(h.reference_degrees,73.5);
    }
    sample(&m,163.5,3000); dashboard_heading_update(&h,&m,3000);
    CLOSE(h.relative_degrees,90); CLOSE(h.reference_degrees,73.5);
    m.degrees[2]=0; CHECK(!dashboard_heading_update(&h,&m,3000)); /* 有效计数保持相同。 */
    CLOSE(h.relative_degrees,90);
    /* 跨越传感器±180度边界时，实际方向只改变2度。 */
    dashboard_heading_init(&h); sample(&m,179,3020); CHECK(dashboard_heading_update(&h,&m,3020));
    sample(&m,-179,3040); dashboard_heading_update(&h,&m,3040); CLOSE(h.relative_degrees,2);
    dashboard_heading_init(&h); sample(&m,-179,3060); dashboard_heading_update(&h,&m,3060);
    sample(&m,179,3080); dashboard_heading_update(&h,&m,3080); CLOSE(h.relative_degrees,-2);
    /* 较大有限输入和整数整圈不应溢出或累计漂移。 */
    dashboard_heading_init(&h); sample(&m,359,3100); dashboard_heading_update(&h,&m,3100);
    sample(&m,-359,3120); dashboard_heading_update(&h,&m,3120); CLOSE(h.relative_degrees,2);
    for(int turn=-20;turn<=20;++turn) {
        sample(&m,359+90+turn*360.0,3140); dashboard_heading_update(&h,&m,3140);
        CLOSE(h.relative_degrees,90); vector(&h,0,1);
    }
    dashboard_heading_init(&h); sample(&m,1e308,3160); dashboard_heading_update(&h,&m,3160);
    sample(&m,-1e308,3180); dashboard_heading_update(&h,&m,3180);
    CHECK(isfinite(h.relative_degrees)); CHECK(h.relative_degrees>=-180 && h.relative_degrees<180);
    /* 新进程或会话重置建立新参考，不修改原始角度。 */
    dashboard_heading_init(&h); CHECK(!h.ready);
    sample(&m,-42.25,3200); m.degrees[0]=0.15; m.degrees[1]=-0.21;
    CHECK(dashboard_heading_update(&h,&m,3200)); CLOSE(h.reference_degrees,-42.25);
    CLOSE(h.relative_degrees,0); CLOSE(m.degrees[0],0.15); CLOSE(m.degrees[1],-0.21);
    CLOSE(m.degrees[2],-42.25);
    printf("PASS heading: %u checks\n",checks); return 0;
}
