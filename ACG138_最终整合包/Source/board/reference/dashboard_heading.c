/* 1.1.0：校准后建立航向零点，处理相对航向回绕并保留方向。 */
#include "dashboard_heading.h"
#include <math.h>
#include <string.h>
void dashboard_heading_init(struct dashboard_heading *heading)
{
    memset(heading, 0, sizeof(*heading));
}
static double relative_yaw(double yaw, double reference)
{
    /* 分别取模，避免很大的有限输入造成溢出。 */
    double delta = fmod(yaw, 360.0) - fmod(reference, 360.0);
    if (delta >= 180.0) delta -= 360.0;
    if (delta < -180.0) delta += 360.0;
    /* 取模后的两操作数之差可能接近720度。 */
    if (delta >= 180.0) delta -= 360.0;
    if (delta < -180.0) delta += 360.0;
    return delta == 0.0 ? 0.0 : delta;
}
int dashboard_heading_update(struct dashboard_heading *heading,
                             const struct imu_model *model, uint64_t now_ms)
{
    if (!model || !model->have_angles || !model->good_count ||
        model->last_good_ms > now_ms || imu_model_state(model, now_ms) != IMU_LIVE ||
        !isfinite(model->degrees[2]) ||
        (heading->ready && heading->last_good_count == model->good_count)) return 0;
    int captured = !heading->ready;
    if (captured) {
        heading->reference_degrees = model->degrees[2];
        heading->ready = 1;
    }
    heading->relative_degrees = relative_yaw(model->degrees[2], heading->reference_degrees);
    heading->last_good_count = model->good_count;
    return captured;
}
int dashboard_heading_vector(const struct dashboard_heading *heading,
                             double *x, double *y)
{
    if (!heading || !heading->ready || !isfinite(heading->relative_degrees)) return 0;
    double radians = heading->relative_degrees * (3.14159265358979323846 / 180.0);
    *x = cos(radians); *y = sin(radians);
    return 1;
}
