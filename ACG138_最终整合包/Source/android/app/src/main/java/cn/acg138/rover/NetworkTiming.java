package cn.acg138.rover;
/* HTTP请求使用总时限，运动操作另行检查有效期。 */
final class NetworkTiming { static final int IO_MS=3000, TOTAL_MS=5000, STAGE_ACK_MS=8000, MAP_MS=8000, TASK_MS=8000, UI_MS=1500, HEARTBEAT_MS=1500; private NetworkTiming(){} }
