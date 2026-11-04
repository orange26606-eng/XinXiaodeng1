package cn.acg138.rover;

/** 重连策略不保存凭据、运动意图或墙钟映射。 */
final class SessionRecovery {
    static final long HEARTBEAT_MS=200,CONTROL_MS=3000,VIEW_MS=10000,MAP_REQUEST_MS=8000,MAP_STALE_MS=15000;
    private long retryAfter;
    static String heartbeatMode(boolean foreground){return foreground?"control":"view";}
    synchronized boolean mayRetry(long now){if(now<retryAfter)return false;retryAfter=now+500;return true;}
    synchronized void succeeded(){retryAfter=0;}
}
