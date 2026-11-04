package cn.acg138.rover;

/* 控制租约按独立心跳更新。 */
final class ControlCadence {
    static final long REQUEST_MS=600,SESSION_MS=8000,HOLD_MS=1500,STATE_MS=1500;
    private long lastAck;
    private int failures;
    synchronized void connected(long time){lastAck=time;failures=0;}
    synchronized void acknowledged(long sent,long completed){if(completed>=sent&&completed-sent<=NetworkTiming.HEARTBEAT_MS){lastAck=sent;failures=0;}}
    synchronized boolean failed(long time){failures++;return failures>=3&&time>=lastAck&&time-lastAck>=SESSION_MS;}
    synchronized int failures(){return failures;}
    synchronized long age(long time){return time>=lastAck?time-lastAck:Long.MAX_VALUE;}
    synchronized boolean expired(long time){return failures>=3&&age(time)>=SESSION_MS;}
}
