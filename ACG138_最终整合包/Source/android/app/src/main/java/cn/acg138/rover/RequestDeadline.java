package cn.acg138.rover;

import java.net.SocketTimeoutException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/** 总操作预算包含连接、写入、响应头和响应体接收。 */
final class RequestDeadline implements AutoCloseable {
    private final long started=System.nanoTime(),budgetNanos;
    private final ScheduledFuture<?> cancellation;
    private volatile boolean expired;
    RequestDeadline(ScheduledExecutorService executor,Runnable cancel,long millis){
        budgetNanos=TimeUnit.MILLISECONDS.toNanos(millis);
        cancellation=executor.schedule(()->{expired=true;cancel.run();},millis,TimeUnit.MILLISECONDS);
    }
    void check() throws SocketTimeoutException {
        if(expired||System.nanoTime()-started>=budgetNanos)throw new SocketTimeoutException("absolute operation deadline");
    }
    @Override public void close(){cancellation.cancel(false);}
}
