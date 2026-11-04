package cn.acg138.rover;

/** 自动启动事务。服务端时间用于令牌校验，手机时钟只计算网络操作期限。 */
final class AutoSwitch {
    static final long BUDGET_MS=2500, HTTP_MS=600;
    static final class Prepared {
        final String token; final long issuedUs,expiresUs;
        Prepared(String token,long issued,long expires) {
            if(token==null||!token.matches("[0-9a-f]{32}")||issued<=0||expires<=issued||expires-issued>8000000)
                throw new IllegalArgumentException("自动模式准备回执不兼容");
            this.token=token;issuedUs=issued;expiresUs=expires;
        }
    }
    static final class Accepted {
        final boolean queued;final long requestUs;
        Accepted(boolean queued,long requestUs){this.queued=queued;this.requestUs=requestUs;}
    }
    interface Guard {boolean current();long now();long nextSequence();}
    interface Transport {
        Prepared prepare(long sequence,int timeoutMs)throws Exception;
        Accepted start(long sequence,String token,int timeoutMs)throws Exception;
    }
    private static int remaining(Guard g,long until)throws Exception {
        if(!g.current())throw new java.io.InterruptedIOException("自动模式请求已取消");
        long left=until-g.now();if(left<=0)throw new java.io.InterruptedIOException("自动模式请求已过期，请重新操作");
        return (int)Math.min(HTTP_MS,left);
    }
    static long execute(Transport t,Guard g,long intentUntil)throws Exception {
        Prepared p=t.prepare(g.nextSequence(),remaining(g,intentUntil));
        Accepted a=t.start(g.nextSequence(),p.token,remaining(g,intentUntil));
        if(!a.queued||a.requestUs!=p.issuedUs)throw new Exception("自动模式回执与本次准备不匹配");
        // 入队回执确认请求已通过服务端时效校验，后续等待由心跳管理。
        // 回执不表示车轮已经转动。
        if(!g.current())throw new java.io.InterruptedIOException("自动模式请求已取消");
        return a.requestUs;
    }
    private AutoSwitch(){}
}
