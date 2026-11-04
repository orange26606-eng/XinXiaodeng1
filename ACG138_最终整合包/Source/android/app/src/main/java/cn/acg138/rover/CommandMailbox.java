package cn.acg138.rover;

/** 只保存当前意图和STOP屏障，排队或重试的HOLD沿用原有效期。 */
final class CommandMailbox {
    static final class Intent {
        final String op,motion,origin; final long version,expiresAt; final boolean sourceExpired;
        Intent(String op,String motion,long version,long expiresAt,boolean sourceExpired,String origin){this.op=op;this.motion=motion;this.version=version;this.expiresAt=expiresAt;this.sourceExpired=sourceExpired;this.origin=origin;}
    }
    private String op="stop",motion=""; private long version=0,sent=-1,expiresAt=0;
    private boolean stopBarrier=true,expiredInput=false;
    private String origin="INITIAL_STOP";
    synchronized boolean offer(String next,String move){return offer(next,move,System.nanoTime()/1000000);}
    synchronized boolean offer(String next,String move,long now){
        return offer(next,move,now,"UI_"+next.toUpperCase(java.util.Locale.ROOT));
    }
    synchronized boolean offer(String next,String move,long now,String source){
        origin=sanitizeOrigin(source);
        if(next.equals("hold")){expiresAt=now+ControlCadence.HOLD_MS;expiredInput=false;}
        else expiresAt=0;
        if(op.equals(next)&&motion.equals(move)&&!next.equals("stop"))return false;
        op=next;motion=move;++version;if(next.equals("stop"))stopBarrier=true;return true;
    }
    synchronized boolean renewHold(String move,long now){if(!op.equals("hold")||!motion.equals(move)||now>=expiresAt)return false;expiresAt=now+ControlCadence.HOLD_MS;return true;}
    synchronized Intent take(){return take(System.nanoTime()/1000000);}
    synchronized Intent take(long now){
        if(op.equals("hold")&&now>=expiresAt){op="stop";motion="";++version;stopBarrier=true;expiredInput=true;origin="HOLD_INPUT_EXPIRED";}
        if(stopBarrier){stopBarrier=false;return new Intent("stop","",op.equals("stop")?version:-1,0,expiredInput,op.equals("stop")?origin:"MANUAL_STOP_BARRIER");}
        if(op.equals("hold")||version!=sent)return new Intent(op,motion,version,expiresAt,false,origin);
        return new Intent("ping","",version,0,false,"CONTROL_PING");
    }
    synchronized void sent(Intent value){if(value.version>=0)sent=Math.max(sent,value.version);}
    synchronized boolean pending(){return stopBarrier||sent!=version;}
    synchronized void reset(){op="stop";motion="";expiresAt=0;expiredInput=false;origin="SESSION_RESET_STOP";++version;sent=-1;stopBarrier=true;}
    static String sanitizeOrigin(String source){return source!=null&&source.matches("[A-Z][A-Z0-9_]{0,47}")?source:"UNKNOWN_CLIENT";}
}
