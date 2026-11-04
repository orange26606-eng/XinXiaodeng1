package cn.acg138.rover;

import android.app.Activity;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.SurfaceTexture;
import android.view.Surface;
import android.view.TextureView;
import android.widget.FrameLayout;
import android.os.Bundle;
import android.os.SystemClock;
import android.view.View;
import android.view.WindowManager;
import android.webkit.JavascriptInterface;
import android.webkit.WebResourceRequest;
import android.webkit.WebResourceResponse;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import java.util.concurrent.atomic.AtomicBoolean;
import org.json.JSONArray;
import org.json.JSONObject;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.Executors;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.List;

/** 本地界面与原生网络通信，外部网页不获得JS桥接权限。 */
public final class MainActivity extends Activity {
    private WebView web;
    private TapSpeech speech;
    private long speechGeneration;
    private final AtomicBoolean associationPending=new AtomicBoolean(),associationSnapshotPending=new AtomicBoolean(),objectNavigationPending=new AtomicBoolean();
    private final CommandMailbox commandMailbox=new CommandMailbox();
    private final AtomicBoolean controlDispatchQueued=new AtomicBoolean();
    private final H264VideoPlayer videoPlayer=new H264VideoPlayer();
    private TextureView videoTexture;
    private volatile Surface videoSurface;
    private String videoView="stereo";
    private long surfaceFrames,lastVideoJs,videoWindow,videoWindowFrames;
    private double displayedFps;
    private ScheduledExecutorService commands, telemetry, video, heartbeat;
    private final AtomicBoolean reconnectPending=new AtomicBoolean();
    private final SessionRecovery sessionRecovery=new SessionRecovery();
    private ExecutorService connections;
    private ExecutorService mapWorker, navigationWorker, motionWorker, controlIO, stopIO;
    private ExecutorService objectWorker, objectImageWorker, objectStateWorker, objectTaskWorker;
    private final AtomicBoolean objectScenePending=new AtomicBoolean(),objectTaskPending=new AtomicBoolean(),objectTaskRequestPending=new AtomicBoolean(),objectStatePending=new AtomicBoolean(),objectMapPending=new AtomicBoolean(),objectSnapshotPending=new AtomicBoolean(),objectSelectionPending=new AtomicBoolean();
    private AtomicLong objectSequence=new AtomicLong((new java.security.SecureRandom().nextLong() & ((1L<<50)-1))+1);
    private ScheduledExecutorService deadlines;
    private final ControlCadence cadence=new ControlCadence();
    private final AtomicLong controlEpoch=new AtomicLong();
    private volatile long serverReference,clientReference,lastControlDiagnostic;
    private final AtomicBoolean mapPending=new AtomicBoolean(),motionPending=new AtomicBoolean();
    private volatile boolean foreground, focused, linked, closed;
    private volatile long uiPulse, stateReceived;
    private volatile String base = "", token = "", session = "";
    private AtomicLong sequence = new AtomicLong();
    private AtomicLong navigationSequence = objectSequence;
    private long voiceAppliedAck;
    private final AtomicLong objectIntentEpoch=new AtomicLong();
    private volatile HttpURLConnection objectTaskConnection;
    private final AtomicBoolean navigationRequestPending = new AtomicBoolean();
    private final AtomicLong generation = new AtomicLong();
    private final AtomicLong connectionIntent = new AtomicLong();
    private SharedPreferences preferences;
    private volatile HttpURLConnection stateConnection, commandConnection;
    private static final class ApiFailure extends Exception {
        final int status; final String code;
        ApiFailure(int status, String message, String code) { super(message); this.status=status; this.code=code; }
    }

    @Override public void onCreate(Bundle state) {
        super.onCreate(state);
        speech=new TapSpeech(this,new TapSpeech.Sink(){public void event(String state,String message,String text,float rms){voiceEvent(state,message,text,rms);}public void finalText(String t){handleVoiceText(t);}});
        preferences = getSharedPreferences("connection", MODE_PRIVATE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN |
            View.SYSTEM_UI_FLAG_HIDE_NAVIGATION | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY |
            View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION |
            View.SYSTEM_UI_FLAG_LAYOUT_STABLE);
        web = new WebView(this); web.setBackgroundColor(Color.TRANSPARENT);
        WebSettings settings = web.getSettings();
        settings.setJavaScriptEnabled(true); settings.setDomStorageEnabled(true);
        settings.setAllowFileAccess(false); settings.setAllowContentAccess(false);
        settings.setMixedContentMode(WebSettings.MIXED_CONTENT_NEVER_ALLOW);
        settings.setSupportZoom(false); settings.setTextZoom(100);
        web.addJavascriptInterface(new Bridge(), "Rover");
        web.setWebViewClient(new WebViewClient() {
            @Override public boolean shouldOverrideUrlLoading(WebView view, WebResourceRequest request) { return true; }
            @Override public WebResourceResponse shouldInterceptRequest(WebView view, WebResourceRequest request) {
                String path = request.getUrl().getPath();
                if (!"https".equals(request.getUrl().getScheme()) ||
                    !"app.acg138.local".equals(request.getUrl().getHost()) || path == null ||
                    !(path.equals("/index.html") || path.equals("/style.css") || path.equals("/app.js") || path.equals("/odometry.js") || path.equals("/nav-map.js") || path.equals("/objects.js") || path.equals("/scene.js") || path.equals("/object-dual.js") || path.equals("/object-flow-diagnostics.js") || path.equals("/association.js") || path.equals("/voice.js"))) {
                    return new WebResourceResponse("text/plain", "UTF-8", 403, "Forbidden",
                        java.util.Collections.emptyMap(), new ByteArrayInputStream(new byte[0]));
                }
                try {
                    String type = path.endsWith(".css") ? "text/css" : path.endsWith(".js") ? "application/javascript" : "text/html";
                    return new WebResourceResponse(type, "UTF-8", getAssets().open(path.substring(1)));
                } catch (Exception error) { return new WebResourceResponse("text/plain", "UTF-8", new ByteArrayInputStream(new byte[0])); }
            }
        });
        FrameLayout content=new FrameLayout(this);content.setBackgroundColor(Color.rgb(16,23,25));
        videoTexture=new TextureView(this);videoTexture.setOpaque(true);
        content.addView(videoTexture,new FrameLayout.LayoutParams(1,1));
        content.addView(web,new FrameLayout.LayoutParams(-1,-1));setContentView(content);
        videoTexture.setSurfaceTextureListener(new TextureView.SurfaceTextureListener(){
            @Override public void onSurfaceTextureAvailable(SurfaceTexture texture,int w,int h){videoSurface=new Surface(texture);updateVideoTransform();}
            @Override public void onSurfaceTextureSizeChanged(SurfaceTexture texture,int w,int h){updateVideoTransform();}
            @Override public boolean onSurfaceTextureDestroyed(SurfaceTexture texture){videoPlayer.cancel();Surface old=videoSurface;videoSurface=null;if(old!=null)old.release();return true;}
            @Override public void onSurfaceTextureUpdated(SurfaceTexture texture){
                if(!linked||!foreground||videoPlayer.decodedFrames()==0)return;
                long time=now();surfaceFrames++;
                if(videoWindow==0){videoWindow=time;videoWindowFrames=surfaceFrames;}
                if(time-videoWindow>=1000){displayedFps=(surfaceFrames-videoWindowFrames)*1000.0/(time-videoWindow);videoWindow=time;videoWindowFrames=surfaceFrames;}
                if(time-lastVideoJs>=100){lastVideoJs=time;js("window.onNativeVideo(1280,720,"+displayedFps+")");}
            }
        });
        commands = Executors.newSingleThreadScheduledExecutor();
        controlIO=Executors.newSingleThreadExecutor();
        stopIO=new java.util.concurrent.ThreadPoolExecutor(1,1,0L,TimeUnit.MILLISECONDS,new java.util.concurrent.ArrayBlockingQueue<Runnable>(1),new java.util.concurrent.ThreadPoolExecutor.DiscardOldestPolicy());
        heartbeat = Executors.newSingleThreadScheduledExecutor();
        telemetry = Executors.newSingleThreadScheduledExecutor();
        video = Executors.newSingleThreadScheduledExecutor();
        connections = Executors.newSingleThreadExecutor();
        mapWorker = Executors.newSingleThreadExecutor();
        motionWorker = Executors.newSingleThreadExecutor();
        objectWorker = Executors.newSingleThreadExecutor();
        objectImageWorker=Executors.newSingleThreadExecutor();objectStateWorker=Executors.newSingleThreadExecutor();objectTaskWorker=new java.util.concurrent.ThreadPoolExecutor(1,1,0L,TimeUnit.MILLISECONDS,new java.util.concurrent.ArrayBlockingQueue<Runnable>(4));
        navigationWorker = Executors.newSingleThreadExecutor();
        deadlines=Executors.newScheduledThreadPool(2);
        commands.scheduleAtFixedRate(this::requestControlDispatch, 50, 80, TimeUnit.MILLISECONDS);
        heartbeat.scheduleAtFixedRate(this::heartbeatTick, 100, SessionRecovery.HEARTBEAT_MS, TimeUnit.MILLISECONDS);
        telemetry.scheduleWithFixedDelay(this::stateTick, 100, 160, TimeUnit.MILLISECONDS);
        video.scheduleWithFixedDelay(this::videoTick, 100, 300, TimeUnit.MILLISECONDS); // 重连等待时间，视频本身使用连续流。
        web.loadUrl("https://app.acg138.local/index.html");
    }

    private void js(String code) {
        if (closed) return;
        runOnUiThread(() -> { if (!closed && web != null) web.evaluateJavascript(code, null); });
    }
    private void voiceJs(String code){
        if(closed)return;
        runOnUiThread(()->{if(!closed&&web!=null){try{web.evaluateJavascript(code,null);}catch(RuntimeException ignored){/* JS桥接不可用时上报状态，服务回调正常返回。 */}}});
    }
    private void voiceEvent(String state,String message,String text,float rms){
        String event="{state:"+JSONObject.quote(state)+",message:"+JSONObject.quote(message)+",text:"+(text==null?"null":JSONObject.quote(text))+",rms:"+(Float.isFinite(rms)?rms:-1)+"}";
        voiceJs("try{if(typeof window.voiceEvent==='function')window.voiceEvent("+event+");}catch(e){console.error('voice UI callback failed',e);}");
    }
    private void handleVoiceText(String text){
        if(speechGeneration!=generation.get()||!foreground||!focused||closed){voiceEvent("error","页面已离开前台、失去焦点或连接已变化；本次文字未执行，请返回后重试。",text,-1);return;}
        VoiceCommands.Intent intent=VoiceCommands.parse(text);
        voiceJs("try{if(typeof window.voiceFinal==='function')window.voiceFinal("+JSONObject.quote(text)+","+(intent==null?"null":"{action:"+JSONObject.quote(intent.action)+",category:"+JSONObject.quote(intent.category)+",color:"+JSONObject.quote(intent.color)+",size:"+JSONObject.quote(intent.size)+",position:"+JSONObject.quote(intent.position)+",ordinal:"+intent.ordinal+",reference:"+intent.reference+",mentioned:"+intent.mentioned+"}")+");}catch(e){console.error('voice final callback failed',e);}");
    }
    private static long now() { return SystemClock.elapsedRealtime(); }
    private void fail(String text, long current) {
        if (current != generation.get() || !linked) return;
        linked = false;videoPlayer.cancel();
        commandMailbox.offer("stop","");
        // 即使故障来自遥测线程，也将STOP排在正在发送的HOLD之后。
        // linked变为false后，排队的运动请求直接返回，不再发送。
        // 即使尽力发送的STOP失败，板端独立租约仍会使车辆停车。
        try { commands.execute(() -> {
            try { postControl("stop", "", current); } catch (Exception ignored) { }
        }); } catch (java.util.concurrent.RejectedExecutionException ignored) { }
        js("window.connectionError(" + JSONObject.quote(text) + ")");
    }
    private HttpURLConnection open(String path) throws Exception {return open(path,NetworkTiming.IO_MS);}
    private HttpURLConnection open(String path,int timeout) throws Exception {
        HttpURLConnection c = (HttpURLConnection)new URL(base + path).openConnection();
        c.setConnectTimeout(timeout); c.setReadTimeout(timeout);
        c.setInstanceFollowRedirects(false); c.setUseCaches(false);
        c.setRequestProperty("Authorization", "Bearer " + token);
        c.setRequestProperty("Connection", "close");
        return c;
    }
    private static byte[] read(InputStream stream, int max) throws Exception {
        try (InputStream in = stream; ByteArrayOutputStream out = new ByteArrayOutputStream()) {
            byte[] buffer = new byte[8192]; int got;
            while ((got = in.read(buffer)) >= 0) { if (out.size() + got > max) throw new Exception("response too large"); out.write(buffer,0,got); }
            return out.toByteArray();
        }
    }
    private void checkNavigationResponse(HttpURLConnection c,String operation) throws Exception {
        int status=c.getResponseCode();if(status==200)return;
        String code="",message="";
        try {InputStream in=c.getErrorStream();if(in!=null){JSONObject error=new JSONObject(new String(read(in,4096),StandardCharsets.UTF_8));code=error.optString("code","");message=error.optString("error","");}}catch(Exception ignored){}
        throw NavigationFailure.http(operation,status,code,message);
    }
    private void reportNavigationFailure(String operation,Exception error) {
        NavigationFailure f=NavigationFailure.from(operation,error);JSONObject data=new JSONObject();
        try{data.put("operation",f.operation);data.put("code",f.code);data.put("http_status",f.httpStatus);data.put("message",f.getMessage());}catch(Exception ignored){}
        android.util.Log.w("ACG138_NAV_HTTP",f.operation+" code="+f.code+" http="+f.httpStatus);
        js("window.navRequestError("+data.toString()+")");
    }
    private static boolean isSessionExpired(Exception e){
        return e instanceof ApiFailure&&((ApiFailure)e).status==401||e instanceof NavigationFailure&&((NavigationFailure)e).httpStatus==401;
    }
    /* 本地控制线程独立维护租约。 */
    private void heartbeatTick(){
        if(SemanticTaskService.owns())return;
        if(!linked||closed||session.isEmpty()||reconnectPending.get())return;
        final long current=generation.get(),sent=now();final String owner=session;
        HttpURLConnection c=null;RequestDeadline total=null;
        try{
            c=open("/api/heartbeat",NetworkTiming.HEARTBEAT_MS);final HttpURLConnection request=c;total=new RequestDeadline(deadlines,request::disconnect,NetworkTiming.HEARTBEAT_MS);
            c.setRequestProperty("X-Rover-Session",owner);c.setRequestMethod("POST");c.setRequestProperty("Content-Type","application/json");c.setDoOutput(true);
            byte[] payload=new JSONObject().put("mode",SessionRecovery.heartbeatMode(foreground)).toString().getBytes(StandardCharsets.UTF_8);c.setFixedLengthStreamingMode(payload.length);
            try(java.io.OutputStream out=c.getOutputStream()){out.write(payload);}
            checkNavigationResponse(c,"heartbeat");JSONObject answer=new JSONObject(new String(read(c.getInputStream(),4096),StandardCharsets.UTF_8));total.check();
            if(current!=generation.get()||!owner.equals(session))return;
            if(foreground&&answer.optBoolean("control_live",false))cadence.acknowledged(sent,now());
        }catch(Exception e){
            if(current==generation.get()&&linked&&!closed){
                if(isSessionExpired(e))reconnectSession("会话已过期",current);
                else if(foreground&&cadence.age(now())>=ControlCadence.SESSION_MS)reconnectSession("心跳持续失败",current);
            }
        }finally{if(total!=null)total.close();if(c!=null)c.disconnect();}
    }
    /* 只读请求恢复与运动请求分开重试。 */
    private void reconnectSession(String reason,long current){
        if(SemanticTaskService.owns()){js("window.controlLinkStatus(\"语音任务原会话等待恢复；未自动换会话或重发开始\")");return;}
        if(current!=generation.get()||!linked||closed||!sessionRecovery.mayRetry(now())||!reconnectPending.compareAndSet(false,true))return;
        cancelOldControl();commandMailbox.reset();videoPlayer.cancel();
        js("window.navSessionState&&window.navSessionState('reconnecting',"+JSONObject.quote(reason+"，正在重连；保留旧图，不自动恢复运动")+")");
        try{connections.execute(()->{
            try{
                if(current!=generation.get()||closed||!linked)return;
                JSONObject answer=post("/api/session",new JSONObject().put("client","ACG138 Android view reconnect"),1800);
                if(current!=generation.get()||closed||!linked)return;
                if(!MotionIdentity.sessionMatches(answer.optLong("config_id",-1),answer.optInt("status_revision",-1)))throw new Exception(MotionIdentity.mismatch(answer.optLong("config_id",-1),answer.optInt("status_revision",-1)));
                serverReference=answer.getLong("server_ms");clientReference=now();
                String next=answer.getString("session");if(next.length()!=32)throw new Exception("会话响应无效");
                generation.incrementAndGet();session=next;commandMailbox.reset();cadence.connected(now());sessionRecovery.succeeded();
                js("window.navSessionState&&window.navSessionState('connected','重连成功；地图继续读取，运动需重新明确开始')");
            }catch(Exception e){
                if(current==generation.get()&&!closed)js("window.navSessionState&&window.navSessionState('waiting','会话重连未完成，稍后重试；旧图仅供查看')");
            }finally{reconnectPending.set(false);}
        });}catch(java.util.concurrent.RejectedExecutionException ignored){reconnectPending.set(false);}
    }
    private long serverTime(long clientTime) {
        long delta=clientTime-clientReference;
        // 会话响应时间用作下界，回包传输只会使
        // 映射后的期限更早，同时计入有界时钟漂移。
        return serverReference+delta-Math.max(0,delta)/5000-1;
    }
    private JSONObject post(String path,JSONObject body,int timeout)throws Exception{return post(path,body,timeout,-1);}
    private JSONObject post(String path, JSONObject body, int timeout,long expectedEpoch) throws Exception {
        return post(path,body,timeout,expectedEpoch,null);
    }
    private JSONObject post(String path, JSONObject body, int timeout,long expectedEpoch,String origin) throws Exception {
        HttpURLConnection c=open(path,timeout);commandConnection=c;
        RequestDeadline deadline=new RequestDeadline(deadlines,c::disconnect,timeout);
        try {
            if(expectedEpoch>=0&&expectedEpoch!=controlEpoch.get())throw new java.io.InterruptedIOException("superseded control");
            byte[] bytes=body.toString().getBytes(StandardCharsets.UTF_8);
            c.setRequestMethod("POST");c.setRequestProperty("Content-Type","application/json");c.setDoOutput(true);c.setFixedLengthStreamingMode(bytes.length);
            if(origin!=null)c.setRequestProperty("X-Rover-Control-Origin",CommandMailbox.sanitizeOrigin(origin));
            try(java.io.OutputStream out=c.getOutputStream()){if(expectedEpoch>=0&&expectedEpoch!=controlEpoch.get())throw new java.io.InterruptedIOException("superseded control");out.write(bytes);}
            int status=c.getResponseCode();byte[] response=read(status<400?c.getInputStream():c.getErrorStream(),32768);
            deadline.check();
            JSONObject result=new JSONObject(new String(response,StandardCharsets.UTF_8));
            if(status>=400)throw new ApiFailure(status,result.optString("error","连接被拒绝"),result.optString("code","HTTP_"+status));
            return result;
        } finally {deadline.close();c.disconnect();if(commandConnection==c)commandConnection=null;}
    }
    private void postControl(String op,String motion,long current) throws Exception {postControl(op,motion,current,0);}
    private void postControl(String op,String motion,long current,long holdExpiry)throws Exception{postControl(op,motion,current,holdExpiry,controlEpoch.get());}
    private void postControl(String op,String motion,long current,long holdExpiry,long expectedEpoch) throws Exception {
        postControl(op,motion,current,holdExpiry,expectedEpoch,"DIRECT_"+op.toUpperCase(java.util.Locale.ROOT));
    }
    private void postControl(String op,String motion,long current,long holdExpiry,long expectedEpoch,String origin) throws Exception {
        SemanticTaskService.awaitRevocation();
        synchronized(SemanticTaskService.WIRE){
        if(current!=generation.get()||session.isEmpty()||SemanticTaskService.owns())return;
        long sent=now();JSONObject body=new JSONObject();body.put("session",session);body.put("seq",sequence.incrementAndGet());body.put("op",op);body.put("motion",motion);
        body.put("sent_ms",serverTime(sent));body.put("hold_until_ms",op.equals("hold")?serverTime(holdExpiry):0);
        post("/api/control",body,(int)ControlCadence.REQUEST_MS,expectedEpoch,origin);
        cadence.acknowledged(sent,now());
        }
    }
    private void cancelOldControl(){abortObjectPendingTask();controlEpoch.incrementAndGet();HttpURLConnection c=commandConnection;if(c!=null)c.disconnect();}
    private void requestControlDispatch(){
        if(!linked||closed||controlIO==null||controlIO.isShutdown()||!controlDispatchQueued.compareAndSet(false,true))return;
        try{controlIO.execute(()->{
            try{commandTick();}finally{
                controlDispatchQueued.set(false);
                if(linked&&!closed&&!SemanticTaskService.owns()&&commandMailbox.pending())requestControlDispatch();
            }
        });}catch(java.util.concurrent.RejectedExecutionException ignored){controlDispatchQueued.set(false);}
    }
    private void commandTick() {
        if(SemanticTaskService.owns())return;
        long a=SemanticTaskService.lastHeartbeatAck();if(a>voiceAppliedAck){voiceAppliedAck=a;cadence.acknowledged(a,a);}
        if(!linked||closed||reconnectPending.get()||(!foreground&&!commandMailbox.pending()))return;long current=generation.get(),epoch=controlEpoch.get();
        try {
            if(foreground&&cadence.expired(now())){reconnectSession("控制会话过期",current);return;}
            // 不依赖JS脉冲、状态或绘制，只有新的真实输入
            // 才能刷新HOLD的1500ms源期限，重试不延长期限。
            CommandMailbox.Intent intent=commandMailbox.take(now());
            if(!foreground&&!intent.op.equals("stop"))return;
            if(intent.op.equals("hold")&&intent.expiresAt<=now())return;
            postControl(intent.op,intent.motion,current,intent.expiresAt,epoch,intent.origin);
            if(intent.sourceExpired)js("window.manualControlUnavailable('手动输入已过期，请重新按下方向键')");
            if(epoch==controlEpoch.get())commandMailbox.sent(intent);
        } catch(Exception error) {
            if(epoch!=controlEpoch.get()||current!=generation.get()||!linked)return;
            if(error instanceof ApiFailure){
                ApiFailure f=(ApiFailure)error;
                if(f.status==401){reconnectSession("控制会话已过期",current);return;}
                if(f.status==409){
                    if(f.code.equals("FINAL_MOTOR_FAULT")||f.code.equals("CONTROL_LINK_LOST")||f.code.equals("INPUT_EXPIRED")){
                        stopNow();js("window.manualControlUnavailable("+JSONObject.quote(f.getMessage())+")");
                    } else js("window.controlLinkStatus('等待控制握手，感知和地图不影响手动')");
                    return;
                }
            }
            boolean disconnected=cadence.failed(now());
            if(now()-lastControlDiagnostic>=1000){lastControlDiagnostic=now();android.util.Log.w("ACG138_CONTROL_HTTP","failures="+cadence.failures()+" ack_age_ms="+cadence.age(now())+" type="+error.getClass().getSimpleName()+" ui_age_ms="+(now()-uiPulse));}
            js("window.controlLinkStatus("+JSONObject.quote(cadence.failures()>=3?"控制请求连续失败，正在重试":"控制请求短时异常，正在重试")+")");
            if(disconnected&&foreground)reconnectSession("控制链路持续异常",current);
        }
    }
    private void stateTick() {
        if(!linked||!foreground||closed)return;long current=generation.get();HttpURLConnection c=null;RequestDeadline deadline=null;
        try {
            c=open("/api/state",(int)ControlCadence.STATE_MS);stateConnection=c;final HttpURLConnection request=c;
            deadline=new RequestDeadline(deadlines,request::disconnect,ControlCadence.STATE_MS);
            if(c.getResponseCode()!=200)throw new Exception("state rejected");
            JSONObject parsed=new JSONObject(new String(read(c.getInputStream(),32768),StandardCharsets.UTF_8));
            deadline.check();
            if(!parsed.has("control")||!parsed.has("telemetry"))throw new Exception("invalid/late state");
            if(current!=generation.get()||!linked||!foreground)return;
            stateReceived=now();js("window.receiveState("+parsed.toString()+")");
        } catch(Exception error) {
            if(current==generation.get()&&linked)js("window.stateReadError('状态读取暂时异常，保留上次数据；手动控制独立运行')");
        } finally {if(deadline!=null)deadline.close();if(c!=null)c.disconnect();if(stateConnection==c)stateConnection=null;}
    }
    private void updateVideoTransform(){
        if(videoTexture==null||videoTexture.getWidth()<=0||videoTexture.getHeight()<=0)return;
        Matrix matrix=new Matrix();matrix.setValues(VideoGeometry.matrix(videoTexture.getWidth(),videoTexture.getHeight(),videoView));videoTexture.setTransform(matrix);
    }
    private void videoTick(){
        Surface surface=videoSurface;
        if(!linked||!foreground||!focused||closed||surface==null||!surface.isValid())return;
        long current=generation.get();
        try{videoPlayer.play(base,token,session,surface,()->current==generation.get()&&linked&&foreground&&focused&&!closed&&surface==videoSurface);}
        catch(Exception error){
            if(current==generation.get()&&linked&&foreground&&!closed)
                js("window.nativeVideoError("+JSONObject.quote(error.getMessage()==null?"视频重新连接中":error.getMessage())+")");
        }
    }
    private void abortObjectPendingTask(){objectIntentEpoch.incrementAndGet();HttpURLConnection c=objectTaskConnection;if(c!=null)c.disconnect();objectTaskRequestPending.set(false);}
    private void stopNow(){
        stopNow("UI_STOP");
    }
    private void stopNow(String origin){
        SemanticTaskService.cancel("用户明确停车/控制方式切换："+origin);
        commandMailbox.offer("stop","",now(),origin);cancelOldControl();priorityStop(origin);requestControlDispatch();
    }
    /* STOP使用独立执行队列。 */
    private void priorityStop(String origin){
        if(!linked||closed||session.isEmpty()||stopIO==null||stopIO.isShutdown())return;
        final long connection=generation.get(),epoch=controlEpoch.get();final String owner=session;
        try{stopIO.execute(()->{HttpURLConnection c=null;RequestDeadline deadline=null;try{
            if(connection!=generation.get()||epoch!=controlEpoch.get()||!owner.equals(session))return;
            JSONObject b=new JSONObject().put("session",owner).put("seq",sequence.incrementAndGet()).put("op","stop").put("motion","").put("sent_ms",serverTime(now())).put("hold_until_ms",0);
            c=open("/api/control",(int)ControlCadence.REQUEST_MS);final HttpURLConnection request=c;deadline=new RequestDeadline(deadlines,request::disconnect,ControlCadence.REQUEST_MS);
            c.setRequestMethod("POST");c.setRequestProperty("Content-Type","application/json");c.setRequestProperty("X-Rover-Control-Origin",CommandMailbox.sanitizeOrigin(origin));c.setDoOutput(true);byte[] body=b.toString().getBytes(StandardCharsets.UTF_8);c.setFixedLengthStreamingMode(body.length);
            try(java.io.OutputStream out=c.getOutputStream()){if(connection!=generation.get()||epoch!=controlEpoch.get()||!owner.equals(session))return;out.write(body);}c.getResponseCode();
        }catch(Exception ignored){android.util.Log.w("ACG138_STOP","优先停车请求未确认；板端独立超时仍生效");}finally{if(deadline!=null)deadline.close();if(c!=null)c.disconnect();}});}catch(java.util.concurrent.RejectedExecutionException ignored){}
    }
    private static boolean validHost(String host) {
        if(!host.matches("(?:[0-9]{1,3}\\.){3}[0-9]{1,3}(?::[0-9]{1,5})?"))return false;
        for(String octet:host.split(":")[0].split("\\."))if(Integer.parseInt(octet)>255)return false;
        if(host.contains(":")){int port=Integer.parseInt(host.split(":")[1]);if(port<1||port>65535)return false;}
        return true;
    }
    private boolean roverAt(String host) {
        if(!validHost(host))return false;
        HttpURLConnection c=null;RequestDeadline total=null;
        try {
            c=(HttpURLConnection)new URL("http://"+host+(host.contains(":")?"":":8080")+"/api/info").openConnection();
            c.setConnectTimeout(500);c.setReadTimeout(500);c.setInstanceFollowRedirects(false);c.setUseCaches(false);
            final HttpURLConnection request=c;total=new RequestDeadline(deadlines,request::disconnect,500);if(c.getResponseCode()!=200)return false;
            JSONObject info=new JSONObject(new String(read(c.getInputStream(),2048),StandardCharsets.UTF_8));
            total.check();return "acg138-rover".equals(info.optString("service"))&&info.optInt("protocol")==1;
        } catch(Exception ignored){return false;}
        finally {if(total!=null)total.close();if(c!=null)c.disconnect();}
    }
    private void beginConnection(String host,String password) {
        String requested="http://"+host+(host.contains(":")?"":":8080");if(SemanticTaskService.attach(requested,password)){base=requested;token=password;session=SemanticTaskService.currentSession();sequence=SemanticTaskService.controlSequence();navigationSequence=objectSequence=SemanticTaskService.navigationSequence();linked=true;stateReceived=now();uiPulse=now();js("window.connectionEstablished("+JSONObject.quote(host)+")");return;}

        final long intent=connectionIntent.incrementAndGet();
        stopNow();
        commands.execute(()->{
            if(closed||intent!=connectionIntent.get())return;
            try{postControl("stop","",generation.get());}catch(Exception ignored){}
            linked=false;long current=generation.incrementAndGet();session="";
            if(stateConnection!=null)stateConnection.disconnect();videoPlayer.cancel();
            connections.execute(()->{
                try {
                    String selected=host;
                    if(selected.isEmpty()){
                        js("window.connectionProgress('正在自动查找小车…')");
                        List<String> hosts=DeviceDiscovery.scan(()->closed||!foreground||intent!=connectionIntent.get()||current!=generation.get());
                        if(closed||!foreground||intent!=connectionIntent.get()||current!=generation.get())return;
                        if(hosts.isEmpty()){
                            String previous=preferences.getString("host","");
                            if(roverAt(previous))hosts.add(previous);
                        }
                        if(hosts.isEmpty())throw new Exception("未找到小车，请确认板端新版通信服务已启动；可展开手动地址连接");
                        if(hosts.size()>1){
                            js("window.discoveryFound("+new JSONArray(hosts).toString()+")");return;
                        }
                        selected=hosts.get(0);
                    }
                    if(closed||!foreground||intent!=connectionIntent.get()||current!=generation.get())return;
                    if(!validHost(selected))throw new Exception("板子地址无效");
                    base="http://"+selected+(selected.contains(":")?"":":8080");token=password;
                    sequence.set(0);commandMailbox.reset();
                    js("window.connectionProgress('已找到小车，正在验证密码…')");
                    JSONObject result=post("/api/session",new JSONObject().put("client","ACG138 Android"),1800);
                    if(!MotionIdentity.sessionMatches(result.optLong("config_id",-1),result.optInt("status_revision",-1)))throw new Exception(MotionIdentity.mismatch(result.optLong("config_id",-1),result.optInt("status_revision",-1)));
                serverReference=result.getLong("server_ms");clientReference=now();
                    if(closed||!foreground||intent!=connectionIntent.get()||current!=generation.get())return;
                    session=result.getString("session");
                    commandMailbox.reset();
                    stateReceived=now();uiPulse=now();cadence.connected(now());linked=true;
                    preferences.edit().putString("host",selected).remove("password").remove("token").apply();
                    js("window.connectionEstablished("+JSONObject.quote(selected)+")");
                } catch(Exception error){
                    if(intent==connectionIntent.get()&&current==generation.get()){
                        linked=false;
                        js("window.connectionError("+JSONObject.quote("连接失败："+error.getMessage())+")");
                    }
                }
            });
        });
    }
    private JSONObject autoPost(String path,JSONObject payload,String owner,int timeout)throws Exception {
        HttpURLConnection c=open(path,timeout);RequestDeadline total=new RequestDeadline(deadlines,c::disconnect,timeout);
        try {
            byte[] data=payload.toString().getBytes(StandardCharsets.UTF_8);
            c.setRequestProperty("X-Rover-Session",owner);c.setRequestProperty("Content-Type","application/json");
            c.setRequestMethod("POST");c.setDoOutput(true);c.setFixedLengthStreamingMode(data.length);
            try(java.io.OutputStream out=c.getOutputStream()){out.write(data);}
            checkNavigationResponse(c,"auto");JSONObject result=new JSONObject(new String(read(c.getInputStream(),4096),StandardCharsets.UTF_8));total.check();return result;
        } finally {total.close();c.disconnect();}
    }
    private void startAutomatic(){
        SemanticTaskService.cancel("用户明确切换自动巡航");
        if(!linked||!foreground||closed||commands==null||navigationWorker==null){js("window.autoModeError("+JSONObject.quote("手机连接或前台状态未就绪，本次没有发送自动模式请求")+")");return;}
        if(!navigationRequestPending.compareAndSet(false,true)){js("window.autoModeError("+JSONObject.quote("上一笔导航请求仍在发送，本次没有切换自动模式，请稍后重试")+")");return;}
        cancelOldControl();commandMailbox.offer("stop","",now());
        final long current=generation.get(),epoch=controlEpoch.get(),intentUntil=now()+AutoSwitch.BUDGET_MS;final String owner=session;
        commands.execute(()->{
            try{
                SemanticTaskService.awaitRevocationForAuto();
                if(current!=generation.get()||epoch!=controlEpoch.get()||!linked||!owner.equals(session))throw new Exception("自动模式请求已取消");
                // 签发切换令牌前，先完成旧STOP屏障。
                // 后续心跳可继续运行，新的手动或STOP操作会撤销本次切换。
                CommandMailbox.Intent stop=commandMailbox.take(now());postControl("stop","",current,0,epoch);commandMailbox.sent(stop);
                navigationWorker.execute(()->{try{
                    AutoSwitch.Guard guard=new AutoSwitch.Guard(){
                        public boolean current(){return current==generation.get()&&epoch==controlEpoch.get()&&linked&&foreground&&!closed&&owner.equals(session);}
                        public long now(){return MainActivity.now();}
                        public long nextSequence(){return navigationSequence.incrementAndGet();}
                    };
                    long request=AutoSwitch.execute(new AutoSwitch.Transport(){
                        public AutoSwitch.Prepared prepare(long seq,int timeout)throws Exception {
                            JSONObject answer=autoPost("/api/nav/auto/prepare",new JSONObject().put("seq",seq),owner,timeout);
                            return new AutoSwitch.Prepared(answer.getString("switch_token"),Long.parseLong(answer.getString("issued_us")),Long.parseLong(answer.getString("expires_us")));
                        }
                        public AutoSwitch.Accepted start(long seq,String token,int timeout)throws Exception {
                            JSONObject answer=autoPost("/api/nav/auto",new JSONObject().put("seq",seq).put("switch_token",token),owner,timeout);
                            return new AutoSwitch.Accepted(answer.optBoolean("queued"),Long.parseLong(answer.getString("auto_request_us")));
                        }
                    },guard,intentUntil);
                    if(guard.current())js("window.autoModeQueued("+JSONObject.quote(Long.toString(request))+")");
                }catch(Exception error){if(current==generation.get()&&epoch==controlEpoch.get())js("window.autoModeError("+JSONObject.quote(error.getMessage()==null?"自动模式启动失败":error.getMessage())+")");}
                finally{navigationRequestPending.set(false);}});
            }catch(Exception error){navigationRequestPending.set(false);if(current==generation.get()&&epoch==controlEpoch.get())js("window.autoModeError("+JSONObject.quote(error.getMessage()==null?"自动模式启动失败":error.getMessage())+")");}
        });
    }
    private final class Bridge {
        @JavascriptInterface public String semanticState(){return SemanticTaskService.snapshot();}
        @JavascriptInterface public void semanticCancel(){stopNow("VOICE_USER_STOP");}
        @JavascriptInterface public void semanticStart(){if(foreground&&focused&&!closed)SemanticTaskService.startPrepared();}
        @JavascriptInterface public void semanticSubmit(String intent){if(!linked||!foreground||!focused||closed)return;cancelOldControl();final long revision=objectIntentEpoch.get(),connection=generation.get();runOnUiThread(()->{if(revision!=objectIntentEpoch.get()||connection!=generation.get()||!linked||!foreground||!focused||closed)return;try{commandMailbox.reset();SemanticTaskService.submit(MainActivity.this,base,token,session,sequence,navigationSequence,intent);}catch(Exception e){voiceEvent("error",e.getMessage(),null,-1);}});}

        private void objectRead(String path,String callback,int maximum,AtomicBoolean pending){objectRead(path,callback,maximum,pending,null,-1);}
        private void objectRead(String path,String callback,int maximum,AtomicBoolean pending,String viewToken,int requestedEye){
            final ExecutorService reader=path.startsWith("/api/objects/snapshot")?objectImageWorker:path.equals("/api/objects/state")?objectStateWorker:path.equals("/api/objects/task")?objectTaskWorker:objectWorker;
            if(!linked||!foreground||!focused||closed||reader==null||!pending.compareAndSet(false,true)){if(viewToken!=null)js("window.objectSnapshotDeferred("+JSONObject.quote(viewToken)+")");return;}
            final long current=generation.get();final String objectSession=session;
            try{reader.execute(()->{HttpURLConnection c=null;RequestDeadline total=null;try{
                if(current!=generation.get()||!linked||closed||!objectSession.equals(session))return;
                long objectBegan=now();c=open(path);final HttpURLConnection request=c;total=new RequestDeadline(deadlines,request::disconnect,NetworkTiming.TOTAL_MS);c.setRequestProperty("X-Rover-Session",objectSession);
                int status=c.getResponseCode();
                if(status!=200){String detail="";try{java.io.InputStream error=c.getErrorStream();if(error!=null){JSONObject body=new JSONObject(new String(read(error,8192),StandardCharsets.UTF_8));detail=body.optString("error","");if(detail.length()>180)detail=detail.substring(0,180);}}catch(Exception ignored){}throw new Exception("HTTP "+status+(detail.isEmpty()?" · 实物服务尚未就绪":" · "+detail));}
                JSONObject data=new JSONObject(new String(read(c.getInputStream(),maximum),StandardCharsets.UTF_8));
                if(data.optInt("version")!=1)throw new Exception("实物数据版本不兼容");total.check();data.put("client_delay_ms",Math.max(0,now()-objectBegan));if(viewToken!=null){data.put("_client_view_token",viewToken);data.put("_client_requested_eye",requestedEye);}
                if(current==generation.get()&&linked&&foreground&&!closed&&objectSession.equals(session))js(callback+"("+data.toString()+")");
            }catch(Exception e){if(current==generation.get()&&!closed)js((path.equals("/api/objects/task")?"window.objectTaskError(\"read\",":"window.objectReceiveError("+JSONObject.quote(path)+",")+JSONObject.quote(e.getMessage()==null?"实物读取失败":e.getMessage())+")");}
            finally{if(total!=null)total.close();if(c!=null)c.disconnect();pending.set(false);}});}catch(java.util.concurrent.RejectedExecutionException ignored){pending.set(false);}
        }
        @JavascriptInterface public void voiceListen(boolean networkChosen){runOnUiThread(()->{if(foreground&&focused&&!closed){speechGeneration=generation.get();speech.start(networkChosen);}else voiceEvent("error","页面当前不在前台或没有焦点；请关闭系统弹窗并返回本页，再点击说话。",null,-1);});}
        @JavascriptInterface public void voiceText(String text){runOnUiThread(()->{if(foreground&&focused&&!closed){speechGeneration=generation.get();speech.text(text);}else voiceEvent("error","页面当前不在前台或没有焦点；请返回本页后再次提交文字。",text,-1);});}
        @JavascriptInterface public void voiceCancel(){runOnUiThread(()->speech.cancel());}
        @JavascriptInterface public void objectReadAssociation(){objectRead("/api/objects/association","window.objectReceiveAssociation",65536,associationPending);}
        @JavascriptInterface public void objectAssociationSnapshot(){objectRead("/api/objects/association-snapshot","window.objectReceiveAssociationSnapshot",3*1024*1024,associationSnapshotPending);}
        @JavascriptInterface public int voiceResolve(String category,String position,int ordinal,String candidates,long age){
            try{JSONArray a=new JSONArray(candidates);if(a.length()>32)return -1;VoiceCommands.Candidate[] c=new VoiceCommands.Candidate[a.length()];for(int i=0;i<c.length;i++){JSONObject o=a.getJSONObject(i);c[i]=new VoiceCommands.Candidate(o.getInt("track_id"),o.getString("label"),o.getDouble("x"));}return VoiceCommands.resolve(new VoiceCommands.Intent("select",category,position,ordinal),c,age);}catch(Exception e){return -1;}
        }
        @JavascriptInterface public void navPreviewObject(String map,String version,String epoch,int track,String association,double standoff){
            if(!linked||!foreground||!focused||closed||objectWorker==null||!objectNavigationPending.compareAndSet(false,true))return;
            final long current=generation.get();final String navSession=session;final String payload;
            try{payload=ObjectNavigationPayload.build(navigationSequence.incrementAndGet(),map,version,epoch,track,association,standoff);}catch(Exception e){objectNavigationPending.set(false);js("window.objectNavigationError("+JSONObject.quote(e.getMessage())+")");return;}
            try{objectWorker.execute(()->{HttpURLConnection c=null;RequestDeadline total=null;try{
                if(current!=generation.get()||!linked||!foreground||!focused||closed||!navSession.equals(session))return;
                byte[] body=payload.getBytes(StandardCharsets.UTF_8);c=open("/api/nav/object");total=new RequestDeadline(deadlines,c::disconnect,NetworkTiming.TOTAL_MS);c.setRequestProperty("X-Rover-Session",navSession);c.setRequestProperty("Content-Type","application/json");c.setRequestMethod("POST");c.setDoOutput(true);c.setFixedLengthStreamingMode(body.length);try(java.io.OutputStream out=c.getOutputStream()){out.write(body);}
                if(c.getResponseCode()!=200)throw new Exception("实物预览被拒绝，请检查身份、关联与定位");JSONObject ack=new JSONObject(new String(read(c.getInputStream(),4096),StandardCharsets.UTF_8));String id=ack.optString("object_request_us","");if(!ack.optBoolean("queued")||!id.matches("[0-9]{1,20}"))throw new Exception("实物预览确认格式不兼容");
                if(current==generation.get()&&linked&&!closed&&navSession.equals(session))js("window.objectNavigationQueued("+JSONObject.quote(id)+")");
            }catch(Exception e){if(current==generation.get()&&!closed)js("window.objectNavigationError("+JSONObject.quote(e.getMessage()==null?"实物预览失败":e.getMessage())+")");}finally{if(total!=null)total.close();if(c!=null)c.disconnect();objectNavigationPending.set(false);}});}catch(java.util.concurrent.RejectedExecutionException e){objectNavigationPending.set(false);}
        }
        @JavascriptInterface public void objectReadState(){objectRead("/api/objects/state","window.objectReceiveState",262144,objectStatePending);}
        @JavascriptInterface public void objectReadMap(){objectRead("/api/objects/map","window.navReceiveObjectsMap",131072,objectMapPending);}
        @JavascriptInterface public void objectReadScene(){objectRead("/api/objects/scene","window.objectReceiveScene",262144,objectScenePending);}
        @JavascriptInterface public void objectReadTask(){objectRead("/api/objects/task","window.objectReceiveTask",16384,objectTaskPending);}
        @JavascriptInterface public void objectSnapshotEye(int eye){if(eye==0||eye==1)objectRead("/api/objects/snapshot?eye="+eye,"window.objectReceiveSnapshot",2*1024*1024,objectSnapshotPending);}
        @JavascriptInterface public void objectSnapshotForView(int eye,String viewToken){if(eye < -1||eye>1||viewToken==null||!viewToken.matches("[1-9][0-9]{0,15}"))return;objectRead(eye<0?"/api/objects/snapshot":"/api/objects/snapshot?eye="+eye,"window.objectReceiveSnapshot",2*1024*1024,objectSnapshotPending,viewToken,eye);}
        @JavascriptInterface public void objectSnapshot(){objectRead("/api/objects/snapshot","window.objectReceiveSnapshot",2*1024*1024,objectSnapshotPending);}
        @JavascriptInterface public void objectAbortTaskIntent(){abortObjectPendingTask();}
        @JavascriptInterface public void objectTask(String action,String epoch,int trackId,String mode,double standoff){
            if(SemanticTaskService.owns()){if("cancel".equals(action))stopNow("OBJECT_STOP_VOICE");else js("window.objectTaskError("+JSONObject.quote(action)+",\"当前语音任务正在执行；先明确手动选择后再切换，旧回调未发送\")");return;}
            if("cancel".equals(action))cancelOldControl();
            if(!linked||!foreground||!focused||closed||objectTaskWorker==null||!objectTaskRequestPending.compareAndSet(false,true)){js("window.objectTaskError("+JSONObject.quote(action)+",\"手机任务发送暂未就绪或上一笔仍在发送；本次没有发送\")");return;}
            final long current=generation.get(),controlGeneration=controlEpoch.get(),intentEpoch=objectIntentEpoch.get();final String owner=session,payload; final long requestId=navigationSequence.incrementAndGet(); final ObjectTaskTicket ticket=new ObjectTaskTicket(current,controlGeneration,intentEpoch,owner);
            try{payload=ObjectTaskPayload.build(action,requestId,epoch,trackId,mode,standoff);}catch(Exception e){objectTaskRequestPending.set(false);js("window.objectTaskError("+JSONObject.quote(action)+","+JSONObject.quote(e.getMessage())+")");return;}
            if("cancel".equals(action)){commandMailbox.offer("stop","");requestControlDispatch();}
            try{objectTaskWorker.execute(()->{HttpURLConnection c=null;RequestDeadline total=null;boolean wireAttempt=false,definitiveRejection=false;try{
                SemanticTaskService.awaitRevocation();
                if(!ticket.current(generation.get(),controlEpoch.get(),objectIntentEpoch.get(),linked,foreground,closed,session))return;
                byte[] body=payload.getBytes(StandardCharsets.UTF_8);c=open("/api/objects/task/"+action);final HttpURLConnection request=c;total=new RequestDeadline(deadlines,request::disconnect,NetworkTiming.TASK_MS);
                objectTaskConnection=c;if(!ticket.current(generation.get(),controlEpoch.get(),objectIntentEpoch.get(),linked,foreground,closed,session))return;c.setRequestProperty("X-Rover-Session",owner);c.setRequestProperty("Content-Type","application/json");c.setRequestMethod("POST");c.setDoOutput(true);c.setFixedLengthStreamingMode(body.length);js("window.objectTaskSending("+JSONObject.quote(action)+","+JSONObject.quote(Long.toString(requestId))+")");wireAttempt=true;try(java.io.OutputStream out=c.getOutputStream()){out.write(body);}
                int status=c.getResponseCode();if(status!=200){definitiveRejection=true;String detail="";try{detail=new JSONObject(new String(read(c.getErrorStream(),4096),StandardCharsets.UTF_8)).optString("error","");}catch(Exception ignored){}throw new Exception("任务未接受 HTTP "+status+" · "+detail);}
                JSONObject ack=new JSONObject(new String(read(c.getInputStream(),4096),StandardCharsets.UTF_8));total.check();String id=ack.optString("request_id","");if(!ack.optBoolean("queued")||!Long.toString(requestId).equals(id))throw new Exception("任务排队回执编号不一致，查询本次原编号结果");
                if(current==generation.get()&&controlGeneration==controlEpoch.get()&&intentEpoch==objectIntentEpoch.get()&&!closed&&owner.equals(session))js("window.objectTaskQueued("+JSONObject.quote(action)+","+JSONObject.quote(id)+")");
            }catch(Exception e){if(current==generation.get()&&controlGeneration==controlEpoch.get()&&intentEpoch==objectIntentEpoch.get()&&!closed){String message=e.getMessage()==null?"任务响应未完成":e.getMessage();if(wireAttempt&&!definitiveRejection&&(action.equals("start")||action.equals("resume")))js("window.objectTaskUncertain("+JSONObject.quote(action)+","+JSONObject.quote(Long.toString(requestId))+","+JSONObject.quote(message)+")");else js("window.objectTaskError("+JSONObject.quote(action)+","+JSONObject.quote(message)+")");}}finally{if(total!=null)total.close();if(c!=null)c.disconnect();if(objectTaskConnection==c)objectTaskConnection=null;if(intentEpoch==objectIntentEpoch.get())objectTaskRequestPending.set(false);}});}catch(java.util.concurrent.RejectedExecutionException e){objectTaskRequestPending.set(false);js("window.objectTaskError("+JSONObject.quote(action)+",\"任务队列繁忙，本次未发送；请稍候重新双击\")");}
        }
        @JavascriptInterface public void objectRoi(String epoch,String snapshot,int eye,int x1,int y1,int x2,int y2){
            final String payload;try{payload=ObjectTaskPayload.roi(objectSequence.incrementAndGet(),epoch,snapshot,eye,x1,y1,x2,y2);}catch(Exception e){js("window.objectReceiveError(\"select\","+JSONObject.quote(e.getMessage())+")");return;}
            if(!linked||!foreground||!focused||closed||!objectSelectionPending.compareAndSet(false,true))return;
            final long current=generation.get();final String owner=session;try{objectWorker.execute(()->{HttpURLConnection c=null;RequestDeadline total=null;try{
                if(current!=generation.get()||!linked||closed||!owner.equals(session))return;byte[] body=payload.getBytes(StandardCharsets.UTF_8);c=open("/api/objects/roi");final HttpURLConnection request=c;total=new RequestDeadline(deadlines,request::disconnect,NetworkTiming.TOTAL_MS);c.setRequestProperty("X-Rover-Session",owner);c.setRequestProperty("Content-Type","application/json");c.setRequestMethod("POST");c.setDoOutput(true);c.setFixedLengthStreamingMode(body.length);try(java.io.OutputStream out=c.getOutputStream()){out.write(body);}
                if(c.getResponseCode()!=200)throw new Exception("框选未接受；请更新图像后重新框选");JSONObject ack=new JSONObject(new String(read(c.getInputStream(),4096),StandardCharsets.UTF_8));total.check();String id=ack.optString("request_id","");if(!ack.optBoolean("queued")||!id.matches("[1-9][0-9]{0,19}"))throw new Exception("框选回执格式错误");if(current==generation.get()&&owner.equals(session))js("window.objectSelectionQueued(\"roi\","+JSONObject.quote(id)+")");
            }catch(Exception e){if(current==generation.get()&&!closed)js("window.objectReceiveError(\"select\","+JSONObject.quote(e.getMessage())+")");}finally{if(total!=null)total.close();if(c!=null)c.disconnect();objectSelectionPending.set(false);}});}catch(java.util.concurrent.RejectedExecutionException e){objectSelectionPending.set(false);}
        }
        @JavascriptInterface public void objectSelect(String epoch,String snapshotId,int trackId){objectSubmit("select",epoch,snapshotId,trackId);}
        @JavascriptInterface public void objectClear(){objectSubmit("clear",null,null,0);}
        private void objectSubmit(String operation,String epoch,String snapshotId,int trackId){
            if(!linked||!foreground||!focused||closed||objectWorker==null||!objectSelectionPending.compareAndSet(false,true))return;
            final long current=generation.get();final String objectSession=session;final String payload;
            try{payload=ObjectPayload.build(operation,objectSequence.incrementAndGet(),epoch,snapshotId,trackId);}
            catch(IllegalArgumentException e){objectSelectionPending.set(false);js("window.objectReceiveError(\"select\","+JSONObject.quote(e.getMessage())+")");return;}
            try{objectWorker.execute(()->{HttpURLConnection c=null;RequestDeadline total=null;try{
                SemanticTaskService.awaitRevocation();
                if(current!=generation.get()||!linked||!foreground||!focused||closed||!objectSession.equals(session))return;
                byte[] body=payload.getBytes(StandardCharsets.UTF_8);c=open("/api/objects/"+operation);total=new RequestDeadline(deadlines,c::disconnect,NetworkTiming.TOTAL_MS);c.setRequestProperty("X-Rover-Session",objectSession);c.setRequestProperty("Content-Type","application/json");c.setRequestMethod("POST");c.setDoOutput(true);c.setFixedLengthStreamingMode(body.length);
                try(java.io.OutputStream out=c.getOutputStream()){out.write(body);}
                if(c.getResponseCode()!=200)throw new Exception("实物选择未接受（HTTP "+c.getResponseCode()+"），请更新抓拍");
                JSONObject ack=new JSONObject(new String(read(c.getInputStream(),2048),StandardCharsets.UTF_8));String request=ack.optString("request_id","");
                if(!ack.optBoolean("queued",false)||!request.matches("[1-9][0-9]{0,19}"))throw new Exception("实物选择回执无效");
                if(current==generation.get()&&linked&&!closed&&objectSession.equals(session))js("window.objectSelectionQueued("+JSONObject.quote(operation)+","+JSONObject.quote(request)+")");
            }catch(Exception e){if(current==generation.get()&&!closed)js("window.objectReceiveError(\"select\","+JSONObject.quote(e.getMessage()==null?"实物选择失败":e.getMessage())+")");}
            finally{if(total!=null)total.close();if(c!=null)c.disconnect();objectSelectionPending.set(false);}});}catch(java.util.concurrent.RejectedExecutionException ignored){objectSelectionPending.set(false);}
        }
        private final AtomicBoolean visionPending=new AtomicBoolean(false);
        @JavascriptInterface public void navReadTags(){
            if(!linked||!foreground||!focused||closed||mapWorker==null||!visionPending.compareAndSet(false,true))return;
            final long current=generation.get();final String navSession=session;
            try{mapWorker.execute(()->{HttpURLConnection c=null;RequestDeadline total=null;try{
                if(current!=generation.get()||closed||!linked)return;
                c=open("/api/vision/tags",700);final HttpURLConnection request=c;total=new RequestDeadline(deadlines,request::disconnect,700);c.setRequestProperty("X-Rover-Session",navSession);
                if(c.getResponseCode()!=200)throw new Exception("标记服务未就绪");
                JSONObject data=new JSONObject(new String(read(c.getInputStream(),32768),StandardCharsets.UTF_8));total.check();
                if(data.optInt("version")!=1)throw new Exception("标记数据版本不兼容");
                if(current==generation.get()&&linked&&foreground&&!closed)js("window.navReceiveTags("+data.toString()+")");
            }catch(Exception e){if(current==generation.get()&&!closed)js("window.navTagsError("+JSONObject.quote(e.getMessage()==null?"标记读取失败":e.getMessage())+")");}
            finally{if(total!=null)total.close();if(c!=null)c.disconnect();visionPending.set(false);}});}catch(java.util.concurrent.RejectedExecutionException ignored){visionPending.set(false);}
        }
        @JavascriptInterface public void navPreviewTag(String mapId,String version,int tagId,double standoff){
            if(tagId<0||tagId>586||!Double.isFinite(standoff)||standoff<0.2||standoff>3.0)return;
            navSubmit("tag",mapId,version,0,0,tagId,standoff);
        }
        @JavascriptInterface public void navReadMap(){
            if(!linked||!foreground||!focused||closed||mapWorker==null||!mapPending.compareAndSet(false,true))return;
            final long current=generation.get();final String navSession=session;
            try{mapWorker.execute(()->{HttpURLConnection c=null;RequestDeadline total=null;try{
                long mapBegan=now();c=open("/api/nav/map",NetworkTiming.MAP_MS);final HttpURLConnection request=c;total=new RequestDeadline(deadlines,request::disconnect,NetworkTiming.MAP_MS);c.setRequestProperty("X-Rover-Session",navSession);
                checkNavigationResponse(c,"map");
                String text=new String(read(c.getInputStream(),2*1024*1024),StandardCharsets.UTF_8);
                JSONObject snapshot=new JSONObject(text);total.check();
                long mapAge=c.getHeaderFieldLong("X-Map-Age-Ms",-1),fileAge=c.getHeaderFieldLong("X-Map-File-Age-Ms",-1);snapshot.put("transport_age_ms",mapAge>=0?mapAge:JSONObject.NULL);snapshot.put("transport_age_known",mapAge>=0);snapshot.put("map_file_age_ms",fileAge>=0?fileAge:JSONObject.NULL);snapshot.put("map_client_delay_ms",Math.max(0,now()-mapBegan));String refresh=c.getHeaderField("X-Map-Refresh-Warning");snapshot.put("map_refresh_state",refresh==null?"UNKNOWN":refresh);snapshot.put("map_publication_age_known","true".equals(c.getHeaderField("X-Map-Publication-Age-Known")));
                if(snapshot.optInt("schema")!=1)throw new NavigationFailure("map","MAP_SCHEMA_INVALID",0,"地图版本不兼容");
                if(current==generation.get()&&linked&&foreground&&!closed)js("window.navReceiveMap("+snapshot.toString()+")");
            }catch(Exception e){if(current==generation.get()&&!closed){if(isSessionExpired(e))reconnectSession("地图查看会话过期",current);else reportNavigationFailure("map",e);}}
            finally{if(total!=null)total.close();if(c!=null)c.disconnect();mapPending.set(false);}});}catch(java.util.concurrent.RejectedExecutionException ignored){mapPending.set(false);}
        }
        @JavascriptInterface public void navReadMotion(){
            if(!linked||!foreground||!focused||closed||motionWorker==null||!motionPending.compareAndSet(false,true))return;
            final long current=generation.get();final String owner=session;
            try{motionWorker.execute(()->{HttpURLConnection c=null;RequestDeadline total=null;try{
                if(current!=generation.get()||!linked||closed||!owner.equals(session))return;
                c=open("/api/nav/motion",NetworkTiming.UI_MS);final HttpURLConnection request=c;total=new RequestDeadline(deadlines,request::disconnect,NetworkTiming.UI_MS);c.setRequestProperty("X-Rover-Session",owner);
                checkNavigationResponse(c,"motion");JSONObject data=new JSONObject(new String(read(c.getInputStream(),16384),StandardCharsets.UTF_8));total.check();
                data.put("transport_age_ms",Math.max(0,c.getHeaderFieldLong("X-Motion-Age-Ms",0)));
                if(current==generation.get()&&linked&&!closed)js("window.navReceiveMotion("+data.toString()+")");
            }catch(Exception error){if(current==generation.get()&&!closed){if(isSessionExpired(error))reconnectSession("运动查看会话过期",current);else js("window.navMotionError("+JSONObject.quote(error.getMessage()==null?"预测轨迹等待更新":error.getMessage())+")");}}
            finally{if(total!=null)total.close();if(c!=null)c.disconnect();motionPending.set(false);}});}catch(java.util.concurrent.RejectedExecutionException ignored){motionPending.set(false);}
        }
        @JavascriptInterface public void navPreviewGoal(String mapId,String version,double x,double y){
            navSubmit("goal",mapId,version,x,y);
        }
        @JavascriptInterface public void navStart(String mapId,String version,double x,double y){
            navSubmit("start",mapId,version,x,y);
        }
        @JavascriptInterface public void navStop(String mapId,String version,double x,double y){
            commandMailbox.offer("stop","");requestControlDispatch();
            navSubmit("stop",mapId,version,x,y);
        }
        @JavascriptInterface public void navResetMap(String mapId,String version){
            commandMailbox.offer("stop","");requestControlDispatch();
            navSubmit("reset",mapId,version,0,0);
        }
        private void navSubmit(String action,String mapId,String version,double x,double y){
            navSubmit(action,mapId,version,x,y,-1,0.5);
        }
        private void navSubmit(String action,String mapId,String version,double x,double y,int tagId,double standoff){
            if(!linked||!foreground||!focused||closed||navigationWorker==null){reportNavigationFailure(action,new NavigationFailure(action,"CLIENT_NOT_READY",0,"手机连接或前台状态尚未就绪，请重新连接后重试"));return;}
            if(mapId==null||version==null||!mapId.matches("[0-9]{1,20}")||!version.matches("[0-9]{1,20}")||!Double.isFinite(x)||!Double.isFinite(y)||Math.abs(x)>100000||Math.abs(y)>100000){reportNavigationFailure(action,new NavigationFailure(action,"REQUEST_INVALID",0,"地图标识或目标坐标无效，请重新选点"));return;}
            if(SemanticTaskService.owns())SemanticTaskService.cancel("地图导航接管");
            if(!navigationRequestPending.compareAndSet(false,true)){reportNavigationFailure(action,new NavigationFailure(action,"CLIENT_REQUEST_BUSY",0,"上一笔导航请求仍在发送，请稍候再点；本次没有发送"));return;}
            final long current=generation.get();final String navSession=session;
            final long requestSequence=navigationSequence.incrementAndGet();
            final long requestSentUs=serverTime(now())*1000,requestEpoch=controlEpoch.get();
            js("window.navGoalSending("+JSONObject.quote(action)+","+JSONObject.quote(Long.toString(requestSequence))+")");
            try{navigationWorker.execute(()->{HttpURLConnection c=null;RequestDeadline total=null;try{
                SemanticTaskService.awaitRevocation();
                if(current!=generation.get()||!linked||!foreground||closed||!navSession.equals(session)||(!action.equals("stop")&&requestEpoch!=controlEpoch.get())){if(current==generation.get()&&!closed)reportNavigationFailure(action,new NavigationFailure(action,"REQUEST_CANCELLED_BY_CONTROL",0,"请求排队期间发生停止、手动接管或会话切换，本次请求未发送"));return;}
                byte[] body=new JSONObject(NavigationPayload.build(action,mapId,version,x,y,tagId,standoff,requestSequence)).put("sent_us",requestSentUs).toString().getBytes(StandardCharsets.UTF_8);c=open("/api/nav/"+action,1200);final HttpURLConnection request=c;total=new RequestDeadline(deadlines,request::disconnect,1200);c.setRequestProperty("X-Rover-Session",navSession);c.setRequestProperty("Content-Type","application/json");c.setRequestMethod("POST");c.setDoOutput(true);c.setFixedLengthStreamingMode(body.length);
                try(java.io.OutputStream out=c.getOutputStream()){out.write(body);}
                checkNavigationResponse(c,action);total.check();
                JSONObject ack=new JSONObject(new String(read(c.getInputStream(),4096),StandardCharsets.UTF_8));total.check();
                String requestId=ack.optString("request_id",""),visionRequest=ack.optString("vision_request_us","");
                if(!ack.optBoolean("queued",false)||!Long.toString(requestSequence).equals(requestId))throw new NavigationFailure(action,"RESPONSE_INVALID",0,"导航请求回执编号不匹配，请更新配套RK程序");
                if("tag".equals(action)&&!visionRequest.matches("[0-9]{1,20}"))throw new NavigationFailure(action,"RESPONSE_INVALID",0,"标记目标确认格式不兼容，请使用配套RK程序");
                if(current==generation.get()&&linked&&!closed)js("window.navGoalQueued("+JSONObject.quote(action)+","+JSONObject.quote(requestId)+","+JSONObject.quote(visionRequest)+")");
            }catch(Exception e){if(current==generation.get()&&!closed)reportNavigationFailure(action,e);}
            finally{if(total!=null)total.close();if(c!=null)c.disconnect();navigationRequestPending.set(false);}});}catch(java.util.concurrent.RejectedExecutionException ignored){navigationRequestPending.set(false);reportNavigationFailure(action,new NavigationFailure(action,"CLIENT_NOT_READY",0,"导航发送线程已关闭，请重新连接后重试"));}
        }
        @JavascriptInterface public void reportUiError(String detail) { android.util.Log.e("ACG138_UI_DIAG", detail == null ? "unknown" : detail.substring(0, Math.min(detail.length(), 1200))); }
        @JavascriptInterface public String getConfig() {
            JSONObject j = new JSONObject();
            try { j.put("host", preferences.getString("host", "")); j.put("password", ""); } catch(Exception ignored){}
            return j.toString();
        }
        @JavascriptInterface public void videoLayout(double x,double y,double width,double height,double viewportWidth,double viewportHeight,String view){
            if(!Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(width)||!Double.isFinite(height)||
               !Double.isFinite(viewportWidth)||!Double.isFinite(viewportHeight)||width<=0||height<=0||viewportWidth<=0||viewportHeight<=0||
               x<0||y<0||x+width>viewportWidth+1||y+height>viewportHeight+1||
               !("stereo".equals(view)||"left".equals(view)||"right".equals(view)))return;
            runOnUiThread(()->{
                if(closed||web==null||videoTexture==null)return;
                float sx=web.getWidth()/(float)viewportWidth,sy=web.getHeight()/(float)viewportHeight;
                FrameLayout.LayoutParams params=new FrameLayout.LayoutParams(Math.max(1,(int)Math.round(width*sx)),Math.max(1,(int)Math.round(height*sy)));
                params.leftMargin=(int)Math.round(x*sx);params.topMargin=(int)Math.round(y*sy);
                videoView=view;videoTexture.setLayoutParams(params);updateVideoTransform();
            });
        }
        @JavascriptInterface public void renewHold(String motion){if(foreground&&linked)commandMailbox.renewHold(motion,now());}
        @JavascriptInterface public void pulse() { if (foreground && focused) uiPulse = now(); }
        @JavascriptInterface public void command(String op, String motion) {
            commandWithOrigin(op,motion,"UI_"+op.toUpperCase(java.util.Locale.ROOT));
        }
        @JavascriptInterface public void commandWithOrigin(String op,String motion,String source) {
            if(!op.equals("stop") && !foreground)return;
            if (!(op.equals("stop") || op.equals("manual") || op.equals("auto") || op.equals("hold"))) return;
            if (op.equals("hold") && !(motion.equals("forward") || motion.equals("reverse") || motion.equals("curve_left") ||
                motion.equals("curve_right") || motion.equals("spin_left") || motion.equals("spin_right") ||
                motion.equals("back_left") || motion.equals("back_right"))) return;
            if(op.equals("stop")){stopNow(CommandMailbox.sanitizeOrigin(source));return;}
            if(op.equals("auto")){startAutomatic();return;}
            if(op.equals("manual")||op.equals("hold")){SemanticTaskService.cancel("用户手动接管");cancelOldControl();}
            if(commandMailbox.offer(op,motion,now(),source))requestControlDispatch();
        }
        @JavascriptInterface public void connect(String host, String code) {
            if(closed||!foreground||host==null||code==null||(!host.isEmpty()&&!validHost(host))||!code.matches("[A-Za-z0-9_-]{6,64}"))return;
            beginConnection(host,code);
        }
        @JavascriptInterface public void cancelConnect(){disconnect();}
        @JavascriptInterface public void disconnect() {
            connectionIntent.incrementAndGet();
            stopNow("EXPLICIT_DISCONNECT"); commands.execute(() -> {
                try { postControl("stop", "", generation.get(),0,controlEpoch.get(),"EXPLICIT_DISCONNECT"); } catch(Exception ignored){}
                linked = false; generation.incrementAndGet(); session="";
                if(stateConnection!=null)stateConnection.disconnect();videoPlayer.cancel();
            });
        }
    }
    @Override protected void onResume() { super.onResume(); foreground=true; uiPulse=now(); if(web!=null)web.onResume(); }
    @Override protected void onPause() { if(speech!=null)speech.cancel();foreground=false;videoPlayer.cancel();connectionIntent.incrementAndGet();if(!SemanticTaskService.owns())stopNow("LIFECYCLE_PAUSE");js("window.appPaused()");if(web!=null)web.onPause();super.onPause(); }
    @Override public void onWindowFocusChanged(boolean focus) { super.onWindowFocusChanged(focus);focused=focus;/* 窗口焦点变化不等于控制断开或松开输入。 */ }
    @Override public void onBackPressed() { if(SemanticTaskService.owns()){moveTaskToBack(true);return;} stopNow("SYSTEM_BACK");super.onBackPressed(); }
    @Override public void onRequestPermissionsResult(int request,String[] permissions,int[] results){super.onRequestPermissionsResult(request,permissions,results);if(request==261){boolean granted=results.length>0&&results[0]==android.content.pm.PackageManager.PERMISSION_GRANTED;voiceEvent(granted?"permission_granted":"error",granted?"麦克风已授权。授权不会自动录音；请再次点击说话开始识别。":"麦克风权限未授予。请用文字输入，或在系统设置 → 应用 → 权限中允许麦克风后重试。",null,-1);}}
    @Override protected void onDestroy() {
        if(speech!=null)speech.destroy();
        foreground=false;if(!SemanticTaskService.owns())stopNow("LIFECYCLE_DESTROY");linked=false;closed=true;connectionIntent.incrementAndGet();
        if(stateConnection!=null)stateConnection.disconnect();videoPlayer.cancel();if(commandConnection!=null)commandConnection.disconnect();
        if(controlIO!=null)controlIO.shutdownNow();if(stopIO!=null)stopIO.shutdownNow();if(heartbeat!=null)heartbeat.shutdownNow();if(commands!=null)commands.shutdownNow();if(telemetry!=null)telemetry.shutdownNow();if(video!=null)video.shutdownNow();if(connections!=null)connections.shutdownNow();if(mapWorker!=null)mapWorker.shutdownNow();if(navigationWorker!=null)navigationWorker.shutdownNow();if(deadlines!=null)deadlines.shutdownNow();
        if(objectImageWorker!=null)objectImageWorker.shutdownNow();if(objectStateWorker!=null)objectStateWorker.shutdownNow();if(objectTaskWorker!=null)objectTaskWorker.shutdownNow();if(objectWorker!=null)objectWorker.shutdownNow();if(motionWorker!=null)motionWorker.shutdownNow();
        if(web!=null){web.removeJavascriptInterface("Rover");web.destroy();}super.onDestroy();
    }
}
