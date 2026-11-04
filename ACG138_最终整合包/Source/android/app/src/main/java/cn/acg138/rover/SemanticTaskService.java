package cn.acg138.rover;
import android.app.*;
import android.content.*;
import android.content.pm.ServiceInfo;
import android.os.*;
import org.json.*;
import java.io.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;
/** 语义任务持有期间只有一个控制租约写入者，不持有麦克风。 */
public final class SemanticTaskService extends Service {
 public static final Object WIRE=new Object();
 private static final AtomicLong ids=new AtomicLong(System.currentTimeMillis()*1000);
 private static final AtomicLong revision=new AtomicLong();
 private static volatile Config cfg;
 private static volatile String result="{}",clientMessage="尚无持续任务";
 private static volatile boolean owned;
 private static volatile long seenAt,submitAt,lastHeartbeatAck;
 public static long lastHeartbeatAck(){return lastHeartbeatAck;}
 private static volatile String acknowledged="";
 private static volatile Future<?> revocation;
 public static void awaitRevocation(){Future<?> f=revocation;if(f!=null)try{f.get(2200,TimeUnit.MILLISECONDS);}catch(Exception ignored){}}
 public static void awaitRevocationForAuto() throws Exception {
  Future<?> pending = revocation;
  if (pending == null) return;
  try {
   pending.get(2200, TimeUnit.MILLISECONDS);
  } catch (TimeoutException error) {
   throw new IOException("原任务取消尚未完成，请重试自动模式", error);
  }
 }
 private ScheduledExecutorService heartbeat,reader;
 private ExecutorService writer;
 private PowerManager.WakeLock wake;private long wakeRenewed;
 private synchronized void awake(){if(wake!=null&&owns()&&(!wake.isHeld()||SystemClock.elapsedRealtime()-wakeRenewed>300000)){wake.acquire(600000);wakeRenewed=SystemClock.elapsedRealtime();}}
 private synchronized void sleep(){if(wake!=null&&wake.isHeld())wake.release();}
 private static volatile SemanticTaskService running;
 static final class Config {final String base,token,session,id,payload,slots;final AtomicLong controlSeq,navSeq;final long revision;
 Config(String b,String t,String s,String i,String p,String meta,AtomicLong c,AtomicLong n,long v){base=b;token=t;session=s;id=i;payload=p;slots=meta;controlSeq=c;navSeq=n;revision=v;}}
 public static boolean owns(){return owned&&cfg!=null;}
 public static String id(){Config c=cfg;return c==null?"":c.id;}
 public static String snapshot(){try{JSONObject j=new JSONObject(result);Config c=cfg;j.put("client_owned",owns()).put("client_message",clientMessage).put("client_age_ms",seenAt==0?-1:SystemClock.elapsedRealtime()-seenAt);if(c!=null){j.put("desired_task_id",c.id).put("spoken",new JSONObject(c.slots));}return j.toString();}catch(Exception e){return "{}";}}
 public static boolean attach(String b,String t){Config c=cfg;return owns()&&c.base.equals(b)&&c.token.equals(t);}
 public static String currentSession(){return cfg==null?"":cfg.session;}
 public static AtomicLong controlSequence(){return cfg.controlSeq;}
 public static AtomicLong navigationSequence(){return cfg.navSeq;}
 public static void submit(Context context,String base,String token,String session,AtomicLong controlSeq,AtomicLong navSeq,String intentText)throws Exception{
 JSONObject i=new JSONObject(intentText);String action=i.getString("action"),mode=action.equals("go")?"GO":action.equals("follow")?"FOLLOW":action.equals("selectPreview")?"PREVIEW":action.equals("find")?"FIND":action.equals("select")?"SELECT":"";
 if(mode.isEmpty())throw new Exception("不支持的持续任务动作");String id=Long.toString(ids.updateAndGet(old->Math.max(old+1,System.currentTimeMillis()*1000)));
 JSONObject p=new JSONObject().put("task_id",id).put("mode",mode).put("category",i.optString("category","")).put("color",i.optString("color","")).put("size",i.optString("size","")).put("position",i.optString("position","")).put("ordinal",i.optInt("ordinal",0)).put("reference",i.optBoolean("reference",false)).put("distance_m",i.optDouble("distance_m",.2));
 synchronized(WIRE){long v=revision.incrementAndGet();cfg=new Config(base,token,session,id,p.toString(),i.toString(),controlSeq,navSeq,v);owned=true;acknowledged="";result="{}";seenAt=submitAt=0;clientMessage="任务正在提交RK；没有目标框也会先接受任务";}
 Intent service=new Intent(context,SemanticTaskService.class);try{context.startForegroundService(service);}catch(RuntimeException e){synchronized(WIRE){owned=false;revision.incrementAndGet();}throw new Exception("系统未允许前台连接服务："+e.getMessage());}
 }
 public static void cancel(String reason){Config c;SemanticTaskService s;
 synchronized(WIRE){if(!owned)return;c=cfg;owned=false;revision.incrementAndGet();clientMessage=reason;s=running;}
 if(c!=null&&s!=null){try{revocation=s.writer.submit(()->{try{JSONObject b=new JSONObject().put("seq",c.navSeq.incrementAndGet()).put("task_id",c.id);request(c,"/api/semantic/cancel",b,600);}catch(Exception ignored){}try{synchronized(WIRE){if(cfg==c&&!owned)request(c,"/api/control",new JSONObject().put("session",c.session).put("seq",c.controlSeq.incrementAndGet()).put("op","stop").put("motion",""),280);}}catch(Exception ignored){}if(!owns())s.idle();});}catch(RejectedExecutionException ignored){}}
 }
 public static void startPrepared(){Config c=cfg;SemanticTaskService s=running;if(!owns()||c==null||s==null)return;s.writer.execute(()->{if(!current(c))return;try{request(c,"/api/semantic/start",new JSONObject().put("seq",c.navSeq.incrementAndGet()).put("task_id",c.id),NetworkTiming.TASK_MS);clientMessage="明确开始已提交，等待RK确认";}catch(Exception e){clientMessage="开始尚未接受："+e.getMessage();}});}
 private static boolean current(Config c){return owned&&cfg==c&&revision.get()==c.revision;}
 private static JSONObject request(Config c,String path,JSONObject body,int timeout)throws Exception{
 HttpURLConnection h=(HttpURLConnection)new URL(c.base+path).openConnection();h.setConnectTimeout(timeout);h.setReadTimeout(timeout);h.setUseCaches(false);h.setInstanceFollowRedirects(false);h.setRequestProperty("Authorization","Bearer "+c.token);h.setRequestProperty("X-Rover-Session",c.session);h.setRequestProperty("Connection","close");
 try{if(body!=null){byte[] b=body.toString().getBytes(StandardCharsets.UTF_8);h.setRequestMethod("POST");h.setRequestProperty("Content-Type","application/json");h.setDoOutput(true);h.setFixedLengthStreamingMode(b.length);try(OutputStream o=h.getOutputStream()){o.write(b);}}
 int status=h.getResponseCode();InputStream in=status<400?h.getInputStream():h.getErrorStream();ByteArrayOutputStream out=new ByteArrayOutputStream();if(in!=null)try(InputStream stream=in){byte[] b=new byte[2048];int n;while((n=stream.read(b))!=-1){if(out.size()+n>32768)throw new IOException("状态响应超过32KB");out.write(b,0,n);}}
 JSONObject j=new JSONObject(out.toString("UTF-8"));if(status>=400)throw new IOException("HTTP "+status+"："+j.optString("error","请求失败"));return j;
 }finally{h.disconnect();}}
 @Override public void onCreate(){super.onCreate();running=this;PowerManager power=(PowerManager)getSystemService(POWER_SERVICE);wake=power.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK,"ACG138:semantic-connection");wake.setReferenceCounted(false);heartbeat=Executors.newSingleThreadScheduledExecutor();reader=Executors.newSingleThreadScheduledExecutor();writer=Executors.newSingleThreadExecutor();
 heartbeat.scheduleWithFixedDelay(()->{Config c=cfg;if(!current(c))return;awake();try{if(current(c)){JSONObject ack=request(c,"/api/heartbeat",new JSONObject().put("mode","control"),NetworkTiming.HEARTBEAT_MS);if(current(c)&&ack.optBoolean("control_live",false))lastHeartbeatAck=SystemClock.elapsedRealtime();}}catch(Exception e){if(current(c))clientMessage="原会话心跳重试；真实断链仍停车："+e.getMessage();}},0,200,TimeUnit.MILLISECONDS);
 reader.scheduleWithFixedDelay(()->{Config c=cfg;if(!current(c))return;try{JSONObject j=request(c,"/api/semantic/state",null,NetworkTiming.UI_MS);if(current(c)){result=j.toString();seenAt=SystemClock.elapsedRealtime();if(c.id.equals(j.optString("task_id"))){acknowledged=c.id;clientMessage="RK实际回报："+j.optString("message");if(!j.optBoolean("active",true)){synchronized(WIRE){if(current(c)){owned=false;idle();}}return;}}else acknowledged="";}}catch(Exception e){if(current(c))clientMessage="任务状态读取失败，任务仍保留："+e.getMessage();}
 if(current(c)&&!c.id.equals(acknowledged)&&SystemClock.elapsedRealtime()-submitAt>=2000){submitAt=SystemClock.elapsedRealtime();writer.execute(()->{if(!current(c))return;try{JSONObject p=new JSONObject(c.payload).put("seq",c.navSeq.incrementAndGet());request(c,"/api/semantic/task",p,NetworkTiming.TASK_MS);if(current(c)&&seenAt==0)clientMessage="RK已排队接受；等待控制器回报";}catch(Exception e){if(current(c))clientMessage="提交未确认，保留相同任务编号重试："+e.getMessage();}});}
 },100,450,TimeUnit.MILLISECONDS);
 }
 @Override public int onStartCommand(Intent intent,int flags,int startId){if(intent!=null&&"STOP".equals(intent.getAction())){cancel("已从通知栏明确停车");return START_NOT_STICKY;}if(!owns()){stopSelf();return START_NOT_STICKY;}
 NotificationManager manager=(NotificationManager)getSystemService(NOTIFICATION_SERVICE);manager.createNotificationChannel(new NotificationChannel("rover-task","小车持续任务连接",NotificationManager.IMPORTANCE_LOW));
 PendingIntent stop=PendingIntent.getService(this,1,new Intent(this,SemanticTaskService.class).setAction("STOP"),PendingIntent.FLAG_IMMUTABLE|PendingIntent.FLAG_UPDATE_CURRENT);
 PendingIntent open=PendingIntent.getActivity(this,2,new Intent(this,MainActivity.class).addFlags(Intent.FLAG_ACTIVITY_SINGLE_TOP),PendingIntent.FLAG_IMMUTABLE|PendingIntent.FLAG_UPDATE_CURRENT);
 Notification n=new Notification.Builder(this,"rover-task").setSmallIcon(android.R.drawable.ic_media_play).setContentTitle("小车任务持续执行").setContentText("关闭页面不取消任务；点击停车可立即取消").setOngoing(true).setContentIntent(open).addAction(new Notification.Action.Builder(android.R.drawable.ic_media_pause,"停车",stop).build()).build();
 if(Build.VERSION.SDK_INT>=29)startForeground(128,n,ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE);else startForeground(128,n);awake();return START_NOT_STICKY;
 }
 private void idle(){sleep();stopForeground(STOP_FOREGROUND_REMOVE);if(reader!=null&&!reader.isShutdown())reader.schedule(()->{synchronized(WIRE){if(!owns())stopSelf();}},30,TimeUnit.SECONDS);}
 @Override public IBinder onBind(Intent intent){return null;}
 @Override public void onDestroy(){sleep();running=null;if(heartbeat!=null)heartbeat.shutdownNow();if(reader!=null)reader.shutdownNow();if(writer!=null)writer.shutdown();super.onDestroy();}
}
