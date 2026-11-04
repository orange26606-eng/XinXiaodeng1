package cn.acg138.rover;
import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.media.AudioFormat;
import android.media.AudioRecord;
import android.media.MediaRecorder;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;
import com.k2fsa.sherpa.onnx.OnlineStream;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;
/** 单句离线语音识别，界面更新与最终动作在主线程执行。 */
final class TapSpeech {
 interface Sink{void event(String state,String message,String text,float rms);void finalText(String text);}
 private final Activity activity;private final Sink sink;private final Handler main=new Handler(Looper.getMainLooper());
 private final ThreadPoolExecutor worker=new ThreadPoolExecutor(1,1,0,TimeUnit.MILLISECONDS,new ArrayBlockingQueue<Runnable>(1));
 private final AtomicLong generation=new AtomicLong();private final AtomicReference<AudioRecord> microphone=new AtomicReference<>();
 private volatile boolean busy,closed;private SherpaChineseEngine engine;private Runnable watchdog;
 TapSpeech(Activity a,Sink s){activity=a;sink=s;}
 private void event(String state,String msg){sink.event(state,msg,null,-1);}
 private void post(long token,String state,String msg,String text,float rms){main.post(()->{if(!closed&&generation.get()==token)sink.event(state,msg,text,rms);});}
 private boolean current(long token){return !closed&&generation.get()==token;}
 void text(String text){cancelInternal();sink.finalText(text);}
 void start(boolean ignoredNetworkChoice){
  if(closed)return;if(busy){event("listening","上一笔离线识别仍在处理；可点取消后重试。");return;}
  event("preparing","已收到点击，使用App内置Sherpa中文模型；音频不上传。");
  if(activity.checkSelfPermission(Manifest.permission.RECORD_AUDIO)!=PackageManager.PERMISSION_GRANTED){event("permission","需要麦克风权限；授权后请再次点击说话。");activity.requestPermissions(new String[]{Manifest.permission.RECORD_AUDIO},261);return;}
  final long token=generation.incrementAndGet();busy=true;worker.getQueue().clear();
  watchdog=()->{if(current(token)&&busy){cancelInternal();event("error","离线识别准备或录音超时，已停止麦克风；请重试并查看ACGOfflineVoice日志。");}};main.postDelayed(watchdog,45000);
  try{worker.execute(()->recognize(token));}catch(RejectedExecutionException e){cancelInternal();event("error","离线识别线程正在退出；请返回页面后重试。");}
 }
 private void recognize(long token){AudioRecord audio=null;OnlineStream stream=null;long begun=SystemClock.elapsedRealtime();
  try{
   if(!current(token))return;
   if(engine==null){post(token,"loading","正在校验并加载内置中文模型，首次启动稍慢；此时尚未录音。",null,-1);long t=SystemClock.elapsedRealtime();engine=new SherpaChineseEngine(activity.getApplicationContext());Log.i("ACGOfflineVoice","model=zh-small-ctc-int8 runtime=2.0.1 loaded_ms="+(SystemClock.elapsedRealtime()-t));}
   if(!current(token))return;int min=AudioRecord.getMinBufferSize(16000,AudioFormat.CHANNEL_IN_MONO,AudioFormat.ENCODING_PCM_16BIT);if(min<=0)throw new IllegalStateException("设备不支持16kHz单声道录音");
   audio=new AudioRecord(MediaRecorder.AudioSource.VOICE_RECOGNITION,16000,AudioFormat.CHANNEL_IN_MONO,AudioFormat.ENCODING_PCM_16BIT,Math.max(min,12800));if(audio.getState()!=AudioRecord.STATE_INITIALIZED)throw new IllegalStateException("麦克风初始化失败");
   microphone.set(audio);if(!current(token))return;stream=engine.stream();audio.startRecording();begun=SystemClock.elapsedRealtime();final long started=begun;
   main.post(()->{if(current(token)){if(watchdog!=null)main.removeCallbacks(watchdog);watchdog=()->{if(current(token)&&busy){cancelInternal();event("error","录音设备超过12秒仍未完成，已停止本次识别；请重试。");}};main.postDelayed(watchdog,12000);}});
   post(token,"ready","麦克风已启动，请说一句命令；停顿约1秒自动结束，最长10秒。",null,-1);
   short[]pcm=new short[1600];SpeechTurn turn=new SpeechTurn();String previous="";String finalText=null;int ending=SpeechTurn.CONTINUE;
   while(current(token)){
    int n=audio.read(pcm,0,pcm.length,AudioRecord.READ_BLOCKING);if(!current(token))return;if(n<0)throw new IllegalStateException("录音读取失败 code="+n);if(n==0)continue;
    double squares=0;float[]samples=new float[n];for(int i=0;i<n;i++){samples[i]=pcm[i]/32768f;squares+=samples[i]*samples[i];}double rms=Math.sqrt(squares/n);float db=(float)(20*Math.log10(Math.max(.000001,rms)));post(token,"rms","",null,db);
    String text=engine.accept(stream,samples);if(!text.equals(previous)){previous=text;post(token,"partial","实时文字；说完后才提交最终命令。",text,-1);}
    ending=turn.update(text,engine.endpoint(stream),SystemClock.elapsedRealtime()-started);
    if(ending!=SpeechTurn.CONTINUE){finalText=ending==SpeechTurn.EMPTY?"":ending==SpeechTurn.ENDPOINT?turn.text():engine.finish(stream);break;}
   }
   if(!current(token))return;final String text=finalText==null?"":finalText.trim();final long elapsed=SystemClock.elapsedRealtime()-started;
   Log.i("ACGOfflineVoice","session="+token+" end="+ending+" record_ms="+elapsed+" chars="+text.length()+" network_audio=NONE");
   /* 最终语句触发已有动作之前，先释放麦克风。 */
   closeAudio(audio);microphone.compareAndSet(audio,null);audio=null;stream.release();stream=null;
   main.post(()->{if(!current(token))return;busy=false;clearWatchdog();if(text.isEmpty())event("error","本次没有识别到文字；可查看音量条、靠近麦克风重试，或输入文字。");else{sink.event("recognizing","录音已结束，正在理解最终文字。",text,-1);sink.finalText(text);}});
  }catch(Throwable e){Log.e("ACGOfflineVoice","session="+token+" failure="+e.getClass().getSimpleName()+" detail="+e.getMessage(),e);final String detail=error(e);main.post(()->{if(current(token)){busy=false;clearWatchdog();event("error",detail);}});}
  finally{if(audio!=null){microphone.compareAndSet(audio,null);closeAudio(audio);}if(stream!=null)try{stream.release();}catch(Throwable ignored){}}
 }
 private static String error(Throwable e){if(e instanceof UnsatisfiedLinkError)return "Sherpa ARM64运行库加载失败："+e.getMessage();if(e instanceof SecurityException)return "麦克风权限被拒绝或已撤销，请在应用权限中允许录音。";return "离线语音失败（"+e.getClass().getSimpleName()+"）："+(e.getMessage()==null?"请查看ACGOfflineVoice日志":e.getMessage());}
 private static void closeAudio(AudioRecord a){if(a==null)return;try{if(a.getRecordingState()==AudioRecord.RECORDSTATE_RECORDING)a.stop();}catch(RuntimeException ignored){}try{a.release();}catch(RuntimeException ignored){}}
 private void clearWatchdog(){if(watchdog!=null){main.removeCallbacks(watchdog);watchdog=null;}}
 private void cancelInternal(){generation.incrementAndGet();busy=false;clearWatchdog();worker.getQueue().clear();AudioRecord a=microphone.get();if(a!=null)try{a.stop();}catch(RuntimeException ignored){}}
 void cancel(){boolean was=busy;cancelInternal();if(was)event("cancelled","已取消本次识别，麦克风停止；不会提交已取消的文字。");}
 void destroy(){closed=true;cancelInternal();try{worker.execute(()->{if(engine!=null){engine.close();engine=null;}});}catch(RejectedExecutionException ignored){}worker.shutdown();}
}
