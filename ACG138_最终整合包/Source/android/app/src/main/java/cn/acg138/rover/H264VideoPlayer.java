package cn.acg138.rover;

import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaCodecList;
import android.media.MediaFormat;
import android.os.Build;
import android.os.SystemClock;
import android.util.Log;
import android.view.Surface;
import java.io.BufferedInputStream;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

/* 独立二进制视频连接。 */
final class H264VideoPlayer {
    interface Active {boolean yes();}
    private volatile HttpURLConnection connection;
    private final AtomicLong epoch=new AtomicLong();
    private volatile long decodedFrames;
    void cancel(){epoch.incrementAndGet();HttpURLConnection c=connection;if(c!=null)c.disconnect();}
    long decodedFrames(){return decodedFrames;}
    private static MediaCodecInfo hardwareDecoder(MediaFormat format)throws IOException {
        for(MediaCodecInfo info:new MediaCodecList(MediaCodecList.REGULAR_CODECS).getCodecInfos()){
            if(info.isEncoder())continue;
            String name=info.getName().toLowerCase(java.util.Locale.ROOT);
            if(Build.VERSION.SDK_INT>=29){if(!info.isHardwareAccelerated())continue;}
            else if(name.startsWith("omx.google.")||name.startsWith("c2.android.")||name.contains("ffmpeg"))continue;
            try{if(info.getCapabilitiesForType("video/avc").isFormatSupported(format))return info;}catch(IllegalArgumentException ignored){}
        }
        throw new IOException("手机没有可用的 720p H.264 硬件解码器");
    }
    void play(String base,String password,String session,Surface surface,Active active)throws Exception {
        final long run=epoch.get();final AtomicBoolean outputStop=new AtomicBoolean();
        final AtomicBoolean outputFailed=new AtomicBoolean();Thread output=null;MediaCodec codec=null;
        HttpURLConnection c=(HttpURLConnection)new URL(base+"/api/video.h264").openConnection();connection=c;
        c.setConnectTimeout(1200);c.setReadTimeout(600);c.setInstanceFollowRedirects(false);c.setUseCaches(false);
        c.setRequestProperty("Authorization","Bearer "+password);c.setRequestProperty("X-Rover-Session",session);c.setRequestProperty("Connection","close");
        try{
            int status=c.getResponseCode();if(status!=200)throw new IOException(status==503?"请更新配套 H.264 板端运行包":"视频连接暂不可用（"+status+"）");
            String type=c.getContentType();if(type==null||!type.startsWith("application/x-acg138-h264"))throw new IOException("板端视频格式不匹配");
            BufferedInputStream in=new BufferedInputStream(c.getInputStream(),32768);
            long previous=-1;boolean waitingKey=true;
            while(run==epoch.get()&&active.yes()&&surface.isValid()){
                H264Wire.Frame frame=H264Wire.read(in);
                if(run!=epoch.get()||!active.yes())break;
                if(previous>=0&&!H264Wire.consecutive(previous,frame.sequence)&&!frame.key())throw new IOException("视频帧中断，重新获取关键帧");
                previous=frame.sequence;
                if(waitingKey&&!frame.key())continue;
                if(codec==null){
                    MediaFormat format=MediaFormat.createVideoFormat("video/avc",1280,720);
                    format.setInteger(MediaFormat.KEY_MAX_INPUT_SIZE,H264Wire.MAX_PACKET);
                    format.setByteBuffer("csd-0",ByteBuffer.wrap(H264Wire.nal(frame.data,7)));
                    format.setByteBuffer("csd-1",ByteBuffer.wrap(H264Wire.nal(frame.data,8)));
                    MediaCodecInfo info=hardwareDecoder(format);
                    if(Build.VERSION.SDK_INT>=30&&info.getCapabilitiesForType("video/avc").isFeatureSupported(MediaCodecInfo.CodecCapabilities.FEATURE_LowLatency))
                        format.setInteger(MediaFormat.KEY_LOW_LATENCY,1);
                    codec=MediaCodec.createByCodecName(info.getName());codec.configure(format,surface,null,0);codec.start();
                    final MediaCodec decoder=codec;decodedFrames=0;
                    Log.i("ACG138Video","hardware decoder="+info.getName()+" size=1280x720 target=30");
                    output=new Thread(()->{
                        MediaCodec.BufferInfo buffer=new MediaCodec.BufferInfo();long last=SystemClock.elapsedRealtime(),count=0;
                        try{
                            while(!outputStop.get()&&run==epoch.get()&&active.yes()){
                                int index=decoder.dequeueOutputBuffer(buffer,10000);
                                if(index>=0){
                                    // 部分Surface解码器会将有效显示图像的size报告为0。
                                    boolean picture=(buffer.flags&(MediaCodec.BUFFER_FLAG_CODEC_CONFIG|MediaCodec.BUFFER_FLAG_END_OF_STREAM))==0;
                                    decoder.releaseOutputBuffer(index,picture);
                                    if(picture){decodedFrames++;count++;}
                                    long now=SystemClock.elapsedRealtime();
                                    if(now-last>=2000){Log.i("ACG138Video","decoded_fps="+(count*1000.0/(now-last))+" frames="+decodedFrames);last=now;count=0;}
                                }
                            }
                        }catch(Exception error){if(!outputStop.get()){Log.w("ACG138Video","decoder output failed",error);outputFailed.set(true);c.disconnect();}}
                    },"acg138-video-output");output.start();waitingKey=false;
                }
                if(outputFailed.get())throw new IOException("手机视频解码中断");
                int index=codec.dequeueInputBuffer(15000);if(index<0)throw new IOException("手机解码队列繁忙，重新同步");
                ByteBuffer input=codec.getInputBuffer(index);
                if(input==null||input.capacity()<frame.data.length)throw new IOException("手机解码缓冲区不足");
                input.clear();input.put(frame.data);
                codec.queueInputBuffer(index,0,frame.data.length,frame.captureMs*1000,frame.key()?MediaCodec.BUFFER_FLAG_KEY_FRAME:0);
            }
        }finally{
            outputStop.set(true);c.disconnect();if(connection==c)connection=null;
            if(output!=null)try{output.join(300);}catch(InterruptedException e){Thread.currentThread().interrupt();}
            if(codec!=null){try{codec.stop();}catch(Exception ignored){}codec.release();}
        }
    }
}
