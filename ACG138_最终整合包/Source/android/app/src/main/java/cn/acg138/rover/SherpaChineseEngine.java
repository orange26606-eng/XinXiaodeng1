package cn.acg138.rover;
import android.content.Context;
import com.k2fsa.sherpa.onnx.*;
import java.io.*;
import java.security.MessageDigest;
import org.json.*;
/** 内置模型及配套ARM64 JNI，本模块不包含网络客户端。 */
final class SherpaChineseEngine {
 private final OnlineRecognizer recognizer;
 SherpaChineseEngine(Context context)throws Exception{
  File dir=new File(context.getFilesDir(),"sherpa-zh-2.0.1");if(!dir.isDirectory()&&!dir.mkdirs())throw new IOException("无法创建模型目录");
  JSONObject manifest=new JSONObject(readText(context,"sherpa-zh/model-manifest.json"));
  for(String name:new String[]{"model.int8.onnx","tokens.txt"}){
   JSONObject item=manifest.getJSONObject("files").getJSONObject(name);File target=new File(dir,name);String want=item.getString("sha256");long size=item.getLong("bytes");
   if(!target.isFile()||target.length()!=size||!digest(target).equals(want)){
    File temp=new File(dir,name+".tmp");try(InputStream in=context.getAssets().open("sherpa-zh/"+name);OutputStream out=new FileOutputStream(temp)){byte[]buf=new byte[65536];int n;while((n=in.read(buf))!=-1)out.write(buf,0,n);}
    if(temp.length()!=size||!digest(temp).equals(want)){temp.delete();throw new IOException("内置中文模型校验失败："+name);}
    if(target.exists()&&!target.delete())throw new IOException("无法更新中文模型："+name);if(!temp.renameTo(target))throw new IOException("无法提交中文模型："+name);
   }
  }
  FeatureConfig feat=new FeatureConfig();feat.setSampleRate(16000);feat.setFeatureDim(80);feat.setDither(0f);
  OnlineZipformer2CtcModelConfig ctc=new OnlineZipformer2CtcModelConfig();ctc.setModel(new File(dir,"model.int8.onnx").getAbsolutePath());
  OnlineModelConfig model=new OnlineModelConfig();model.setZipformer2Ctc(ctc);model.setTokens(new File(dir,"tokens.txt").getAbsolutePath());model.setNumThreads(2);model.setProvider("cpu");model.setDebug(false);
  EndpointConfig endpoint=new EndpointConfig();endpoint.setRule1(new EndpointRule(false,3.0f,0f));endpoint.setRule2(new EndpointRule(true,1.0f,0f));endpoint.setRule3(new EndpointRule(false,0f,10f));
  OnlineRecognizerConfig config=new OnlineRecognizerConfig();config.setFeatConfig(feat);config.setModelConfig(model);config.setEndpointConfig(endpoint);config.setEnableEndpoint(true);config.setDecodingMethod("greedy_search");
  recognizer=new OnlineRecognizer(null,config);
 }
 OnlineStream stream(){return recognizer.createStream("");}
 String accept(OnlineStream stream,float[]samples){stream.acceptWaveform(samples,16000);decode(stream);return recognizer.getResult(stream).getText();}
 void decode(OnlineStream stream){while(recognizer.isReady(stream))recognizer.decode(stream);}
 boolean endpoint(OnlineStream stream){return recognizer.isEndpoint(stream);}
 String finish(OnlineStream stream){stream.acceptWaveform(new float[8000],16000);stream.inputFinished();decode(stream);return recognizer.getResult(stream).getText();}
 void close(){recognizer.release();}
 private static String readText(Context c,String name)throws IOException{try(InputStream in=c.getAssets().open(name);ByteArrayOutputStream out=new ByteArrayOutputStream()){byte[]buf=new byte[8192];int n;while((n=in.read(buf))!=-1)out.write(buf,0,n);return new String(out.toByteArray(),java.nio.charset.StandardCharsets.UTF_8);}}
 private static String digest(File file)throws Exception{MessageDigest d=MessageDigest.getInstance("SHA-256");try(InputStream in=new FileInputStream(file)){byte[]b=new byte[65536];int n;while((n=in.read(b))!=-1)d.update(b,0,n);}StringBuilder s=new StringBuilder();for(byte b:d.digest())s.append(String.format(java.util.Locale.ROOT,"%02x",b&255));return s.toString();}
}
