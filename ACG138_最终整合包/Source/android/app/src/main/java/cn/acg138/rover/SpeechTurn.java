package cn.acg138.rover;
/* 一次点击对应一次语音轮次，最终文本进入指令解析。 */
final class SpeechTurn {
 static final int CONTINUE=0,ENDPOINT=1,LIMIT=2,EMPTY=3;
 static final long MAX_RECORD_MS=10000;
 private String latest="";private boolean finished;
 int update(String text,boolean endpoint,long elapsedMs){if(finished)return CONTINUE;if(text!=null)latest=text.trim();if(endpoint&&!latest.isEmpty()){finished=true;return ENDPOINT;}if(elapsedMs>=MAX_RECORD_MS){finished=true;return latest.isEmpty()?EMPTY:LIMIT;}return CONTINUE;}
 String text(){return latest;}
 boolean finished(){return finished;}
}
