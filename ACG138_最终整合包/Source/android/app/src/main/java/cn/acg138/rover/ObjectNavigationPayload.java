package cn.acg138.rover;
public final class ObjectNavigationPayload {
 public static String build(long seq,String map,String version,String epoch,int track,String association,double distance){
 if(seq<=0||seq>9007199254740991L||!decimal(map)||!decimal(version)||!decimal(epoch)||track<=0||association==null||!association.matches("[A-Za-z0-9_.:-]{1,95}")||!Double.isFinite(distance)||distance<.2||distance>3)throw new IllegalArgumentException("实物预览身份或停车距离无效");
 return "{\"seq\":"+seq+",\"map_id\":\""+map+"\",\"map_version\":\""+version+"\",\"epoch\":\""+epoch+"\",\"track_id\":"+track+",\"association_id\":\""+association+"\",\"standoff_m\":"+Double.toString(distance)+"}";
 }
 private static boolean decimal(String s){if(s==null||!s.matches("[0-9]{1,20}"))return false;try{return Long.compareUnsigned(Long.parseUnsignedLong(s),0)>0;}catch(Exception e){return false;}}
}
