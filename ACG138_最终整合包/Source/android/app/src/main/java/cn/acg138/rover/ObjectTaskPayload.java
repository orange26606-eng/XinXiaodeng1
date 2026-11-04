package cn.acg138.rover;
/** 本地任务接口，选择和ROI请求不包含电机操作。 */
final class ObjectTaskPayload {
 private static void seq(long s){if(s<=0||s>9007199254740991L)throw new IllegalArgumentException("任务编号无效");}
 private static void epoch(String e){if(e==null||!e.matches("[1-9][0-9]{0,19}"))throw new IllegalArgumentException("实物身份无效");try{Long.parseUnsignedLong(e);}catch(Exception x){throw new IllegalArgumentException("实物身份超出范围");}}
 static String build(String action,long s,String e,int id,String mode,double standoff){seq(s);epoch(e);if(!action.equals("prepare")&&!action.equals("start")&&!action.equals("cancel")&&!action.equals("pause")&&!action.equals("resume"))throw new IllegalArgumentException("任务操作无效");if((action.equals("pause")||action.equals("resume"))&&!"follow".equals(mode))throw new IllegalArgumentException("暂停和继续只适用于跟随模式");if(id<1||(!mode.equals("static")&&!mode.equals("follow"))||!Double.isFinite(standoff)||standoff<.1||standoff>3)throw new IllegalArgumentException("目标、模式或间距无效");return "{\"seq\":"+s+",\"epoch\":\""+e+"\",\"track_id\":"+id+",\"mode\":\""+mode+"\",\"standoff_m\":"+Double.toString(standoff)+"}";}
 static String roi(long s,String e,String shot,int eye,int x1,int y1,int x2,int y2){seq(s);epoch(e);ObjectPayload.build("select",s,e,shot,1);if((eye!=0&&eye!=1)||x1<0||y1<0||x2>960||y2>1080||x2-x1<8||y2-y1<8)throw new IllegalArgumentException("框选需位于原图内，且宽高至少8像素");return "{\"seq\":"+s+",\"epoch\":\""+e+"\",\"snapshot_id\":\""+shot+"\",\"eye\":"+eye+",\"bbox\":["+x1+","+y1+","+x2+","+y2+"]}";}
}
