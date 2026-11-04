package cn.acg138.rover;
/* 离线语义解析使用完整语句。 */
public final class VoiceCommands {
 public static final class Intent {
  public final String action,category,position,color,size;public final int ordinal;public final boolean reference;public int mentioned;
  Intent(String a,String c,String p,int o){this(a,c,p,o,false);}Intent(String a,String c,String p,int o,boolean r){this(a,c,p,o,r,"","");}Intent(String a,String c,String p,int o,boolean r,String co,String sz){action=a;category=c;position=p;ordinal=o;reference=r;color=co;size=sz;}
 }
 private static final String[][] ALIASES={
  {"cup","杯子","水杯","茶杯","马克杯","杯"},{"glass","wine glass","玻璃杯","酒杯"},{"box","箱子","纸箱","快递箱","盒子","carton","cardboard box","paper box","箱"},
  {"mouse","鼠标","鼠鼠标","电脑鼠标"},{"cell phone","手机","移动电话","电话","phone","cellphone","mobile phone","smartphone"},
  {"person","人","行人","人类","人员"},{"chair","椅子","椅","座椅"},{"bottle","瓶子","水瓶","饮料瓶","瓶"},
  {"cat","猫","小猫"},{"dog","狗","小狗"},{"suitcase","行李箱","旅行箱"},{"backpack","背包","书包"},
  {"dining table","桌子","餐桌","table"},{"couch","沙发","sofa"},{"bed","床"},{"laptop","笔记本电脑","笔记本"},
  {"keyboard","键盘"},{"tv","电视","显示器","monitor"},{"book","书","书本"},{"bowl","碗"},{"bicycle","自行车","单车"}
 };
 private static String key(String s){return s==null?"":s.trim().toLowerCase(java.util.Locale.ROOT).replaceAll("[ _-]","");}
 public static String canonical(String label){String k=key(label);for(String[] row:ALIASES)for(String a:row)if(key(a).equals(k))return row[0];return k;}
 private static boolean known(String s){for(String[] row:ALIASES)for(String a:row)if(key(a).equals(key(s)))return true;return false;}
 public static Intent parse(String raw){Intent i=parseCore(raw);if(i==null)return null;String s=raw==null?"":raw;if((i.action.equals("find")&&!s.matches(".*(?:只识别|仅识别|只查找|仅查找|只找|识别|在哪).*"))||(i.action.equals("selectPreview")&&!s.contains("预览")))i=new Intent("go",i.category,i.position,i.ordinal,i.reference,i.color,i.size);int mask=0;
  if(s.matches(".*(?:走到|靠近|前往|跟随|跟着|跟上|寻找|找到|选择|选中|锁定|预览|开始|停车|停止|取消|去哪|到.+旁边|找|去).*"))mask|=1;
  if(!i.category.isEmpty()){for(String[] row:ALIASES)for(String a:row)if(s.toLowerCase(java.util.Locale.ROOT).contains(a.toLowerCase(java.util.Locale.ROOT)))mask|=2;}
  if(!i.position.isEmpty())mask|=4;if(i.ordinal>0)mask|=8;if(i.reference)mask|=16;if(!i.color.isEmpty())mask|=32;if(!i.size.isEmpty())mask|=64;i.mentioned=mask;return i;
 }
 private static Intent parseCore(String raw){
  if(raw==null||raw.length()>120)return null;String s=raw.trim().replaceAll("[，。！？!?、,.;；：:\\s]","");
  for(int k=0;k<4;k++){String n=s.replaceFirst("^(?:请帮我|能不能帮我|帮我|给我|麻烦你|麻烦|请|机器人|小车)","");if(n.equals(s))break;s=n;}
  s=s.replaceFirst("(?:一下吧|一下|好吗|可以吗|吧)$","");
  if(s.matches("退出跟随模式|关闭跟随模式|取消跟随|停止跟随|结束跟随|不要跟随了?|别跟随了?|不要跟着了?|别跟着了?|取消寻找|停止寻找"))return new Intent("cancel","","",0);
  if(s.matches("先?预览(?:一下)?(?:路线)?(?:但)?不要开始"))return new Intent("preview","","",0);
  if(s.replace("识别","").matches(".*[不别勿没非].*"))return null;
  if(s.matches("停车|停止|停|停下|停一下|停止导航|紧急停车|马上停车|立即停车"))return new Intent("stop","","",0);
  if(s.matches("开始导航|开始|开始行驶|开始接近|执行|继续跟随|继续导航"))return new Intent("start","","",0);
  if(s.matches("取消|取消导航|取消任务|结束任务"))return new Intent("cancel","","",0);
  if(s.matches("状态|查询状态|现在什么状态|当前状态|任务状态"))return new Intent("status","","",0);
  if(s.matches("预览|预览路线|看看路线|看路线|规划路线"))return new Intent("preview","","",0);
  if(s.matches("开启跟随模式|进入跟随模式|开始跟随|跟随所选目标"))return new Intent("follow","","",0,true);
  if(s.matches("跟着我|跟随我"))return new Intent("follow","person","",0,true);
  String action=null,target=null;
  java.util.regex.Matcher where=java.util.regex.Pattern.compile("(.+?)(?:在哪里|在哪儿|在哪)").matcher(s);if(where.matches()){action="find";target=where.group(1);}
  java.util.regex.Matcher sequence=java.util.regex.Pattern.compile("(?:找到|寻找)(.+?)(?:然后|并|后)(走过去|过去|靠近|跟随)").matcher(s);
  if(action==null&&sequence.matches()){action=sequence.group(2).equals("跟随")?"follow":"go";target=sequence.group(1);}
  if(action==null){String[][] prefixes={{"select","选择","选中","选一下","选","锁定","我要"},{"follow","跟随","跟着","跟上"},{"find","只识别","仅识别","只查找","仅查找","只找","识别一下","识别","寻找","找到","找一下","找","看看有没有"},{"go","走到","开到","到","靠近","前往","过去找"},{"selectPreview","去","预览"}};
   outer:for(String[] row:prefixes)for(int i=1;i<row.length;i++)if(s.startsWith(row[i])){action=row[0];target=s.substring(row[i].length());break outer;}}
  if(action==null){action="select";target=s.replaceFirst("^就是|^就","");}if(target==null||target.isEmpty())return null;
  if(action.equals("selectPreview")&&s.startsWith("去")&&target.matches(".*(?:旁边|附近|边上|跟前)$"))action="go";
  target=target.replaceFirst("(?:的)?(?:旁边|附近|边上|跟前|那里|那边)$","");
  if(target.matches("他|她"))return new Intent(action,"person","",0,true);
  if(target.matches("它|这个|那个|这个目标|那个目标|当前目标|选中的目标|所选目标|选中目标|刚才那个|刚才选的|刚才选中的目标|之前选的目标"))return new Intent(action,"","",0,true);
  java.util.regex.Matcher reference=java.util.regex.Pattern.compile("(?:这个|那个|刚才那个|刚才选的)(.+)").matcher(target);
  if(reference.matches()&&known(reference.group(1)))return new Intent(action,canonical(reference.group(1)),"",0,true);
  String pos="";String[][] sides={{"left","最左边","最左侧","左边","左侧","左面"},{"right","最右边","最右侧","右边","右侧","右面"},{"center","正前方","正前面","前方","前面","中间","中央"},{"near","最近的","最近","近处","近一点"},{"far","最远的","最远","远处","远一点"}};
  outer:for(String[] row:sides)for(int i=1;i<row.length;i++)if(target.startsWith(row[i])){pos=row[0];target=target.substring(row[i].length());break outer;}
  target=target.replaceFirst("^的","").replaceFirst("^(?:这|那)(?:一)?(?:个|把|只|张|台)?","");
  target=target.replaceFirst("^(?:一)?(?:个|把|只|张|台)","");
  int ordinal=0;java.util.regex.Matcher n=java.util.regex.Pattern.compile("第([0-9一二两三四五六七八九十]+)(?:个|把|只|张|台)(.*)").matcher(target);
  if(n.matches()){ordinal=number(n.group(1));if(ordinal<1||ordinal>65||!pos.isEmpty())return null;target=n.group(2);if(target.equals("目标"))target="";}
  String color="",size="";
  String[][] colors={{"red","红色","红的","红"},{"orange","橙色","橙的","橙"},{"yellow","黄色","黄的","黄"},{"green","绿色","绿的","绿"},{"cyan","青色","青的","青"},{"blue","蓝色","蓝的","蓝"},{"purple","紫色","紫的","紫"},{"pink","粉红色","粉色","粉红","粉"},{"brown","棕色","褐色","棕","褐"},{"white","白色","白的","白"},{"gray","灰色","灰的","灰"},{"black","黑色","黑的","黑"},{"mixed","多色","双色"}};
  String[][] sizes={{"large","大一点的","大号的","大型的","大号","大型","大的","大"},{"small","小一点的","小号的","小型的","小号","小型","小的","小"},{"medium","中等大小的","中号的","中型的","中号","中型"}};
  for(int pass=0;pass<3&&!known(target);pass++){
   boolean removed=false;
   outerColor:for(String[] row:colors)for(int k=1;k<row.length;k++)if(target.startsWith(row[k])){if(!color.isEmpty())return null;color=row[0];target=target.substring(row[k].length()).replaceFirst("^的","");removed=true;break outerColor;}
   outerSize:for(String[] row:sizes)for(int k=1;k<row.length;k++)if(target.startsWith(row[k])){if(!size.isEmpty())return null;size=row[0];target=target.substring(row[k].length()).replaceFirst("^的","");removed=true;break outerSize;}
   if(!removed)break;
  }
  if(!target.isEmpty()&&!known(target))return null;
  boolean ref=target.isEmpty()&&ordinal==0&&pos.isEmpty();if(ref)return null;
  return new Intent(action,target.isEmpty()?"":canonical(target),pos,ordinal,false,color,size);
 }
 private static int number(String n){try{return Integer.parseInt(n);}catch(NumberFormatException ignored){}n=n.replace('两','二');int t=n.indexOf('十');if(t<0)return n.length()==1?"一二三四五六七八九".indexOf(n)+1:-1;if(n.indexOf('十',t+1)>=0||t>1||n.length()-t>2)return -1;int tens=t==0?1:"一二三四五六七八九".indexOf(n.substring(0,t))+1;int ones=t==n.length()-1?0:"一二三四五六七八九".indexOf(n.substring(t+1))+1;return tens*10+ones;}
 public static final class Candidate {public final int track;public final String label;public final double x,range;public Candidate(int t,String l,double p){this(t,l,p,Double.NaN);}public Candidate(int t,String l,double p,double r){track=t;label=l;x=p;range=r;}}
 public static int resolve(Intent intent,Candidate[] candidates,long ageMs){
  if(intent==null||candidates==null||candidates.length>65||ageMs<0||ageMs>800)return -1;
  java.util.HashSet<Integer> ids=new java.util.HashSet<>();java.util.ArrayList<Candidate> found=new java.util.ArrayList<>();
  for(Candidate c:candidates){if(c==null||c.track<1||c.label==null||!Double.isFinite(c.x)||!ids.add(c.track))return -1;if(intent.category.isEmpty()||(canonical(c.label).equals(canonical(intent.category))||canonical(intent.category).equals("cup")&&canonical(c.label).equals("glass")))found.add(c);}
  if(intent.ordinal>0)return intent.ordinal<=found.size()?found.get(intent.ordinal-1).track:-1;
  String side=intent.position.replace("左边","left").replace("右边","right").replace("中间","center");
  if(found.size()==1&&side.isEmpty())return found.get(0).track;
  if(found.isEmpty()||side.isEmpty())return -1;
  if(side.equals("near")||side.equals("far")){for(Candidate c:found)if(!Double.isFinite(c.range)||c.range<=0)return -1;found.sort((a,b)->Double.compare(a.range,b.range));int index=side.equals("near")?0:found.size()-1;if(found.size()>1&&Math.abs(found.get(index).range-found.get(index==0?1:index-1).range)<.15)return -1;return found.get(index).track;}
  found.sort((a,b)->Double.compare(a.x,b.x));int index=side.equals("left")?0:side.equals("right")?found.size()-1:side.equals("center")&&found.size()%2==1?found.size()/2:-1;
  if(index<0)return -1;if(found.size()>1&&((index>0&&Math.abs(found.get(index).x-found.get(index-1).x)<20)||(index+1<found.size()&&Math.abs(found.get(index).x-found.get(index+1).x)<20)))return -1;return found.get(index).track;
 }
 public static final class Session {private long id,started;private boolean consumed;public long begin(long now){started=now;consumed=false;return ++id;}public void cancel(){++id;consumed=true;}public boolean current(long token,long now){return token==id&&!consumed&&now>=started&&now-started<=15000;}public boolean take(long token,long now){if(!current(token,now))return false;consumed=true;return true;}}
}
