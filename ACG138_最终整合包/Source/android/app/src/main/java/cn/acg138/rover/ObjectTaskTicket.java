package cn.acg138.rover;
/** 请求代次不可变，打开连接前和写入前均核对。 */
final class ObjectTaskTicket {
 private final long connection,control,intent; private final String owner;
 ObjectTaskTicket(long connection,long control,long intent,String owner){this.connection=connection;this.control=control;this.intent=intent;this.owner=owner;}
 boolean current(long connection,long control,long intent,boolean linked,boolean foreground,boolean closed,String owner){return this.connection==connection&&this.control==control&&this.intent==intent&&linked&&foreground&&!closed&&this.owner!=null&&this.owner.equals(owner);}
}
