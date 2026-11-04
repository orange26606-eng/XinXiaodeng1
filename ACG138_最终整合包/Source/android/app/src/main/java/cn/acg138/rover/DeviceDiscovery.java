package cn.acg138.rover;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.security.SecureRandom;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Enumeration;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/** 不依赖Android专有接口，可在JVM上测试同一设备发现逻辑。 */
final class DeviceDiscovery {
    static final int PORT = 37020;
    interface Cancelled { boolean get(); }
    static final class Channel {
        final DatagramSocket socket;
        final List<InetAddress> destinations;
        Channel(DatagramSocket socket,List<InetAddress> destinations) {
            this.socket=socket;this.destinations=destinations;
        }
    }
    static boolean privateAddress(InetAddress address) {
        if (!(address instanceof Inet4Address)) return false;
        byte[] value=address.getAddress();int a=value[0]&255,b=value[1]&255;
        return a==10 || a==172&&b>=16&&b<=31 || a==192&&b==168;
    }
    static String response(String text,String nonce,InetAddress sender) {
        String[] fields=text.split(" ",-1);
        if(fields.length!=3||!fields[0].equals("ACG138_ROVER_V1")||!fields[1].equals(nonce)||
                !fields[2].matches("[0-9]{1,5}"))return null;
        int port=Integer.parseInt(fields[2]);
        if(port<1||port>65535||!(sender instanceof Inet4Address))return null;
        return sender.getHostAddress()+":"+port;
    }
    static List<Channel> channels() throws Exception {
        List<Channel> channels=new ArrayList<>();
        Enumeration<NetworkInterface> interfaces;
        try{interfaces=NetworkInterface.getNetworkInterfaces();}
        catch(Exception unavailable){return channels;}
        if(interfaces==null)return channels;
        while(interfaces.hasMoreElements()&&channels.size()<8){
            NetworkInterface net=interfaces.nextElement();String name=net.getName();
            if(name.matches("(?i).*(rmnet|ccmni|pdp|wwan|tun|ipsec|clat).*"))continue;
            try {
                if(!net.isUp()||net.isLoopback()||net.isPointToPoint())continue;
                for(InterfaceAddress local:net.getInterfaceAddresses()){
                    InetAddress address=local.getAddress();
                    if(!privateAddress(address))continue;
                    DatagramSocket socket=null;
                    try {
                        socket=new DatagramSocket(new InetSocketAddress(address,0));
                        socket.setBroadcast(true);socket.setSoTimeout(85);
                        List<InetAddress> targets=new ArrayList<>();
                        if(local.getBroadcast()!=null)targets.add(local.getBroadcast());
                        targets.add(InetAddress.getByName("255.255.255.255"));
                        channels.add(new Channel(socket,targets));socket=null;
                    } finally {if(socket!=null)socket.close();}
                }
            } catch(Exception ignored) { /* 过期接口或VPN接口不应遮蔽Wi-Fi接口。 */ }
        }
        return channels;
    }
    static List<String> scan(Cancelled cancelled) throws Exception {
        List<Channel> channels=channels();
        try {return scan(channels,PORT,2600,cancelled);}
        finally {for(Channel channel:channels)channel.socket.close();}
    }
    static List<String> scan(List<Channel> channels,int port,int timeout,Cancelled cancelled) throws Exception {
        byte[] random=new byte[16];new SecureRandom().nextBytes(random);StringBuilder nonce=new StringBuilder();
        for(byte value:random)nonce.append(String.format(java.util.Locale.ROOT,"%02x",value&255));
        byte[] query=("ACG138_DISCOVER_V1 "+nonce).getBytes(StandardCharsets.US_ASCII);
        Set<String> devices=new LinkedHashSet<>();long deadline=System.nanoTime()+timeout*1000000L,nextSend=0,firstFound=0;
        while(!channels.isEmpty()&&System.nanoTime()<deadline&&!cancelled.get()&&!Thread.currentThread().isInterrupted()){
            long now=System.nanoTime();
            if(now>=nextSend){
                for(Channel channel:channels)for(InetAddress destination:channel.destinations){
                    try{channel.socket.send(new DatagramPacket(query,query.length,destination,port));}catch(Exception ignored){}
                }
                nextSend=now+650000000L;
            }
            for(Channel channel:channels){
                if(cancelled.get()||Thread.currentThread().isInterrupted())return Collections.emptyList();
                try {
                    byte[] buffer=new byte[192];DatagramPacket packet=new DatagramPacket(buffer,buffer.length);
                    channel.socket.receive(packet);
                    if(packet.getPort()!=port)continue;
                    String host=response(new String(packet.getData(),packet.getOffset(),packet.getLength(),StandardCharsets.US_ASCII),nonce.toString(),packet.getAddress());
                    if(host!=null){devices.add(host);if(firstFound==0)firstFound=System.nanoTime();}
                } catch(SocketTimeoutException ignored) {} catch(Exception ignored) {}
            }
            if(firstFound!=0&&System.nanoTime()-firstFound>400000000L)break;
        }
        return new ArrayList<>(devices);
    }
}
