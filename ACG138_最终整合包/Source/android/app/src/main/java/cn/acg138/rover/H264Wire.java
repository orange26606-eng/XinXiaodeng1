package cn.acg138.rover;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Arrays;

/** 有界ACV1访问单元，多字节整数使用网络字节序。 */
final class H264Wire {
    static final int MAX_PACKET=2*1024*1024,KEY=2,CONFIG=1;
    static final class Frame {
        final long sequence,captureMs,encodedMs;final int flags;final byte[] data;
        Frame(long seq,long capture,long encoded,int flags,byte[] data){sequence=seq;captureMs=capture;encodedMs=encoded;this.flags=flags;this.data=data;}
        boolean key(){return (flags&KEY)!=0;}
    }
    static Frame read(InputStream input)throws IOException {
        DataInputStream in=new DataInputStream(input);
        if(in.readInt()!=0x41435631||in.readUnsignedShort()!=40)throw new IOException("视频流格式不匹配");
        int flags=in.readUnsignedShort(),length=in.readInt();long seq=in.readInt()&0xffffffffL;
        long capture=in.readLong(),encoded=in.readLong();int width=in.readUnsignedShort(),height=in.readUnsignedShort(),fps=in.readInt();
        if((flags&~3)!=0||length<=0||length>MAX_PACKET||capture<0||encoded<capture||width!=1280||height!=720||fps!=30000)
            throw new IOException("视频帧参数异常");
        byte[] data=new byte[length];in.readFully(data);
        boolean idr=nal(data,5)!=null;byte[] sps=nal(data,7),pps=nal(data,8);
        if(idr!=((flags&KEY)!=0)||((flags&CONFIG)!=0)!=(sps!=null&&pps!=null)||idr&&(sps==null||pps==null))
            throw new IOException("视频关键帧不完整");
        return new Frame(seq,capture,encoded,flags,data);
    }
    private static int prefix(byte[] b,int i){
        if(i+3<b.length&&b[i]==0&&b[i+1]==0&&b[i+2]==0&&b[i+3]==1)return 4;
        if(i+2<b.length&&b[i]==0&&b[i+1]==0&&b[i+2]==1)return 3;return 0;
    }
    static byte[] nal(byte[] data,int type){
        for(int i=0;i+3<data.length;i++){
            int n=prefix(data,i);if(n==0||i+n>=data.length)continue;
            int start=i,end=i+n+1;
            while(end<data.length&&prefix(data,end)==0)end++;
            if((data[i+n]&31)==type)return Arrays.copyOfRange(data,start,end);
            i=end-1;
        }
        return null;
    }
    static boolean consecutive(long previous,long current){return ((previous+1)&0xffffffffL)==current;}
}
