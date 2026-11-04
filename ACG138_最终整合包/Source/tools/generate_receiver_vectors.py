"""生成确定性的合法及错误以太网接收测试帧。"""
from pathlib import Path
import random,struct,zlib
ROOT=Path(__file__).resolve().parents[1]
rng=random.Random(77381)
def frame(kind, body, broadcast=False):
    port=5012 if kind==0 else 5009
    payload=body+struct.pack("<I",zlib.crc32(body))
    ip=bytearray(struct.pack("!BBHHHBBH4s4s",0x45,0,20+8+len(payload),0,0x4000,64,17,0,bytes([192,168,0,3]),bytes([192,168,0,2])))
    total=sum(struct.unpack("!10H",ip));total=(total&65535)+(total>>16);total=(total&65535)+(total>>16)
    ip[10:12]=struct.pack("!H",(~total)&65535)
    ethernet=(bytes([255])*6 if broadcast else bytes.fromhex("000a3501fec0"))+bytes.fromhex("0200000000010800")
    data=ethernet+ip+struct.pack("!HHHH",port,port,8+len(payload),0)+payload
    return bytearray(bytes([0x55])*7+bytes([0xd5])+data+struct.pack("<I",zlib.crc32(data)))
def recalc_fcs(data):
    data[-4:]=struct.pack("<I",zlib.crc32(data[8:-4]))
def body(kind,i):
    if kind==0:
        return bytes(rng.randrange(256) for _ in range(60))
    b=bytearray(28);b[:5]=b"MCV1\x01";b[5]=i%5;b[6]=1+i%8 if b[5]==3 else 0
    b[8:12]=struct.pack("<I",rng.getrandbits(32));b[12:16]=struct.pack("<I",rng.getrandbits(32));return bytes(b)
vectors=[]
for kind in [0,1]:
    for i in range(32):
        vectors.append((kind,1,255,frame(kind,body(kind,i),i%7==0)))
    for corruption in range(13):
        data=frame(kind,body(kind,3));error=255
        if corruption==0:data[-1]^=1
        elif corruption==1:
            data[-8]^=1;recalc_fcs(data)
        elif corruption==2:
            data[32]^=1;recalc_fcs(data)
        elif corruption==3:
            data[41]=9;recalc_fcs(data)
        elif corruption==4:
            data[45]^=1;recalc_fcs(data)
        elif corruption==5:error=len(data)-1
        elif corruption==6:data=data[:-1]
        elif corruption==7:data+=bytes([0])
        elif corruption==8:data[7]^=1
        elif corruption==9:
            data[8]^=1;recalc_fcs(data)
        elif corruption==10:
            data[22]=0x46;recalc_fcs(data)
        elif corruption==11:data=data[:-17]
        else:data[0]=0x54
        vectors.append((kind,0,error,data))
    if kind==1:
        for offset,value in [(50,0),(55,5),(56,9),(57,1),(66,1)]:
            data=frame(kind,body(kind,3));data[offset]=value
            data[78:82]=struct.pack("<I",zlib.crc32(data[50:78]));recalc_fcs(data)
            vectors.append((kind,0,255,data))
# 逐个修改受检报文头字段，同时保持IP、应用及以太网CRC有效。
# 以此将字段解析拒绝与CRC拒绝分开验证。
for kind in [0,1]:
    checked=list(range(1,8))+[20,21,22,24,25,28,29,31]+list(range(34,50))
    if kind==1:
        checked+=list(range(50,55))+[57]+list(range(66,78))
    for offset in checked:
        data=frame(kind,body(kind,3));data[offset]^=1
        if 22<=offset<=41:
            data[32:34]=bytes(2)
            total=sum(struct.unpack("!10H",data[22:42]));total=(total&65535)+(total>>16);total=(total&65535)+(total>>16)
            data[32:34]=struct.pack("!H",(~total)&65535)
        if kind==1 and offset>=50:
            data[78:82]=struct.pack("<I",zlib.crc32(data[50:78]))
        recalc_fcs(data);vectors.append((kind,0,255,data))
    data=frame(kind,body(kind,3));data[28]=0
    data[32:34]=bytes(2);total=sum(struct.unpack("!10H",data[22:42]));total=(total&65535)+(total>>16);total=(total&65535)+(total>>16)
    data[32:34]=struct.pack("!H",(~total)&65535);recalc_fcs(data)
    vectors.append((kind,1,255,data))
assert len(vectors)<256
image=bytearray(65536);image[:2]=struct.pack("<H",len(vectors))
for number,(kind,accepted,error,data) in enumerate(vectors,1):
    assert len(data)<=128
    start=number*256;image[start:start+4]=bytes([len(data),error,accepted,kind]);image[start+128:start+128+len(data)]=data
(ROOT/"tests/route_opt/fixtures/receiver_frames.hex").write_text("".join(f"{b:02x}\n"for b in image),encoding="ascii")
print("receiver vectors",len(vectors))
