"""生成已初始化的UART文字缓冲与可变字段字节描述表。"""
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "fpga/src/motion"

def table(feedback, near):
    template = []
    variables = []
    def literal(text):
        template.extend(text.encode("ascii"))
    def field(number, count):
        for index in range(count):
            variables.append((len(template), number, index))
            template.append(ord("_"))
    literal("F=");field(1,5);literal(" L=");field(2,5);literal(" R=");field(3,5)
    literal(" Q=");field(4,3);literal(" RANGE=");field(5,3);literal(" ERR=");field(6,3)
    literal(" Z=");field(7,3)
    if feedback or near:
        literal(" C=");field(8,3)
    literal(" MODE=");field(9,4);literal(" ACT=");field(10,6)
    literal(" FB=");field(11,1);literal(" WHY=");field(12,10);literal(" N=");field(13,1)
    if feedback:
        literal(" FBS=");field(14,1);literal(" FBR=");field(15,1)
    literal("\r\nUSHEX F=");field(16,24);literal(" L=");field(17,24);literal(" R=");field(18,24);literal("\r\n")
    length = (194 if near else 188) + (12 if feedback else 0)
    offset = len(template) - length
    assert offset >= 0
    template = template[offset:]
    variables = [(address-offset, number, index) for address,number,index in variables if address >= offset]
    name = "fb_near" if feedback and near else "fb_legacy" if feedback else "near" if near else "legacy"
    descriptors = [(address<<12)|(number<<7)|index for address,number,index in variables]
    (DEST / ("status_template_"+name+".hex")).write_text("".join(f"{value:02x}\n" for value in template+[0]*(256-length)), encoding="ascii")
    (DEST / ("status_fields_"+name+".hex")).write_text("".join(f"{value:05x}\n" for value in descriptors+[0]*(256-len(descriptors))), encoding="ascii")
    print(name, "bytes", length, "variable_bytes", len(descriptors))

if __name__ == "__main__":
    for feedback, near in [(0,0),(0,1),(1,0),(1,1)]:
        table(feedback,near)
