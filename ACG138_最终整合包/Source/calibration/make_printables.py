#!/usr/bin/env python3
"""生成9×6内角点、25mm格长的棋盘打印母版。"""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parent

def output_rects(stem,page_mm,rects,labels):
    from reportlab.pdfgen import canvas
    from reportlab.lib.units import mm
    out=ROOT/'printables';out.mkdir(exist_ok=True)
    w,h=page_mm;c=canvas.Canvas(str(out/(stem+'.pdf')),pagesize=(w*mm,h*mm),pageCompression=1)
    c.setTitle(stem+' - print Actual Size 100%');c.setAuthor('ACG138 calibration tools')
    c.setFillColorRGB(1,1,1);c.rect(0,0,w*mm,h*mm,stroke=0,fill=1);c.setFillColorRGB(0,0,0)
    svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}mm" height="{h}mm" viewBox="0 0 {w} {h}">',
         f'<rect width="{w}" height="{h}" fill="white"/>']
    for x,y,rw,rh in rects:
        c.rect(x*mm,(h-y-rh)*mm,rw*mm,rh*mm,stroke=0,fill=1)
        svg.append(f'<rect x="{x:.8f}" y="{y:.8f}" width="{rw:.8f}" height="{rh:.8f}" fill="black"/>')
    c.setFont('Helvetica',10)
    for x,y,text in labels:
        c.drawString(x*mm,(h-y)*mm,text)
        svg.append(f'<text x="{x}" y="{y}" font-family="sans-serif" font-size="3.5">{text}</text>')
    c.showPage();c.save();svg.append('</svg>');(out/(stem+'.svg')).write_text('\n'.join(svg)+'\n',encoding='utf8')


def main():
    rects=[]
    for y in range(7):
        for x in range(10):
            if (x+y)%2==0:rects.append((23.5+x*25,17.5+y*25,25,25))
    output_rects('checkerboard_9x6_inner_25mm_A4_landscape',(297,210),rects,
                 [(23.5,10,'9 x 6 INNER corners | 10 x 7 squares | square 25 mm | pattern 250 x 175 mm'),
                  (23.5,203,'Print Actual Size / 100%. Measure both axes; mount on a flat rigid board.')])
    (ROOT/'printables/dimensions.json').write_text(json.dumps(dict(
        checkerboard=dict(page_mm=[297,210],square_mm=25,squares=[10,7],inner_corners=[9,6],pattern_mm=[250,175])),indent=2)+'\n')
    print('PASS: chessboard vector PDF/SVG generated.')

if __name__=='__main__':main()
