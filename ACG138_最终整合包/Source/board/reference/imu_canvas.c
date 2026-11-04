/* 从ASCII姿态与状态绘制扩展为浅色中文界面、抗锯齿字形、连接和统计显示；仅绘制，保留报文与校准处理。


 */
#include "imu_canvas.h"
#include "imu_font_data.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static const uint32_t background=0x00F2F3F5, ink=0x00232930, muted=0x0067727E;
static const uint32_t green=0x0024764B, red=0x00B33730, amber=0x00915E09;
static char connection_interface[32]="自动选择", connection_bind[48]="192.168.0.3", connection_peer[48]="192.168.0.2";
static unsigned connection_port=5006;
void imu_canvas_set_connection(const char *interface_name,const char *bind,const char *peer,unsigned port)
{
    snprintf(connection_interface,sizeof(connection_interface),"%s",interface_name && *interface_name ? interface_name : "自动选择");
    snprintf(connection_bind,sizeof(connection_bind),"%s",bind ? bind : "—");
    snprintf(connection_peer,sizeof(connection_peer),"%s",peer ? peer : "—");
    connection_port=port;
}
static void rectangle(uint32_t *p,int x,int y,int w,int h,uint32_t color)
{
    for(int dy=0;dy<h;++dy) for(int dx=0;dx<w;++dx) {
        int px=x+dx,py=y+dy;
        if(px>=0 && px<IMU_CANVAS_WIDTH && py>=0 && py<IMU_CANVAS_HEIGHT)
            p[(size_t)py*IMU_CANVAS_WIDTH+px]=color;
    }
}
static uint32_t next_codepoint(const unsigned char **cursor)
{
    const unsigned char *s=*cursor; uint32_t c=*s++; unsigned count=0;
    if(c>=0xc2 && c<=0xdf) { c&=31; count=1; }
    else if(c>=0xe0 && c<=0xef) { c&=15; count=2; }
    else if(c>=0xf0 && c<=0xf4) { c&=7; count=3; }
    else if(c>=0x80) c='?';
    for(unsigned i=0;i<count;++i) {
        if((*s&0xc0)!=0x80) { c='?'; break; }
        c=(c<<6)|(*s++&63);
    }
    *cursor=s; return c;
}
static const struct imu_glyph *glyph(unsigned font,uint32_t code)
{
    const struct imu_font *f=&imu_fonts[font]; size_t lo=0,hi=f->count;
    while(lo<hi) { size_t mid=lo+(hi-lo)/2; if(f->glyphs[mid].code<code) lo=mid+1; else hi=mid; }
    if(lo<f->count && f->glyphs[lo].code==code) return &f->glyphs[lo];
    return code=='?' ? NULL : glyph(font,'?');
}
static int text_width(const char *string,unsigned font)
{
    int width=0; const unsigned char *s=(const unsigned char *)string;
    while(*s) { const struct imu_glyph *g=glyph(font,next_codepoint(&s)); if(g) width+=g->advance; }
    return width;
}
static uint32_t blend(uint32_t dst,uint32_t src,unsigned a)
{
    unsigned r=(((src>>16)&255)*a+((dst>>16)&255)*(255-a)+127)/255;
    unsigned g=(((src>>8)&255)*a+((dst>>8)&255)*(255-a)+127)/255;
    unsigned b=((src&255)*a+(dst&255)*(255-a)+127)/255;
    return (r<<16)|(g<<8)|b;
}
static void text(uint32_t *p,int x,int y,const char *string,unsigned font,uint32_t color)
{
    const unsigned char *s=(const unsigned char *)string;
    while(*s) {
        const struct imu_glyph *g=glyph(font,next_codepoint(&s)); if(!g) continue;
        for(int row=0;row<g->height;++row) for(int col=0;col<g->width;++col) {
            int px=x+g->left+col,py=y+imu_fonts[font].ascent+g->top+row;
            if(px<0 || px>=IMU_CANVAS_WIDTH || py<0 || py>=IMU_CANVAS_HEIGHT) continue;
            unsigned a=imu_font_pixels[g->offset+(size_t)row*g->width+col];
            if(a) { size_t index=(size_t)py*IMU_CANVAS_WIDTH+px; p[index]=blend(p[index],color,a); }
        }
        x+=g->advance;
    }
}
static void centered(uint32_t *p,int x,int y,int width,const char *s,unsigned font,uint32_t color)
{ text(p,x+(width-text_width(s,font))/2,y,s,font,color); }
static void fit(uint32_t *p,int x,int y,int width,const char *s,unsigned font,uint32_t color)
{
    if(text_width(s,font)>width) font=3;
    char limited[256]; snprintf(limited,sizeof(limited),"%s",s);
    while(text_width(limited,font)>width && *limited) {
        size_t n=strlen(limited)-1;
        while(n && ((unsigned char)limited[n]&0xc0)==0x80) --n;
        limited[n]=0;
    }
    text(p,x,y,limited,font,color);
}
static const char *error_zh(unsigned status)
{
    switch(status) {
    case 1:return "设备未应答"; case 2:return "SCL 等待超时";
    case 3:return "总线空闲检查超时"; case 4:return "非法命令";
    case 5:return "数据有效性检查失败"; default:return "未知错误";
    }
}
void imu_canvas_draw(uint32_t *p,const struct imu_model *model,uint64_t now_ms)
{
    enum imu_state state=imu_model_state(model,now_ms);
    uint32_t status_color=state==IMU_LIVE ? green : state==IMU_WAITING || state==IMU_STALE ? amber : red;
    const char *badge=state==IMU_LIVE ? "实时接收" : state==IMU_WAITING ? "等待数据" : "数据异常";
    char line[256];
    rectangle(p,0,0,IMU_CANVAS_WIDTH,IMU_CANVAS_HEIGHT,background);
    rectangle(p,0,0,IMU_CANVAS_WIDTH,58,0x00FAFAFB);
    text(p,24,12,model->simulated ? "FPGA IMU 姿态调试 · 示例数据" : "FPGA IMU 姿态调试",1,ink);
    rectangle(p,882,15,134,30,state==IMU_LIVE ? 0x00E2F0E8 : 0x00F7EBDD);
    centered(p,882,15,134,model->simulated ? "非实测数据" : badge,0,status_color);
    snprintf(line,sizeof(line),"接收接口  %s",connection_interface); fit(p,24,74,290,line,0,ink);
    snprintf(line,sizeof(line),"FPGA  %s",connection_peer); fit(p,350,74,305,line,0,ink);
    snprintf(line,sizeof(line),"UDP  %s : %u",connection_bind,connection_port); fit(p,680,74,336,line,0,muted);
    rectangle(p,24,112,992,1,0x00D5D9DE);
    static const char *const labels[]={"Roll 横滚","Pitch 俯仰","Yaw 航向"};
    for(unsigned axis=0;axis<3;++axis) {
        int x=24+(int)axis*336;
        rectangle(p,x,132,320,196,0x00BFC5CC); rectangle(p,x+1,133,318,194,0x00FCFCFD);
        centered(p,x,148,320,labels[axis],0,ink);
        if(!model->have_angles) snprintf(line,sizeof(line),"—");
        else if(fabs(model->degrees[axis])<1000.0) snprintf(line,sizeof(line),"%+.2f",model->degrees[axis]);
        else snprintf(line,sizeof(line),"%+.2E",model->degrees[axis]);
        unsigned f=text_width(line,2)<=288 ? 2 : 1;
        centered(p,x,199,320,line,f,state==IMU_LIVE ? ink : muted);
        centered(p,x,284,320,"单位：度 °",0,muted);
    }
    switch(state) {
    case IMU_WAITING:snprintf(line,sizeof(line),"等待 FPGA 的首帧姿态数据…");break;
    case IMU_LIVE:snprintf(line,sizeof(line),model->simulated ? "界面预览：以下为示例姿态，非开发板实测数据。" : "姿态读取成功，正在接收有效数据。");break;
    case IMU_DEVICE_ERROR:snprintf(line,sizeof(line),"读取失败 E=%u：%s；显示上次有效值。",model->status,error_zh(model->status));break;
    case IMU_INVALID:snprintf(line,sizeof(line),"异常数据帧：%s；显示上次有效值。",imu_decode_name(model->invalid_reason));break;
    case IMU_OUT_OF_ORDER:snprintf(line,sizeof(line),"序号异常；显示上次有效值。");break;
    case IMU_STALE:snprintf(line,sizeof(line),"超过 1 秒没有有效姿态，当前显示为旧值。");break;
    }
    rectangle(p,25,354,6,18,status_color); fit(p,44,345,972,line,0,status_color);
    if(model->have_sequence) snprintf(line,sizeof(line),"序号  %08" PRIX32,model->sequence);
    else snprintf(line,sizeof(line),"序号  —");
    text(p,24,387,line,0,ink);
    if(model->seen_packet) {
        double age=(now_ms>=model->last_packet_ms ? now_ms-model->last_packet_ms : 0)/1000.0;
        if(age<3600) snprintf(line,sizeof(line),"最近收到  %.2f 秒前",age);
        else snprintf(line,sizeof(line),"最近收到  超过 1 小时");
    } else snprintf(line,sizeof(line),"最近收到  —");
    text(p,1016-text_width(line,0),387,line,0,muted);
    rectangle(p,24,427,992,1,0x00D5D9DE);
    static const char *const names[]={"成功","读取失败","异常帧","缺帧","序号异常"};
    const uint64_t values[]={model->good_count,model->error_count,model->invalid_count,model->missing_count,model->order_count};
    for(unsigned i=0;i<5;++i) {
        int x=24+(int)i*200; text(p,x,439,names[i],3,muted);
        snprintf(line,sizeof(line),"%" PRIu64,values[i]); fit(p,x,459,192,line,1,i==0 ? green : values[i] ? red : ink);
    }
    text(p,24,516,"失败或超过 1 秒没有有效数据时，保留上次角度并标为旧值。",3,muted);
}
/* 兼容入口，窗口在终端确认后打开。 */
int imu_canvas_confirm_hit(double x,double y)
{ return x>=24 && x<744 && y>=336 && y<384; }
void imu_canvas_draw_control(uint32_t *p,const struct imu_model *model,const struct imu_control_session *control,uint64_t now_ms)
{
    if(control->phase==IMU_PHASE_STREAMING) { imu_canvas_draw(p,model,now_ms); return; }
    struct imu_model blank; imu_model_init(&blank,now_ms); blank.simulated=model->simulated;
    imu_canvas_draw(p,&blank,now_ms); rectangle(p,24,132,992,370,background);
    const char *message="正在设置九轴算法并发送校准命令，请保持 IMU 水平静止。"; uint32_t color=amber;
    if(control->phase==IMU_PHASE_WAIT_LED) {
        message="请保持静止，确认 IMU 的 D3 校准灯停止闪烁。";
        rectangle(p,24,336,720,48,0x00E2F0E8); text(p,42,342,"我已确认 D3 停止闪烁",0,green);
    } else if(control->phase==IMU_PHASE_CONFIRMING) message="已确认，正在等待 FPGA READY。";
    else if(control->phase==IMU_PHASE_FAILED) {
        color=red; message=control->timed_out ? "控制请求超时，角度接收已暂停。" :
            control->local_error ? "控制通信失败，角度接收已暂停。" :
            control->busy ? "FPGA 会话忙，角度接收已暂停。" : "FPGA 初始化失败，角度接收已暂停。";
    }
    fit(p,24,220,992,message,0,color);
}
