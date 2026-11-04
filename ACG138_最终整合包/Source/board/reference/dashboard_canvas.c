/* 文字光栅工具沿用imu_canvas.c v1.2.0，固定左视频、右姿态布局，不改报文或传感器处理。更新内容包括有效报文更新率、当前目标、快照年龄、采样航向零点、三角度卡片、中文类别和独立置信度显示；1.4.2将置信度保留两位小数。







 */
#include "dashboard_canvas.h"
#include "imu_font_data.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static const uint32_t background=0x00F2F3F5, ink=0x00232930, muted=0x0067727E;
static const uint32_t green=0x0024764B, red=0x00B33730, amber=0x00915E09;
static void rectangle(uint32_t *p,int x,int y,int w,int h,uint32_t color)
{
    for(int dy=0;dy<h;++dy) for(int dx=0;dx<w;++dx) {
        int px=x+dx,py=y+dy;
        if(px>=0 && px<DASHBOARD_WIDTH && py>=0 && py<DASHBOARD_HEIGHT)
            p[(size_t)py*DASHBOARD_WIDTH+px]=color;
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
            if(px<0 || px>=DASHBOARD_WIDTH || py<0 || py>=DASHBOARD_HEIGHT) continue;
            unsigned a=imu_font_pixels[g->offset+(size_t)row*g->width+col];
            if(a) { size_t index=(size_t)py*DASHBOARD_WIDTH+px; p[index]=blend(p[index],color,a); }
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

static void segment(uint32_t *p, double x0, double y0, double x1, double y1,
                    int width, uint32_t color)
{
    double dx=x1-x0, dy=y1-y0;
    int steps=(int)ceil(fmax(fabs(dx),fabs(dy)));
    if (!steps) steps=1;
    for(int i=0;i<=steps;++i)
        rectangle(p,(int)lround(x0+dx*i/steps)-width/2,
                  (int)lround(y0+dy*i/steps)-width/2,width,width,color);
}

static void heading_plot(uint32_t *p, const struct dashboard_heading *heading,
                         enum imu_state state)
{
    const int cx=1102, cy=411;
    const uint32_t grid=0x00E2E7ED, axis=0x00949EAA;
    const uint32_t direction=state==IMU_LIVE ? 0x00256AB3 : muted;
    char line[128];
    rectangle(p,944,258,316,324,0x00BFC5CC);
    rectangle(p,945,259,314,322,0x00FCFCFD);
    text(p,958,269,"二维航向（相对校准）",0,ink);
    for(int offset=-80;offset<=80;offset+=40) {
        segment(p,cx-100,cy+offset,cx+100,cy+offset,1,grid);
        segment(p,cx+offset,cy-100,cx+offset,cy+100,1,grid);
    }
    for(int degree=0;degree<360;++degree) {
        double a=degree*3.14159265358979323846/180.0;
        double b=(degree+1)*3.14159265358979323846/180.0;
        segment(p,cx+82*cos(a),cy-82*sin(a),cx+82*cos(b),cy-82*sin(b),1,grid);
    }
    segment(p,975,cy,1225,cy,1,axis);
    segment(p,cx,cy+99,cx,cy-101,1,axis);
    segment(p,1225,cy,1219,cy-4,1,axis);
    segment(p,1225,cy,1219,cy+4,1,axis);
    segment(p,cx,cy-101,cx-4,cy-95,1,axis);
    segment(p,cx,cy-101,cx+4,cy-95,1,axis);
    text(p,1231,cy-12,"+X",3,muted);
    text(p,1110,301,"+Y",3,muted);
    text(p,960,cy-12,"-X",3,muted);
    text(p,1110,492,"-Y",3,muted);
    /* 细琥珀色虚线表示固定初始方向。 */
    for(int x=28;x<116;x+=10) segment(p,cx+x,cy,cx+x+4,cy,2,0x00C2934B);
    double vx=0,vy=0;
    if(dashboard_heading_vector(heading,&vx,&vy)) {
        double sx=vx,sy=-vy,px=-sy,py=sx;
        segment(p,cx,cy,cx+72*sx,cy+72*sy,3,direction);
        segment(p,cx+72*sx,cy+72*sy,cx+59*sx+8*px,cy+59*sy+8*py,3,direction);
        segment(p,cx+72*sx,cy+72*sy,cx+59*sx-8*px,cy+59*sy-8*py,3,direction);
        /* 车身固定在原点，车头与车轮一起旋转。 */
        const double along[]={24,12,-18,-18,12,24};
        const double across[]={0,11,11,-11,-11,0};
        for(unsigned i=0;i<5;++i)
            segment(p,cx+along[i]*sx+across[i]*px,cy+along[i]*sy+across[i]*py,
                      cx+along[i+1]*sx+across[i+1]*px,cy+along[i+1]*sy+across[i+1]*py,3,direction);
        for(int side=-1;side<=1;side+=2) for(int axle=-10;axle<=10;axle+=20)
            segment(p,cx+(axle-4)*sx+side*15*px,cy+(axle-4)*sy+side*15*py,
                      cx+(axle+4)*sx+side*15*px,cy+(axle+4)*sy+side*15*py,3,direction);
        snprintf(line,sizeof(line),"相对转角  %+.2f°",heading->relative_degrees);
        centered(p,944,518,316,line,1,direction);
        snprintf(line,sizeof(line),"起点 Yaw  %+.2f°",heading->reference_degrees);
        fit(p,958,555,288,line,3,muted);
    } else {
        centered(p,944,522,316,"等待有效 Yaw…",0,amber);
        centered(p,944,555,316,"水平静置，收到数据后记录零点",3,muted);
    }
    rectangle(p,cx-2,cy-2,5,5,ink);
}

// 使用只读里程计替换旧航向图，保留原始IMU卡片。
void dashboard_odometry_draw(uint32_t*p,const struct odom_state*s,uint64_t now)
{
    char value[160];
    unsigned q=!s->accepted?0:now<s->stamp_ms||now-s->stamp_ms>500?4:s->quality;
    const char*labels[]={"等待定位数据","未标定估计","IMU融合估计","仅编码器估计","定位数据过期","数据中断·轨迹有缺口","疑似轮滑"};
    uint32_t color=q==2?green:q==4?red:amber;
    rectangle(p,944,258,316,324,0x00BFC5CC);rectangle(p,945,259,314,322,0x00FCFCFD);
    text(p,957,266,"里程计坐标 / 轨迹 · 只读",3,ink);
    if(!s->points){text(p,960,310,"等待新版计数与IMU…",0,amber);return;}
    snprintf(value,sizeof(value),"X %+.3f m   Y %+.3f m",s->x,s->y);fit(p,957,292,292,value,3,ink);
    snprintf(value,sizeof(value),"航向 %+.1f°  累计 %.3f m",s->heading*57.295779513,s->total_distance);fit(p,957,318,292,value,3,ink);
    double xmin=0,xmax=0,ymin=0,ymax=0;
    for(unsigned i=0;i<s->points&&i<ODOM_POINTS;i++){xmin=fmin(xmin,s->trace[i].x);xmax=fmax(xmax,s->trace[i].x);ymin=fmin(ymin,s->trace[i].y);ymax=fmax(ymax,s->trace[i].y);}
    xmin=fmin(xmin,s->x);xmax=fmax(xmax,s->x);ymin=fmin(ymin,s->y);ymax=fmax(ymax,s->y);
    double scale=fmin(260/fmax(1,xmax-xmin),170/fmax(1,ymax-ymin)),cx=(xmin+xmax)/2,cy=(ymin+ymax)/2;
    rectangle(p,957,348,290,190,0x00EDF2F6);
    double ox=1102-cx*scale,oy=443+cy*scale;
    segment(p,962,oy,1242,oy,1,0x00C7D2DE);segment(p,ox,353,ox,533,1,0x00C7D2DE);
    for(unsigned i=1;i<s->points&&i<ODOM_POINTS;i++)segment(p,1102+(s->trace[i-1].x-cx)*scale,443-(s->trace[i-1].y-cy)*scale,1102+(s->trace[i].x-cx)*scale,443-(s->trace[i].y-cy)*scale,2,q==4?muted:0x00256AB3);
    double x=1102+(s->x-cx)*scale,y=443-(s->y-cy)*scale,a=s->heading;
    rectangle(p,(int)ox-2,(int)oy-2,5,5,amber);rectangle(p,(int)x-2,(int)y-2,5,5,0x00256AB3);
    segment(p,x,y,x+16*cos(a),y-16*sin(a),3,0x00256AB3);
    snprintf(value,sizeof(value),"%s%s",q<7?labels[q]:"定位异常",s->gaps?" · 有缺口":"");fit(p,957,540,292,value,3,color);
    snprintf(value,sizeof(value),"+X初始前方 / +Y左侧 · %.0fpx/m",scale);fit(p,957,560,292,value,3,muted);
}

static void distance_strip(uint32_t *p,const struct robot_model *robot,uint64_t now)
{
    static const char *const labels[]={"前方","左前","右前"};
    char value[40],note[40];
    for(unsigned ch=0;ch<3;++ch) {
        int x=20+(int)ch*305;
        enum robot_distance_state s=robot_distance_state(robot,ch,now);
        enum robot_range_state range=robot_auto_range_state(robot,ch,now);
        uint32_t color=range==ROBOT_RANGE_ALLOWED?green:range==ROBOT_RANGE_BLOCKED?amber:red;
        rectangle(p,x,65,290,24,0x00FCFCFD);
        text(p,x+6,68,labels[ch],3,muted);
        if((s==ROBOT_DISTANCE || s==ROBOT_HOLD)&&robot->mm[ch]>0) snprintf(value,sizeof(value),"%.1f cm",robot->mm[ch]/10.0);
        else if(s==ROBOT_DISTANCE || s==ROBOT_HOLD) snprintf(value,sizeof(value),"--");
        else if(s==ROBOT_MAXIMUM) snprintf(value,sizeof(value),">450 cm");
        else if(s==ROBOT_BLIND) snprintf(value,sizeof(value),"盲区");
        else if(s==ROBOT_LOST) snprintf(value,sizeof(value),"失效");
        else snprintf(value,sizeof(value),"无效");
        if(s==ROBOT_HOLD) {snprintf(note,sizeof(note),"HOLD");color=amber;}
        else if(range==ROBOT_RANGE_BLOCKED) snprintf(note,sizeof(note),"障碍");
        else if(s==ROBOT_MAXIMUM) snprintf(note,sizeof(note),"ERR=0");
        else if(s==ROBOT_WAITING) snprintf(note,sizeof(note),"WAIT");
        else if(s==ROBOT_STALE) snprintf(note,sizeof(note),"STALE");
        else if(robot && robot->bad_packet) snprintf(note,sizeof(note),"PACKET");
        else snprintf(note,sizeof(note),"ERR=%u",robot?robot->error[ch]:5);
        text(p,x+72,65,value,0,color);
        if(s==ROBOT_MAXIMUM)text(p,x+168,68,"MAX",3,color);
        text(p,x+219,68,note,3,range==ROBOT_RANGE_ALLOWED?muted:color);
    }
}
void dashboard_canvas_draw(uint32_t *p, const struct imu_model *model,
    const uint32_t *video, uint64_t video_frames, uint64_t video_last_ms,
    uint64_t now_ms, const struct dashboard_rate *rate,
    const struct dashboard_detections *detections,
    const struct dashboard_heading *heading, const struct robot_model *robot,
    const struct dashboard_yolo_fps *yolo_fps)
{
    enum imu_state state=imu_model_state(model,now_ms);
    uint32_t status_color=state==IMU_LIVE ? green :
        state==IMU_WAITING || state==IMU_STALE ? amber : red;
    int camera_live=video_frames && now_ms>=video_last_ms && now_ms-video_last_ms<=1000;
    char line[256];
    rectangle(p,0,0,DASHBOARD_WIDTH,DASHBOARD_HEIGHT,background);
    rectangle(p,0,0,DASHBOARD_WIDTH,64,0x00FCFCFD);
    enum robot_drive_mode mode=robot_drive_mode(robot,now_ms);
    /* R4手动命令不经过超声近障门；这里只显示观测，不暗示旧版禁行距离。 */
    text(p,20,3,!robot || !robot->nav_supported?"等待数据":
        mode==ROBOT_MODE_AUTO?"自动模式：FPGA · 测距与方向":"手动：测距只显示 · 按键驱动",3,ink);
    const char *mode_text=mode==ROBOT_MODE_AUTO?"模式：自动巡航":
                          mode==ROBOT_MODE_MANUAL?"模式：按键驱动":"模式：等待数据";
    text(p,460,0,mode_text,0,mode==ROBOT_MODE_AUTO?green:mode==ROBOT_MODE_MANUAL?ink:muted);
    if(robot && robot->nav_supported && mode!=ROBOT_MODE_UNKNOWN) {
        if(robot->feedback_supported && robot->feedback_phase)
            snprintf(line,sizeof(line),"%s | %s %u/2 | FB=0x%X",robot_action_text(robot->actual),robot_feedback_phase_text(robot->feedback_phase),robot->feedback_retries,robot->feedback);
        else if(robot->auto_mode) {
            static const char *const auto_state[]={"等待测距","正常前进","左弧线","左转","右转","恢复前进","右弧线","后退","前进","后退","后退","等待方向","近障停车","恢复方向","恢复方向"};
            const char *state=robot->stop_reason==10?"等待编码器":robot->state<15?auto_state[robot->state]:"等待数据";
            snprintf(line,sizeof(line),"%s | %s (%u)",robot_action_text(robot->actual),state,robot->state);
        } else snprintf(line,sizeof(line),"%s | %s",robot_action_text(robot->actual),robot_reason_text(robot->stop_reason));
        uint32_t phase_color=robot->feedback_supported && robot->feedback_phase && robot->feedback_phase!=4?amber:red;
        fit(p,20,30,890,line,3,robot->stop_reason==4?phase_color:robot->stop_reason?amber:ink);
    } else text(p,20,30,"FPGA 双摄像头与 IMU 姿态",1,ink);
    text(p,940,34,model->simulated ? "界面预览 · 非实测数据" : "实时感知",0,model->simulated ? amber : green);
    distance_strip(p,robot,now_ms);
    text(p,944,68,"IMU 姿态角",3,muted);
    rectangle(p,19,91,902,508,0x00BFC5CC);
    rectangle(p,20,92,DASHBOARD_VIDEO_WIDTH,DASHBOARD_VIDEO_HEIGHT,0x00151B24);
    if(video && video_frames) {
        for(unsigned y=0;y<DASHBOARD_VIDEO_HEIGHT;++y)
            memcpy(p+(size_t)(y+92)*DASHBOARD_WIDTH+20,
                video+(size_t)y*DASHBOARD_VIDEO_WIDTH,DASHBOARD_VIDEO_WIDTH*sizeof(*p));
    } else centered(p,20,318,900,"等待摄像头首帧…",1,0x00D6DAE0);
    static const char *const labels[]={"Roll 横滚","Pitch 俯仰","Yaw 航向"};
    for(unsigned axis=0;axis<3;++axis) {
        int x=944,y=92+(int)axis*53;
        rectangle(p,x,y,316,48,0x00BFC5CC);
        rectangle(p,x+1,y+1,314,46,0x00FCFCFD);
        text(p,x+14,y+10,labels[axis],0,ink);
        if(!model->have_angles) snprintf(line,sizeof(line),"—");
        else if(fabs(model->degrees[axis])<1000.0) snprintf(line,sizeof(line),"%+.2f°",model->degrees[axis]);
        else snprintf(line,sizeof(line),"%+.2E°",model->degrees[axis]);
        unsigned f=text_width(line,1)<=164 ? 1 : 3;
        text(p,x+300-text_width(line,f),y+7,line,f,state==IMU_LIVE ? ink : muted);
    }
    heading_plot(p,heading,state);
    if(video_frames && !camera_live) {
        rectangle(p,20,92,900,34,0x00F7EBDD);
        text(p,32,93,"摄像头暂无新画面 · 当前为旧画面",0,amber);
    }
    if(yolo_fps) {
        double measured=0;
        if(dashboard_yolo_fps_value(yolo_fps,now_ms,&measured)) {
            if(measured<1000)snprintf(line,sizeof(line),"YOLO %.1f FPS",measured);
            else snprintf(line,sizeof(line),"YOLO %.3g FPS",measured);
        } else snprintf(line,sizeof(line),"YOLO -- FPS");
        int width=text_width(line,1)+24;
        int x=20+DASHBOARD_VIDEO_WIDTH-width-12;
        rectangle(p,x,104,width,38,0x00131C26);
        text(p,x+12,108,line,1,0x00FFFFFF);
    }
    uint64_t object_age=0;
    int have_result=dashboard_detections_age_ms(detections,now_ms,&object_age);
    int delayed=have_result && (object_age>=10000 || !camera_live);
    text(p,20,608,"识别到的物体",0,ink);
    if(have_result) {
        if(delayed && detections->total>DASHBOARD_OBJECTS)
            snprintf(line,sizeof(line),"旧结果 · 共 %u 个，显示前 %u 个",detections->total,DASHBOARD_OBJECTS);
        else if(delayed) snprintf(line,sizeof(line),"等待更新 · 保留上次结果");
        else if(detections->total>DASHBOARD_OBJECTS)
            snprintf(line,sizeof(line),"共 %u 个 · 显示置信度前 %u 个",detections->total,DASHBOARD_OBJECTS);
        else snprintf(line,sizeof(line),"最近一次识别 · %u 个目标",detections->total);
        fit(p,210,611,370,line,3,delayed ? amber : muted);
        if(object_age<60000)
            snprintf(line,sizeof(line),"距上次更新：%.1f 秒",object_age/1000.0);
        else snprintf(line,sizeof(line),"距上次更新：%" PRIu64 " 分 %" PRIu64 " 秒",
                      object_age/60000,(object_age/1000)%60);
        int age_width=text_width(line,0);
        fit(p,age_width<=330 ? 920-age_width : 590,608,330,line,0,delayed ? amber : muted);
    }
    const char *object_status=NULL;
    if(!detections) object_status="未启用 YOLO 识别";
    else if(!detections->seen) object_status="等待识别结果…";
    else if(!have_result) object_status="识别结果时间异常，等待更新…";
    else if(!detections->total) object_status="最近一次识别未检测到物体";
    if(object_status) text(p,20,644,object_status,0,muted);
    else for(unsigned i=0;i<detections->count;++i) {
        int x=20+(int)(i%2)*460,y=640+(int)(i/2)*26;
        const char *name=detections->objects[i].name;
        const char *translation=dashboard_object_label(name);
        if(strcmp(name,translation)) snprintf(line,sizeof(line),"%s(%s)",name,translation);
        else snprintf(line,sizeof(line),"%s",name);
        fit(p,x,y,330,line,0,delayed ? muted : ink);
        snprintf(line,sizeof(line),"%.2f%%",detections->objects[i].score*100.0);
        text(p,x+430-text_width(line,0),y,line,0,delayed ? muted : green);
    }
    const char *status="等待姿态数据";
    switch(state) {
    case IMU_LIVE: status="姿态读取成功"; break;
    case IMU_STALE: status="姿态超过 1 秒未更新 · 旧值"; break;
    case IMU_DEVICE_ERROR: status="姿态读取失败 · 保留旧值"; break;
    case IMU_INVALID: status="姿态帧异常 · 保留旧值"; break;
    case IMU_OUT_OF_ORDER: status="姿态序号异常 · 保留旧值"; break;
    case IMU_WAITING: break;
    }
    fit(p,944,592,316,status,3,status_color);
    if(!rate || !rate->ready) snprintf(line,sizeof(line),"回报率（实收） 统计中…");
    else {
        double hz=rate->hz;
        if(model->have_angles && now_ms>=model->last_good_ms && now_ms-model->last_good_ms>=1000) hz=0;
        snprintf(line,sizeof(line),"回报率（实收） %.1f Hz",hz);
    }
    fit(p,944,616,316,line,0,ink);
    if(state==IMU_DEVICE_ERROR) snprintf(line,sizeof(line),"E=%u %s",model->status,error_zh(model->status));
    else if(model->have_sequence) snprintf(line,sizeof(line),"序号  %08" PRIX32,model->sequence);
    else snprintf(line,sizeof(line),"序号  —");
    fit(p,944,644,316,line,3,muted);
    snprintf(line,sizeof(line),"成功 %" PRIu64 "   失败 %" PRIu64,model->good_count,model->error_count);
    fit(p,944,666,316,line,3,muted);
    snprintf(line,sizeof(line),"异常 %" PRIu64 "   缺帧 %" PRIu64 "   乱序 %" PRIu64,
        model->invalid_count,model->missing_count,model->order_count);
    fit(p,944,688,316,line,3,muted);
}
