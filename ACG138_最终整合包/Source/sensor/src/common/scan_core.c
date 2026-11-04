#include "scan_core.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
static uint16_t u16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t u32(const uint8_t *p) { return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3]; }
static uint64_t u64(const uint8_t *p) { return ((uint64_t)u32(p)<<32)|u32(p+4); }
static void w16(uint8_t *p, uint16_t n) { p[0]=(uint8_t)(n>>8);p[1]=(uint8_t)n; }
static void w32(uint8_t *p, uint32_t n) { p[0]=(uint8_t)(n>>24);p[1]=(uint8_t)(n>>16);p[2]=(uint8_t)(n>>8);p[3]=(uint8_t)n; }
static void w64(uint8_t *p, uint64_t n) { w32(p,(uint32_t)(n>>32));w32(p+4,(uint32_t)n); }
uint32_t scan_crc32(const uint8_t *p,size_t length) {
    uint32_t crc=0xffffffffu;
    for(size_t i=0;i<length;++i) {
        if(i>=36 && i<40) continue;
        crc ^= p[i];
        for(int bit=0;bit<8;++bit) crc=(crc>>1)^(0xedb88320u & (uint32_t)-(int32_t)(crc&1));
    }
    return ~crc;
}
size_t scan_encode(uint8_t *out,size_t capacity,const struct scan_frame *f,uint16_t offset,uint16_t count) {
    size_t size=SCAN_HEADER+(size_t)count*8;
    if(!f || f->source<1 || f->source>2 || !f->count || f->count>SCAN_MAX_POINTS ||
       !count || count>SCAN_CHUNK_POINTS || (unsigned)offset+count>f->count || capacity<size || (f->flags&~1u)) return 0;
    memset(out,0,size);memcpy(out,"LS01",4);out[4]=1;out[5]=f->source;w16(out+6,f->flags);
    w32(out+8,f->session);w32(out+12,f->sequence);w64(out+16,f->timestamp_us);
    w16(out+24,f->count);w16(out+26,offset);w16(out+28,count);w32(out+32,f->period_us);
    for(unsigned i=0;i<count;++i) {
        const struct scan_point *p=&f->points[offset+i];uint8_t *d=out+SCAN_HEADER+i*8;
        w16(d,p->angle_q14);w32(d+2,p->distance_q2);d[6]=p->quality;d[7]=p->flags;
    }
    w32(out+36,scan_crc32(out,size));return size;
}
static int newer(uint32_t a,uint32_t b) { return (int32_t)(a-b)>0; }
void scan_expire(struct scan_receiver *s,uint64_t now) {
    for(int i=0;i<2;++i) if(s->slots[i].active && now>=s->slots[i].started_ms && now-s->slots[i].started_ms>250) {
        s->slots[i].active=0;++s->expired;
    }
}
int scan_receive(struct scan_receiver *s,const uint8_t *p,size_t length,uint64_t now) {
    ++s->packets;scan_expire(s,now);
    if(length<SCAN_HEADER || length>SCAN_PACKET_MAX || memcmp(p,"LS01",4) || p[4]!=1 ||
       p[5]<1 || p[5]>2 || (u16(p+6)&~1u) || u16(p+30)!=0 || u32(p+36)!=scan_crc32(p,length)) {
        ++s->invalid;return -1;
    }
    uint32_t session=u32(p+8),seq=u32(p+12);
    unsigned total=u16(p+24),offset=u16(p+26),count=u16(p+28);
    if(!total || total>SCAN_MAX_POINTS || !count || count>SCAN_CHUNK_POINTS || offset+count>total ||
       length!=SCAN_HEADER+count*8u) {++s->invalid;return -1;}
    if(s->have_session && s->current_session!=session) {
        for(unsigned i=0;i<s->retired_count;++i) if(s->retired[i]==session) {++s->dropped;return 0;}
        s->retired[s->retired_index]=s->current_session;s->retired_index=(s->retired_index+1)%4;
        if(s->retired_count<4) ++s->retired_count;
        for(int i=0;i<2;++i) s->slots[i].active=0;
        s->have_latest=0;
    }
    s->current_session=session;s->have_session=1;
    if(s->have_latest && !newer(seq,s->latest.sequence)) {++s->dropped;return 0;}
    struct scan_slot *slot=NULL;
    for(int i=0;i<2;++i) if(s->slots[i].active && s->slots[i].frame.sequence==seq) slot=&s->slots[i];
    if(!slot) {
        for(int i=0;i<2;++i) if(!s->slots[i].active) {slot=&s->slots[i];break;}
        if(!slot) {
            int oldest=newer(s->slots[0].frame.sequence,s->slots[1].frame.sequence)?1:0;
            if(!newer(seq,s->slots[oldest].frame.sequence)) {++s->dropped;return 0;}
            slot=&s->slots[oldest];++s->dropped;
        }
        memset(slot,0,sizeof(*slot));slot->active=1;slot->started_ms=now;
        slot->frame.session=session;slot->frame.sequence=seq;slot->frame.count=(uint16_t)total;
        slot->frame.timestamp_us=u64(p+16);slot->frame.source=p[5];slot->frame.flags=u16(p+6);slot->frame.period_us=u32(p+32);
    }
    if(slot->frame.count!=total || slot->frame.timestamp_us!=u64(p+16) || slot->frame.source!=p[5] ||
       slot->frame.flags!=u16(p+6) || slot->frame.period_us!=u32(p+32)) {++s->invalid;return -1;}
    for(unsigned i=0;i<count;++i) {
        unsigned index=offset+i;uint8_t mask=(uint8_t)(1u<<(index%8));
        const uint8_t *d=p+SCAN_HEADER+i*8;
        struct scan_point point={u16(d),u32(d+2),d[6],d[7]};
        if(slot->seen[index/8]&mask) {
            const struct scan_point *prior=&slot->frame.points[index];
            if(prior->angle_q14!=point.angle_q14 || prior->distance_q2!=point.distance_q2 ||
               prior->quality!=point.quality || prior->flags!=point.flags) {slot->active=0;++s->invalid;return -1;}
            ++s->duplicates;continue;
        }
        slot->seen[index/8]|=mask;slot->frame.points[index]=point;++slot->received;
    }
    if(slot->received==total) {
        slot->frame.received_ms=now;s->latest=slot->frame;s->have_latest=1;slot->active=0;++s->completed;
        for(int i=0;i<2;++i) if(s->slots[i].active && !newer(s->slots[i].frame.sequence,seq)) s->slots[i].active=0;
        return 1;
    }
    return 0;
}
int map_config_valid(const struct map_config *c) {
    return c && isfinite(c->half_extent_m) && c->half_extent_m>=1 && c->half_extent_m<=12 &&
        isfinite(c->mount_x_m) && isfinite(c->mount_y_m) && fabs(c->mount_x_m)<c->half_extent_m*.8 &&
        fabs(c->mount_y_m)<c->half_extent_m*.8 && isfinite(c->yaw_deg) && fabs(c->yaw_deg)<=360 &&
        isfinite(c->vehicle_width_m) && c->vehicle_width_m>0 && c->vehicle_width_m<=2 &&
        isfinite(c->vehicle_length_m) && c->vehicle_length_m>0 && c->vehicle_length_m<=2;
}
void map_point_xy(const struct scan_point *p,const struct map_config *c,double *x,double *y) {
    double angle=(c->yaw_deg-p->angle_q14*90.0/16384.0)*0.017453292519943295;
    double r=p->distance_q2/4000.0;*x=c->mount_x_m+r*cos(angle);*y=c->mount_y_m+r*sin(angle);
}
int map_cell_xy(double x,double y,const struct map_config *c,int *column,int *row) {
    *column=(int)floor((c->half_extent_m-y)/(2*c->half_extent_m)*MAP_CELLS);
    *row=(int)floor((c->half_extent_m-x)/(2*c->half_extent_m)*MAP_CELLS);
    return *column>=0 && *column<MAP_CELLS && *row>=0 && *row<MAP_CELLS;
}
static void ray(struct local_map *m,int x0,int y0,int x1,int y1,int endpoint) {
    int dx=abs(x1-x0),sx=x0<x1?1:-1,dy=-abs(y1-y0),sy=y0<y1?1:-1,err=dx+dy;
    for(int steps=0;steps<MAP_CELLS*3;++steps) {
        if(x0<0 || y0<0 || x0>=MAP_CELLS || y0>=MAP_CELLS) break;
        int last=x0==x1 && y0==y1;uint8_t *cell=&m->cells[y0*MAP_CELLS+x0];
        if(last && endpoint) *cell=2;else if(*cell!=2) *cell=1;
        if(last) break;
        int e2=err*2;if(e2>=dy) {err+=dy;x0+=sx;}if(e2<=dx) {err+=dx;y0+=sy;}
    }
}
void map_build(struct local_map *m,const struct scan_frame *f,const struct map_config *c) {
    memset(m,0,sizeof(*m));for(int k=0;k<3;++k)m->nearest[k]=-1;
    if(!map_config_valid(c) || !f || f->count>SCAN_MAX_POINTS) return;
    int start_col,start_row;map_cell_xy(c->mount_x_m,c->mount_y_m,c,&start_col,&start_row);
    for(unsigned i=0;i<f->count;++i) {
        const struct scan_point *p=&f->points[i];double r=p->distance_q2/4000.0;
        m->zeros+=p->distance_q2==0;m->low_quality+=p->quality==0;
        if(!p->quality || r<.05 || r>12) {++m->invalid;continue;}
        ++m->valid;double x,y;map_point_xy(p,c,&x,&y);
        double angle=atan2(y,x)*57.29577951308232;
        const double center[3]={0,90,-90};
        for(int k=0;k<3;++k) if(fabs(remainder(angle-center[k],360.0))<=20 && (m->nearest[k]<0 || hypot(x,y)<m->nearest[k])) m->nearest[k]=hypot(x,y);
        int col,row;int hit=map_cell_xy(x,y,c,&col,&row);
        if(!hit) {
            double dx=x-c->mount_x_m,dy=y-c->mount_y_m,scale=1;
            const double edge=c->half_extent_m*(1-1.0/MAP_CELLS);
            if(x>edge) scale=fmin(scale,(edge-c->mount_x_m)/dx);
            if(x<-edge) scale=fmin(scale,(-edge-c->mount_x_m)/dx);
            if(y>edge) scale=fmin(scale,(edge-c->mount_y_m)/dy);
            if(y<-edge) scale=fmin(scale,(-edge-c->mount_y_m)/dy);
            map_cell_xy(c->mount_x_m+dx*scale,c->mount_y_m+dy*scale,c,&col,&row);
        }
        ray(m,start_col,start_row,col,row,hit);
    }
}
