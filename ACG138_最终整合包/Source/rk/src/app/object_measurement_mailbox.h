#ifndef ACG_OBJECT_MEASUREMENT_MAILBOX_H
#define ACG_OBJECT_MEASUREMENT_MAILBOX_H
#include "object_target_v2.h"
#include <string.h>
#define OBJECT_MEASUREMENT_QUEUE_MAX 32u
/* 调用方提供互斥锁；有限队列保留未消费近距观测，所有条目使用来源时间。 */

typedef struct object_measurement_mailbox {object_target_observation_v2 items[OBJECT_MEASUREMENT_QUEUE_MAX];unsigned count;lab_real_context context;uint64_t epoch,last_source,dropped;int track_id,bound;} object_measurement_mailbox;
static void object_measurement_bind(object_measurement_mailbox*q,const lab_real_context*c,uint64_t epoch,int id){
 if(!q->bound||memcmp(&q->context,c,sizeof(*c))||q->epoch!=epoch||q->track_id!=id){uint64_t dropped=q->dropped;memset(q,0,sizeof(*q));q->dropped=dropped;q->context=*c;q->epoch=epoch;q->track_id=id;q->bound=1;}
}
static int object_measurement_close(const object_target_observation_v2*m){return m->surface_valid&&m->surface_gap_m>=0&&m->surface_gap_m<=.23;}
static void object_measurement_push(object_measurement_mailbox*q,const object_target_observation_v2*m){
 if(!m->valid||!q->bound||memcmp(&q->context,&m->context,sizeof(m->context))||q->epoch!=m->epoch||q->track_id!=m->track_id||m->source_us<=q->last_source)return;q->last_source=m->source_us;
 if(q->count==OBJECT_MEASUREMENT_QUEUE_MAX){unsigned drop=0;while(drop<q->count&&object_measurement_close(&q->items[drop]))drop++;if(drop==q->count)drop=1;memmove(q->items+drop,q->items+drop+1,(q->count-drop-1)*sizeof(*m));q->count--;q->dropped++;}
 q->items[q->count++]=*m;
}
static int object_measurement_pop(object_measurement_mailbox*q,object_target_observation_v2*out){
 if(!q->count)return 0;unsigned take=0;for(unsigned i=0;i<q->count;i++)if(object_measurement_close(&q->items[i])){take=i;break;}
 *out=q->items[take];/* 较旧的远距离不能覆盖已排队的近距离测量。 */
 unsigned consumed=take+1;memmove(q->items,q->items+consumed,(q->count-consumed)*sizeof(*out));q->count-=consumed;return 1;
}
#endif
