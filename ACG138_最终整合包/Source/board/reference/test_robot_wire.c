#include "robot_protocol.h"
#include <assert.h>
#include <stdio.h>
int main(int argc,char **argv)
{
    assert(argc==2);
    unsigned char p[32];FILE *f=fopen(argv[1],"rb");assert(f);
    assert(fread(p,1,32,f)==32 && fgetc(f)==EOF);fclose(f);
    struct robot_model m={0};assert(robot_model_push(&m,p,32,100)==1);
    assert(m.valid==1 && m.mm[0]==1234 && m.mm[1]==4500 && m.mm[2]==0);
    assert(m.error[0]==0 && m.error[1]==0 && m.error[2]==2);
    assert(m.auto_mode==1 && m.state==7 && m.stop_reason==3);
    assert(robot_distance_state(&m,0,100)==ROBOT_DISTANCE);
    assert(robot_distance_state(&m,1,100)==ROBOT_MAXIMUM);
    assert(robot_distance_state(&m,2,100)==ROBOT_INVALID);
    puts("PASS RTL-to-host: real simulated Ethernet payload decodes as 123.4cm, 450cm MAX, invalid ERR=2");
    return 0;
}
