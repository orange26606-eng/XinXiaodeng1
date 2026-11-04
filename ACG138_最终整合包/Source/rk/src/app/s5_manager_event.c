/* 监督器在控制进程之外串行写诊断记录；fsync在记录线程执行。 */


#define _GNU_SOURCE
#define S5_LOG_SOFTWARE_POLICY 8u
#include "s5_persistent_log.h"
static int token(const char*s){if(!s||!*s||strlen(s)>96)return 0;for(;*s;s++)if(!((*s>='A'&&*s<='Z')||(*s>='0'&&*s<='9')||*s=='_'))return 0;return 1;}
int main(int argc,char**argv){
 if(argc!=4||!token(argv[1])||!token(argv[2]))return 2;
 char*end=NULL;errno=0;long code=strtol(argv[3],&end,10);if(errno||!end||*end||code< -1||code>255)return 2;
 const char*base=getenv("ACG_S5_LOG_DIR");if(!base||strlen(base)>440)return 3;
 char directory[512];snprintf(directory,sizeof(directory),"%s/manager",base);
 s5_log*l=s5_log_open(directory,getenv("ACG_S5_FS_SHA256"));if(!l)return 3;
 char snapshot[256];int n=snprintf(snapshot,sizeof(snapshot),"{\"module_or_phase\":\"%s\",\"exit_code\":%ld,\"exit_code_known\":%s}",argv[2],code,code<0?"false":"true");
 int ok=n>0&&(size_t)n<sizeof(snapshot)&&s5_log_submit(l,s5_log_clock(),argv[1],snapshot,(size_t)n,1);
 s5_log_close(l);return ok?0:4;
}
