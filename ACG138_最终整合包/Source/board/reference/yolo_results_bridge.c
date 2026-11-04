/* 实时检测结果适配器，在私有PTY中运行现有YOLO程序以获得逐行标准输出。只管理该子进程组，不改推理或视频。原子发布最新完整检测快照，诊断日志有容量上限。


 */
#define _GNU_SOURCE
#include "dashboard_detections.h"
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
static volatile sig_atomic_t stopping;
static void stop(int sig) { stopping=sig; }
static uint64_t now_ms(void)
{ struct timespec t; if(clock_gettime(CLOCK_MONOTONIC,&t)) return 0; return (uint64_t)t.tv_sec*1000+(uint64_t)t.tv_nsec/1000000; }
static void terminate_child(pid_t pid,int sig)
{ if(kill(-pid,sig) && errno==ESRCH) (void)kill(pid,sig); }
int main(int argc,char **argv)
{
    const char *snapshot=NULL,*logpath=NULL; int command=0;
    for(int i=1;i<argc;++i) {
        if(!strcmp(argv[i],"--snapshot") && i+1<argc) snapshot=argv[++i];
        else if(!strcmp(argv[i],"--log") && i+1<argc) logpath=argv[++i];
        else if(!strcmp(argv[i],"--") && i+1<argc) { command=i+1;break; }
        else { fprintf(stderr,"Usage: %s --snapshot PATH --log PATH -- YOLO [arguments]\n",argv[0]);return !strcmp(argv[i],"--help") ? 0 : 2; }
    }
    if(!snapshot || !logpath || !command || strlen(snapshot)>4000) return 2;
    FILE *log=fopen(logpath,"w"); if(!log) { perror("YOLO log");return 1; }
    (void)fcntl(fileno(log),F_SETFD,FD_CLOEXEC);
    (void)remove(snapshot);
    int master=posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    if(master<0 || grantpt(master) || unlockpt(master)) { perror("YOLO PTY");if(master>=0)close(master);fclose(log);return 1; }
    char *name=ptsname(master);
    int slave=name ? open(name,O_RDWR|O_NOCTTY|O_CLOEXEC) : -1;
    if(slave<0) { perror("YOLO PTY slave");close(master);fclose(log);return 1; }
    signal(SIGINT,stop);signal(SIGTERM,stop);signal(SIGHUP,stop);
    pid_t child=fork();
    if(child<0) { perror("YOLO fork");close(slave);close(master);fclose(log);return 1; }
    if(!child) {
        signal(SIGINT,SIG_DFL);signal(SIGTERM,SIG_DFL);signal(SIGHUP,SIG_DFL);
        if(setsid()<0) _exit(126);
        (void)ioctl(slave,TIOCSCTTY,0);
        int input=open("/dev/null",O_RDONLY);
        if(input<0 || dup2(input,0)<0 || dup2(slave,1)<0 || dup2(slave,2)<0) _exit(126);
        if(input>2) close(input);
        if(slave>2) close(slave);
        close(master);
        execvp(argv[command],&argv[command]);perror("YOLO exec");_exit(127);
    }
    close(slave);
    struct dashboard_detection_parser parser;dashboard_detections_init(&parser);
    size_t logged=0;int failed=0,ended=0,status=0,eof=0;
    while(!stopping && !failed && !eof) {
        struct pollfd p={.fd=master,.events=POLLIN};
        int rc=poll(&p,1,100);
        if(rc<0) { if(errno==EINTR)continue;perror("YOLO output poll");failed=1;break; }
        if(p.revents & POLLNVAL) {failed=1;break;}
        if(p.revents & (POLLIN|POLLHUP|POLLERR)) {
            for(unsigned block=0;block<16;++block) {
                char bytes[8192];ssize_t n=read(master,bytes,sizeof(bytes));
                if(n<0) {
                    if(errno==EINTR)continue;
                    if(errno==EAGAIN)break;
                    if(errno!=EIO){perror("YOLO output read");failed=1;}
                    eof=1;break;
                }
                if(!n){eof=1;break;}
                if(logged+(size_t)n>4u*1024u*1024u) {
                    if(fflush(log) || ftruncate(fileno(log),0) || fseek(log,0,SEEK_SET)){failed=1;break;}
                    logged=0;
                }
                if(fwrite(bytes,1,(size_t)n,log)!=(size_t)n || fflush(log)){failed=1;break;}
                logged+=(size_t)n;
                if(dashboard_detections_feed(&parser,bytes,(size_t)n,now_ms()) &&
                   dashboard_detections_write(snapshot,&parser.latest)) {
                    perror("YOLO result snapshot");failed=1;break;
                }
            }
        }
        if(!ended) {
            pid_t rcwait=waitpid(child,&status,WNOHANG);
            if(rcwait==child) ended=1;
            else if(rcwait<0 && errno!=EINTR){failed=1;ended=1;}
        }
        if(ended && !rc) eof=1;
    }
    if(!ended) {
        terminate_child(child,SIGTERM);
        uint64_t deadline=now_ms()+2000;
        while(now_ms()<deadline) {
            pid_t rc=waitpid(child,&status,WNOHANG);
            if(rc==child || (rc<0 && errno==ECHILD)){ended=1;break;}
            (void)poll(NULL,0,20);
        }
        if(!ended) {terminate_child(child,SIGKILL);while(waitpid(child,&status,0)<0 && errno==EINTR){};}
    }
    close(master);if(fclose(log))failed=1;
    (void)remove(snapshot);
    char temporary[4096];snprintf(temporary,sizeof(temporary),"%s.tmp",snapshot);(void)remove(temporary);
    if(stopping)return 128+stopping;
    if(failed)return 1;
    return WIFEXITED(status) ? WEXITSTATUS(status) : WIFSIGNALED(status) ? 128+WTERMSIG(status) : 1;
}
