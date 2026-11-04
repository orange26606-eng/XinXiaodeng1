#pragma once
#include <memory>
#include <mutex>
#include <condition_variable>
#include <cstdint>
namespace acg {
struct MailboxStats{uint64_t submitted=0,replaced=0,rejected=0,taken=0;};
template<class T>class LatestMailbox {
 std::mutex mutex;std::condition_variable cv;std::unique_ptr<T>pending;bool stopped=false;uint64_t newest=0;MailboxStats counters;
public:
 bool put(std::unique_ptr<T>p){std::lock_guard<std::mutex>lock(mutex);if(stopped||!p||!p->sequence||p->sequence<=newest){counters.rejected++;return false;}newest=p->sequence;counters.submitted++;if(pending)counters.replaced++;pending=std::move(p);cv.notify_one();return true;}
 bool take(std::unique_ptr<T>&p){std::unique_lock<std::mutex>lock(mutex);cv.wait(lock,[&]{return stopped||pending;});if(!pending)return false;p=std::move(pending);counters.taken++;return true;}
 void stop(){std::lock_guard<std::mutex>lock(mutex);stopped=true;cv.notify_all();}
 MailboxStats stats(){std::lock_guard<std::mutex>lock(mutex);return counters;}
};
}
