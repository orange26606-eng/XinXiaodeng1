#ifndef ACG_CAPTURE_QUEUE_H
#define ACG_CAPTURE_QUEUE_H
#include <array>
#include <cstddef>
#include <cstdint>

namespace acg {
/* 每个回调执行一次非阻塞操作。成功出队且编号已知的缓冲区由辅助模块持有，直至重新入队或将选定帧交给调用方。按出队顺序选择，避免受V4L2序号回绕影响；此处不读写像素。


 */
enum class CaptureIo {Frame, Again, Interrupted, Failed};
enum class CaptureDrainStatus {Ready, Empty, Stopped, Failed};
enum class CaptureDrainReason {None, InvalidConfig, DequeueFailed, InvalidIndex,
    DuplicateIndex, InvalidBytes, RequeueFailed};
struct CaptureFrame {
    unsigned index=0;
    size_t bytes_used=0;
    uint32_t sequence=0;
    bool error=false;
};
struct CaptureDrainStats {
    unsigned attempts=0,dequeued=0,requeued=0,older_dropped=0,error_dropped=0,
        interrupts=0,queue_failures=0;
    bool budget_exhausted=false;
};
struct CaptureDrainResult {
    CaptureDrainStatus status=CaptureDrainStatus::Empty;
    CaptureDrainReason reason=CaptureDrainReason::None;
    CaptureFrame frame{};
    CaptureDrainStats stats{};
};
inline const char*capture_drain_reason_text(CaptureDrainReason reason){
    switch(reason){
      case CaptureDrainReason::None:return "NONE";
      case CaptureDrainReason::InvalidConfig:return "INVALID_CONFIG";
      case CaptureDrainReason::DequeueFailed:return "DEQUEUE_FAILED";
      case CaptureDrainReason::InvalidIndex:return "INVALID_INDEX";
      case CaptureDrainReason::DuplicateIndex:return "DUPLICATE_INDEX";
      case CaptureDrainReason::InvalidBytes:return "INVALID_BYTES";
      case CaptureDrainReason::RequeueFailed:return "REQUEUE_FAILED";
    }
    return "UNKNOWN";
}

template<class Dequeue,class Requeue,class Stop>
CaptureDrainResult drain_capture(unsigned count,const size_t*lengths,size_t expected_bytes,
                                Dequeue&&dequeue,Requeue&&requeue,Stop&&stop,unsigned max_frames=0){
    CaptureDrainResult result;
    if(!lengths||!count||count>8||!expected_bytes||max_frames>count){
        result.status=CaptureDrainStatus::Failed;
        result.reason=CaptureDrainReason::InvalidConfig;
        return result;
    }
    for(unsigned i=0;i<count;++i)if(lengths[i]<expected_bytes){
        result.status=CaptureDrainStatus::Failed;
        result.reason=CaptureDrainReason::InvalidConfig;
        return result;
    }
    std::array<bool,8>owned{};
    std::array<CaptureFrame,8>frames{};
    int selected=-1;
    auto put=[&](unsigned index){
        /* 调用QBUF前先清除所有权标记。QBUF失败后不重试，设备端结果不确定，由调用方执行STREAMOFF。
 */
        owned[index]=false;
        if(!requeue(index)){++result.stats.queue_failures;return false;}
        ++result.stats.requeued;
        return true;
    };
    auto cleanup=[&]{for(unsigned i=0;i<count;++i)if(owned[i])put(i);selected=-1;};
    auto fail=[&](CaptureDrainReason reason){
        result.status=CaptureDrainStatus::Failed;result.reason=reason;
        cleanup();return result;
    };
    const unsigned frame_limit=max_frames?max_frames:count;
    const unsigned attempt_limit=frame_limit*2;
    while(result.stats.dequeued<frame_limit&&result.stats.attempts<attempt_limit){
        if(stop()){
            result.status=CaptureDrainStatus::Stopped;
            cleanup();
            if(result.stats.queue_failures){result.status=CaptureDrainStatus::Failed;result.reason=CaptureDrainReason::RequeueFailed;}
            return result;
        }
        CaptureFrame frame;
        ++result.stats.attempts;
        CaptureIo io=dequeue(frame);
        if(io==CaptureIo::Again)break;
        if(io==CaptureIo::Interrupted){++result.stats.interrupts;continue;}
        if(io!=CaptureIo::Frame)return fail(CaptureDrainReason::DequeueFailed);
        ++result.stats.dequeued;
        if(frame.index>=count)return fail(CaptureDrainReason::InvalidIndex);
        if(owned[frame.index])return fail(CaptureDrainReason::DuplicateIndex);
        owned[frame.index]=true;frames[frame.index]=frame;
        /* 错误帧载荷可能为零，仅归还已知编号，不读取像素，并保留此前有效选择。
 */
        if(frame.error){
            ++result.stats.error_dropped;
            if(!put(frame.index))return fail(CaptureDrainReason::RequeueFailed);
            continue;
        }
        if(frame.bytes_used<expected_bytes||frame.bytes_used>lengths[frame.index])
            return fail(CaptureDrainReason::InvalidBytes);
        if(selected>=0){
            ++result.stats.older_dropped;
            if(!put((unsigned)selected))return fail(CaptureDrainReason::RequeueFailed);
        }
        selected=(int)frame.index;
    }
    result.stats.budget_exhausted=result.stats.dequeued==frame_limit||result.stats.attempts==attempt_limit;
    if(stop()){
        result.status=CaptureDrainStatus::Stopped;cleanup();
        if(result.stats.queue_failures){result.status=CaptureDrainStatus::Failed;result.reason=CaptureDrainReason::RequeueFailed;}
        return result;
    }
    if(selected>=0){result.status=CaptureDrainStatus::Ready;result.frame=frames[(unsigned)selected];}
    return result;
}
}
#endif
