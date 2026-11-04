/* License and attribution: LICENSES/source_notices/notice_012.txt */








/* License and attribution: LICENSES/source_notices/notice_010.txt */
























#pragma once

#include "hal/types.h"
#define CLASS_THREAD(c , x ) \
	rp::hal::Thread::create_member<c, &c::x>(this )

namespace rp{ namespace hal{

class Thread
{
public:
    enum priority_val_t
	{
		PRIORITY_REALTIME = 0,
		PRIORITY_HIGH     = 1,
		PRIORITY_NORMAL   = 2,
		PRIORITY_LOW      = 3,
		PRIORITY_IDLE     = 4,
	};

    template <class T, u_result (T::*PROC)(void)>
    static Thread create_member(T * pthis)
    {
		return create(_thread_thunk<T,PROC>, pthis);
	}

	template <class T, u_result (T::*PROC)(void) >
	static _word_size_t THREAD_PROC _thread_thunk(void * data)
	{
		return (static_cast<T *>(data)->*PROC)();
	}
	static Thread create(thread_proc_t proc, void * data = NULL );

public:
    ~Thread() { }
    Thread():  _data(NULL),_func(NULL),_handle(0)  {}
    _word_size_t getHandle(){ return _handle;}
    u_result terminate();
    void *getData() { return _data;}
    u_result join(unsigned long timeout = -1);
	
    // disabled as on platforms like Linux, the priority will be inherited by the child thread
    // which may caused unexpected behavior.
    // Please using Thread::SetSelfPriority instead
    // u_result setPriority( priority_val_t p);
	priority_val_t getPriority();

    static u_result SetSelfPriority(priority_val_t p);


    bool operator== ( const Thread & right) { return this->_handle == right._handle; }
protected:
    Thread( thread_proc_t proc, void * data ): _data(data),_func(proc), _handle(0)  {}
    void * _data;
    thread_proc_t _func;
    _word_size_t _handle;
};

}}

