/* License and attribution: LICENSES/source_notices/notice_012.txt */








/* License and attribution: LICENSES/source_notices/notice_010.txt */
























#pragma once
namespace rp{ namespace hal{ 

class Locker
{
public:
    enum LOCK_STATUS
    {
        LOCK_OK = 1,
        LOCK_TIMEOUT = -1,
        LOCK_FAILED = 0
    };

    Locker(bool recusive = false){
#ifdef _WIN32
        _lock = NULL;
#endif
        init(recusive);
    }

    ~Locker()
    {
        release();
    }

    Locker::LOCK_STATUS lock(unsigned long timeout = 0xFFFFFFFF)
    {
#ifdef _WIN32
        switch (WaitForSingleObject(_lock, timeout==0xFFFFFFF?INFINITE:(DWORD)timeout))
        {
        case WAIT_ABANDONED:
            return LOCK_FAILED;
        case WAIT_OBJECT_0:
            return LOCK_OK;
        case WAIT_TIMEOUT:
            return LOCK_TIMEOUT;
        }

#else
#ifdef _MACOS
        if (timeout !=0 ) {
            if (pthread_mutex_lock(&_lock) == 0) return LOCK_OK;
        }
#else
        if (timeout == 0xFFFFFFFF){
            if (pthread_mutex_lock(&_lock) == 0) return LOCK_OK;
        }
#endif
        else if (timeout == 0)
        {
            if (pthread_mutex_trylock(&_lock) == 0) return LOCK_OK;
        }
#ifndef _MACOS
        else
        {
            timespec wait_time;
            timeval now;
            gettimeofday(&now,NULL);

            wait_time.tv_sec = timeout/1000 + now.tv_sec;
            wait_time.tv_nsec = (timeout%1000)*1000000 + now.tv_usec*1000;
        
            if (wait_time.tv_nsec >= 1000000000)
            {
               ++wait_time.tv_sec;
               wait_time.tv_nsec -= 1000000000;
            }
            switch (pthread_mutex_timedlock(&_lock,&wait_time))
            {
            case 0:
                return LOCK_OK;
            case ETIMEDOUT:
                return LOCK_TIMEOUT;
            }
        }
#endif
#endif

        return LOCK_FAILED;
    }


    void unlock()
    {
#ifdef _WIN32
        if (_recusive) {
            ReleaseMutex(_lock);
        } else {
            ReleaseSemaphore(_lock, 1, NULL);
        }
#else
        pthread_mutex_unlock(&_lock);
#endif
    }

#ifdef _WIN32
    HANDLE getLockHandle()
    {
        return _lock;
    }
#else
    pthread_mutex_t *getLockHandle()
    {
        return &_lock;
    }
#endif



protected:
    void    init(bool recusive)
    {
        
#ifdef _WIN32
        if (_recusive = recusive) {
            _lock = CreateMutex(NULL, FALSE, NULL);
        } else {
            _lock = CreateSemaphore(NULL, 1, 1, NULL);
        }
#else
        
        if (recusive) {
            pthread_mutexattr_t attr;
            pthread_mutexattr_init(&attr);
            pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
            pthread_mutex_init(&_lock, &attr);
        } else {
            pthread_mutex_init(&_lock, NULL);
        }
#endif
    }

    void    release()
    {
        unlock(); //force unlock before release
#ifdef _WIN32

        if (_lock) CloseHandle(_lock);
        _lock = NULL;
#else
        pthread_mutex_destroy(&_lock);
#endif
    }

#ifdef _WIN32
    HANDLE  _lock;
    bool _recusive;
#else
    pthread_mutex_t _lock;
#endif
    
};

class AutoLocker
{
public :
    AutoLocker(Locker &l): _binded(l)
    {
        _binded.lock();
    }

    void forceUnlock() {
        _binded.unlock();
    }
    ~AutoLocker() {_binded.unlock();}
    Locker & _binded;
};


}}

