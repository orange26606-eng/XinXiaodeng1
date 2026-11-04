/* License and attribution: LICENSES/source_notices/notice_020.txt */







/* License and attribution: LICENSES/source_notices/notice_013.txt */
























#pragma once

#ifdef __cplusplus
#include <cstdint>

#define SL_DEFINE_TYPE(IntType, NewType)    typedef std::IntType NewType
#else
#include <stdint.h>

#define SL_DEFINE_TYPE(IntType, NewType)    typedef IntType NewType
#endif

#define SL_DEFINE_INT_TYPE(Bits) \
    SL_DEFINE_TYPE(int ## Bits ## _t, sl_s ## Bits); \
    SL_DEFINE_TYPE(uint ## Bits ## _t, sl_u ## Bits); \

SL_DEFINE_INT_TYPE(8)
SL_DEFINE_INT_TYPE(16)
SL_DEFINE_INT_TYPE(32)
SL_DEFINE_INT_TYPE(64)

#if !defined(__GNUC__) && !defined(__attribute__)
#   define __attribute__(x)
#endif

#ifdef WIN64
typedef sl_u64          sl_word_size_t;
#elif defined(WIN32)
typedef sl_u32          sl_word_size_t;
#elif defined(__GNUC__)
typedef unsigned long   sl_word_size_t;
#elif defined(__ICCARM__)
typedef sl_u32          sl_word_size_t;
#endif

typedef uint32_t sl_result;

#define SL_RESULT_OK                     (sl_result)0
#define SL_RESULT_FAIL_BIT               (sl_result)0x80000000
#define SL_RESULT_ALREADY_DONE           (sl_result)0x20
#define SL_RESULT_INVALID_DATA           (sl_result)(0x8000 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_OPERATION_FAIL         (sl_result)(0x8001 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_OPERATION_TIMEOUT      (sl_result)(0x8002 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_OPERATION_STOP         (sl_result)(0x8003 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_OPERATION_NOT_SUPPORT  (sl_result)(0x8004 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_FORMAT_NOT_SUPPORT     (sl_result)(0x8005 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_INSUFFICIENT_MEMORY    (sl_result)(0x8006 | SL_RESULT_FAIL_BIT)

#define SL_IS_OK(x)    ( ((x) & SL_RESULT_FAIL_BIT) == 0 )
#define SL_IS_FAIL(x)  ( ((x) & SL_RESULT_FAIL_BIT) )
