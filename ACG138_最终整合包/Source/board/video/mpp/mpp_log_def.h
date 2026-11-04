/* License and attribution: LICENSES/source_notices/notice_005.txt */

#ifndef __MPP_LOG_DEF_H__
#define __MPP_LOG_DEF_H__

#ifdef __cplusplus
extern "C" {
#endif

#define MPP_LOG_UNKNOWN     0   /* internal use only                    */
#define MPP_LOG_FATAL       1   /* fatal error on aborting              */
#define MPP_LOG_ERROR       2   /* error log on unrecoverable failures  */
#define MPP_LOG_WARN        3   /* warning log on recoverable failures  */
#define MPP_LOG_INFO        4   /* Informational log                    */
#define MPP_LOG_DEBUG       5   /* Debug log                            */
#define MPP_LOG_VERBOSE     6   /* Verbose log                          */
#define MPP_LOG_SILENT      7   /* internal use only                    */

#ifdef __cplusplus
}
#endif

#endif /*__MPP_LOG_DEF_H__*/
