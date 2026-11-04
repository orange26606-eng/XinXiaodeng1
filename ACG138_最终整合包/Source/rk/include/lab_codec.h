#ifndef ACG_MAPPING_LAB_CODEC_H
#define ACG_MAPPING_LAB_CODEC_H
#include "lab_types.h"
#ifdef __cplusplus
extern "C" {
#endif
#define LAB_FILE_HEADER_BYTES 64u
#define LAB_EVENT_HEADER_BYTES 80u
#define LAB_FILE_FOOTER_BYTES 32u
typedef struct lab_file_header {
 uint32_t metadata_bytes,metadata_crc32,flags,mode;
 uint64_t created_unix_us,recording_id,timeline_start_us;
} lab_file_header;
typedef struct lab_file_footer {uint64_t event_count;uint32_t stream_crc32,flags;} lab_file_footer;
enum lab_footer_flags {LAB_FOOTER_CLOSED=1u,LAB_FOOTER_CAPTURE_LOSS=2u};
/* 标准CRC-32/ISO-HDLC，扩展计算接收上一段已完成CRC，初值0。 */
uint32_t lab_crc32_extend(uint32_t previous,const void *data,size_t bytes);
uint32_t lab_crc32(const void *data,size_t bytes);
lab_status lab_file_header_encode(uint8_t out[LAB_FILE_HEADER_BYTES],const lab_file_header *header);
lab_status lab_file_header_decode(lab_file_header *out,const uint8_t *data,size_t bytes);
lab_status lab_file_footer_encode(uint8_t out[LAB_FILE_FOOTER_BYTES],const lab_file_footer *footer);
lab_status lab_file_footer_decode(lab_file_footer *out,const uint8_t *data,size_t bytes);
size_t lab_event_payload_size(const lab_event *event);
lab_status lab_event_encode(uint8_t header[LAB_EVENT_HEADER_BYTES],uint8_t *payload,
 size_t capacity,size_t *payload_bytes,const lab_event *event);
/* 分配或读取载荷前核对文件头、CRC及载荷长度。 */
lab_status lab_event_header_check(const uint8_t *header,size_t bytes,size_t *payload_bytes);
lab_status lab_event_decode(lab_event *out,const uint8_t *header,size_t header_bytes,
 const uint8_t *payload,size_t payload_bytes);
#ifdef __cplusplus
}
#endif
#endif
