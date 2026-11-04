/* License and attribution: LICENSES/source_notices/notice_025.txt */







 /*
  *  Sample Data Unpacker System
  *
  */

  /* License and attribution: LICENSES/source_notices/notice_026.txt */

























#pragma once

#include "dataupacker_namespace.h"

BEGIN_DATAUNPACKER_NS()


class LIDARSampleDataListener
{


public:
	virtual void onHQNodeScanResetReq() = 0;
	virtual void onHQNodeDecoded(_u64 timestamp_uS, const rplidar_response_measurement_node_hq_t* node) = 0;
	virtual void onCustomSampleDataDecoded(_u8 ansType, _u32 customCode, const void* data, size_t size) {}

	virtual void onDecodingError(int errMsg, _u8 ansType, const void* payload, size_t size) {}
};

class LIDARSampleDataUnpacker
{
public:
	enum {
		ERR_EVENT_ON_EXP_ENCODER_RESET = 0x8001,
		ERR_EVENT_ON_EXP_CHECKSUM_ERR = 0x8002,
	};

	enum UnpackerContextType {
		UNPACKER_CONTEXT_TYPE_LIDAR_UNKNOWN = 0,
		UNPACKER_CONTEXT_TYPE_LIDAR_TIMING = 1,
		UNPACKER_CONTEXT_TYPE_TRIANGULATION_OPTICAL_FACTOR = 2,

	};

	virtual ~LIDARSampleDataUnpacker();
	static LIDARSampleDataUnpacker* CreateInstance(LIDARSampleDataListener& listener);
	static void ReleaseInstance(LIDARSampleDataUnpacker*);

	virtual void updateUnpackerContext(UnpackerContextType type, const void* data, size_t size) = 0;

	virtual void enable() = 0;
	virtual void disable() = 0;

	virtual bool onSampleData(_u8 ansType, const void* buffer, size_t size) = 0;
	virtual void reset() = 0;
	virtual void clearCache() = 0;

protected:
	LIDARSampleDataUnpacker(LIDARSampleDataListener&);
	LIDARSampleDataListener& _listener;

};

END_DATAUNPACKER_NS()