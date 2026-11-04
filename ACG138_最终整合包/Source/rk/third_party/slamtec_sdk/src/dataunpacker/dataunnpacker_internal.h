/* License and attribution: LICENSES/source_notices/notice_025.txt */







 /*
  *  Sample Data Unpacker System
  *  Internal Definition
  */

  /* License and attribution: LICENSES/source_notices/notice_026.txt */
























#pragma once

BEGIN_DATAUNPACKER_NS()


class LIDARSampleDataUnpackerInner: public LIDARSampleDataUnpacker
{
public:
	LIDARSampleDataUnpackerInner(LIDARSampleDataListener& l): LIDARSampleDataUnpacker(l){}

	virtual void publishHQNode(_u64 timestamp_uS, const rplidar_response_measurement_node_hq_t* node) = 0;
	virtual void publishDecodingErrorMsg(int errorType, _u8 ansType, const void* payload, size_t size) = 0;
	virtual void publishCustomData(_u8 ansType, _u32 customCode, const void* payload, size_t size) = 0;
	virtual void publishNewScanReset() = 0;


	virtual _u64 getCurrentTimestamp_uS() = 0;

};

class IDataUnpackerHandler
{
public:
	IDataUnpackerHandler() {}
	virtual ~IDataUnpackerHandler() {}


	virtual void onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size) = 0;

	virtual _u8 getSampleAnswerType() const = 0;
	virtual void onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t size) = 0;
	virtual void reset() = 0;

};

END_DATAUNPACKER_NS()