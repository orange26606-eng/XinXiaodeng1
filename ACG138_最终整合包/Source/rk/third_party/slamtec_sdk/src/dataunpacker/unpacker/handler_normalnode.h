/* License and attribution: LICENSES/source_notices/notice_025.txt */







 /*
  *  Sample Data Unpacker System
  *  Normal Sample Node Handler
  */

  /* License and attribution: LICENSES/source_notices/notice_026.txt */
























#pragma once

BEGIN_DATAUNPACKER_NS()
	
namespace unpacker{

class UnpackerHandler_NormalNode : public IDataUnpackerHandler {
public:
	UnpackerHandler_NormalNode();
	virtual ~UnpackerHandler_NormalNode();

	virtual _u8 getSampleAnswerType() const;
	virtual void onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t size);
	virtual void reset();
	virtual void onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size);
protected:
	std::vector<_u8> _cached_scan_node_buf;
	int              _cached_scan_node_buf_pos;

	SlamtecLidarTimingDesc _cachedTimingDesc;
};

}

END_DATAUNPACKER_NS()