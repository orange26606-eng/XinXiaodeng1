/* License and attribution: LICENSES/source_notices/notice_025.txt */







 /*
  *  Sample Data Unpacker System
  *  Capsule Style Sample Node Handlers
  */

  /* License and attribution: LICENSES/source_notices/notice_026.txt */
























#pragma once

BEGIN_DATAUNPACKER_NS()

namespace unpacker {

class UnpackerHandler_CapsuleNode : public IDataUnpackerHandler {
public:
	UnpackerHandler_CapsuleNode();
	virtual ~UnpackerHandler_CapsuleNode();

	virtual _u8 getSampleAnswerType() const;
	virtual void onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t size);
	virtual void reset();
	virtual void onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size);
protected:

	void _onScanNodeCapsuleData(rplidar_response_capsule_measurement_nodes_t &, LIDARSampleDataUnpackerInner* engine);

	std::vector<_u8> _cached_scan_node_buf;
	int              _cached_scan_node_buf_pos;
	bool             _is_previous_capsuledataRdy;

	rplidar_response_capsule_measurement_nodes_t _cached_previous_capsuledata;
	_u64             _cached_last_data_timestamp_us;

	SlamtecLidarTimingDesc _cachedTimingDesc;
};

class UnpackerHandler_UltraCapsuleNode : public IDataUnpackerHandler {
public:
	UnpackerHandler_UltraCapsuleNode();
	virtual ~UnpackerHandler_UltraCapsuleNode();

	virtual _u8 getSampleAnswerType() const;
	virtual void onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t size);
	virtual void reset();
	virtual void onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size);
protected:
	void _onScanNodeUltraCapsuleData(rplidar_response_ultra_capsule_measurement_nodes_t&, LIDARSampleDataUnpackerInner* engine);


	std::vector<_u8> _cached_scan_node_buf;
	int              _cached_scan_node_buf_pos;
	bool             _is_previous_capsuledataRdy;

	rplidar_response_ultra_capsule_measurement_nodes_t _cached_previous_ultracapsuledata;
	_u64             _cached_last_data_timestamp_us;

	SlamtecLidarTimingDesc _cachedTimingDesc;

};



class UnpackerHandler_DenseCapsuleNode : public IDataUnpackerHandler {
public:
	UnpackerHandler_DenseCapsuleNode();
	virtual ~UnpackerHandler_DenseCapsuleNode();

	virtual _u8 getSampleAnswerType() const;
	virtual void onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t size);
	virtual void reset();
	virtual void onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size);
protected:
	void _onScanNodeDenseCapsuleData(rplidar_response_dense_capsule_measurement_nodes_t&, LIDARSampleDataUnpackerInner* engine);


	std::vector<_u8> _cached_scan_node_buf;
	int              _cached_scan_node_buf_pos;
	bool             _is_previous_capsuledataRdy;

	rplidar_response_dense_capsule_measurement_nodes_t _cached_previous_dense_capsuledata;
	_u64             _cached_last_data_timestamp_us;

	SlamtecLidarTimingDesc _cachedTimingDesc;

};


class UnpackerHandler_UltraDenseCapsuleNode : public IDataUnpackerHandler {
public:
	UnpackerHandler_UltraDenseCapsuleNode();
	virtual ~UnpackerHandler_UltraDenseCapsuleNode();

	virtual _u8 getSampleAnswerType() const;
	virtual void onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t size);
	virtual void reset();
	virtual void onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size);
protected:
	void _onScanNodeUltraDenseCapsuleData(rplidar_response_ultra_dense_capsule_measurement_nodes_t&, LIDARSampleDataUnpackerInner* engine);

	std::vector<_u8> _cached_scan_node_buf;
	int              _cached_scan_node_buf_pos;
	bool             _is_previous_capsuledataRdy;

	rplidar_response_ultra_dense_capsule_measurement_nodes_t _cached_previous_ultra_dense_capsuledata;
	_u64             _cached_last_data_timestamp_us;



	int              _last_node_sync_bit;
	int              _last_dist_q2;

	SlamtecLidarTimingDesc _cachedTimingDesc;
};


}

END_DATAUNPACKER_NS()