/*
 *  @author KKest
 *		@created 19.01.2022
 *	
 * Types for rpLidar library
 *
 */

#ifndef rplidarTypes_h
#define rplidarTypes_h

#include "Arduino.h"

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
#define DEFAULT_TIMEOUT 2000

#define  MAX_SCAN_NODES  (512)

#define SL_LIDAR_STATUS_OK                 0x0
#define SL_LIDAR_STATUS_WARNING            0x1
#define SL_LIDAR_STATUS_ERROR              0x2

#define SL_LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2

#define SL_LIDAR_RESP_HQ_FLAG_SYNCBIT               (0x1<<0)

#define SL_LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1


// Definition of the variable bit scale encoding mechanism
#define SL_LIDAR_VARBITSCALE_X2_SRC_BIT  9
#define SL_LIDAR_VARBITSCALE_X4_SRC_BIT  11
#define SL_LIDAR_VARBITSCALE_X8_SRC_BIT  12
#define SL_LIDAR_VARBITSCALE_X16_SRC_BIT 14

#define SL_LIDAR_VARBITSCALE_X2_DEST_VAL 512
#define SL_LIDAR_VARBITSCALE_X4_DEST_VAL 1280
#define SL_LIDAR_VARBITSCALE_X8_DEST_VAL 1792
#define SL_LIDAR_VARBITSCALE_X16_DEST_VAL 3328

#define SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_1               0xA
#define SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_2               0x5

#define SL_LIDAR_RESP_MEASUREMENT_HQ_SYNC                  0xA5

#define SL_LIDAR_RESP_MEASUREMENT_EXP_SYNCBIT              (0x1<<15)

#define SL_LIDAR_VARBITSCALE_GET_SRC_MAX_VAL_BY_BITS(_BITS_) \
    (  (((0x1<<(_BITS_)) - SL_LIDAR_VARBITSCALE_X16_DEST_VAL)<<4) + \
       ((SL_LIDAR_VARBITSCALE_X16_DEST_VAL - SL_LIDAR_VARBITSCALE_X8_DEST_VAL)<<3) + \
       ((SL_LIDAR_VARBITSCALE_X8_DEST_VAL - SL_LIDAR_VARBITSCALE_X4_DEST_VAL)<<2) + \
       ((SL_LIDAR_VARBITSCALE_X4_DEST_VAL - SL_LIDAR_VARBITSCALE_X2_DEST_VAL)<<1) + \
       SL_LIDAR_VARBITSCALE_X2_DEST_VAL - 1)

typedef uint8_t rp_descriptor_t[7];
typedef uint8_t rq_Packet_t[9];
typedef uint8_t rq_message_t[2];
typedef uint32_t sl_result;
typedef uint8_t sl_u8;
typedef uint16_t sl_u16;
typedef uint32_t sl_u32;
/// hold a measure point for standard scan mode
typedef struct scanDataPoint
{
	uint8_t quality;
	uint8_t angle_low;
	uint8_t angle_high;
	uint8_t distance_low;
	uint8_t distance_high;
}stScanDataPoint_t;

typedef struct _sl_lidar_response_ultra_cabin_nodes_t
{
    // 31                                              0
    // | predict2 10bit | predict1 10bit | major 12bit |
    sl_u32 combined_x3;
} __attribute__((packed)) sl_lidar_response_ultra_cabin_nodes_t;

typedef struct _sl_lidar_response_ultra_capsule_measurement_nodes_t
{
    sl_u8                             s_checksum_1; // see [s_checksum_1]
    sl_u8                             s_checksum_2; // see [s_checksum_1]
    sl_u16                            start_angle_sync_q6;
    sl_lidar_response_ultra_cabin_nodes_t  ultra_cabins[32];
} sl_lidar_response_ultra_capsule_measurement_nodes_t;

typedef struct  _sl_lidar_response_measurement_node_hq_t
{
    sl_u16   angle_z_q14;
    sl_u32   dist_mm_q2;
    sl_u8    quality;
    sl_u8    flag;
} __attribute__((packed)) sl_lidar_response_measurement_node_hq_t;

struct scanDot {
    sl_u8   quality;
    float angle;
    float dist;
};
typedef struct Point
{
	double angle; 
	uint16_t distance; 
}point_t;

typedef struct deviceHealtStatus
{
	uint8_t status;
	uint8_t errorCode_low;
	uint8_t errorCode_high;
}stDeviceStatus_t;


typedef struct expressData
{
	uint16_t angle;
	uint16_t cabin[40];
}stExpressDataPacket_t;

typedef struct expressDataStorage
{
	uint8_t angle_low;
	uint8_t angle_high;
	uint16_t distance;
}expressDataStorage_t;

typedef struct rp_stDeviceInfo
{
  uint8_t model;
  uint8_t firmware_minor;
  uint8_t firmware_major;
  uint8_t hardware;
  uint8_t serialnumber[16];
}stDeviceInfo_t;

enum enDescriptor
{
		legacyVersion,  ///< Legacy scan version
		extendedVersion, ///< Extendet scan version
		denseVersion,	 ///< Dense scan version
		startScan,		 ///< start scan
		forceScan,		 ///< force to scan in idle mode
		deviceInfo,		 ///< deviceInfo of the Lidar
		healthInfo,		 ///< Error Codes and Status
		sampleRate,		 ///< momentary sampleRate
		deviceConf,
    ultradense		 
};
typedef struct _sl_lidar_ans_header_t
{
    sl_u8  syncByte1; // must be SL_LIDAR_ANS_SYNC_BYTE1
    sl_u8  syncByte2; // must be SL_LIDAR_ANS_SYNC_BYTE2
    sl_u32 size_q30_subtype; // see _u32 size:30; _u32 subType:2;
    sl_u8  type;
} __attribute__((packed)) sl_lidar_ans_header_t;
enum enRequest
{
	rq_stop,
	rq_reset,
	rq_scan,
	rq_scanExpress,
	rq_scanForce,
	rq_info,
	rq_health,
	rq_sampleRate,
	rq_deviceConf
};

enum enMode
{
	stop,
	standard,
	express
};

extern rp_descriptor_t resp_descriptor[];	///< List of Response descriptors 
extern rq_message_t req_message[];			///< List of Request Messages
extern rq_Packet_t req_Express[];			///< Request Message for Express Scan Modes
#endif
