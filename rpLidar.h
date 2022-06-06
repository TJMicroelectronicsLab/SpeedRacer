/*
 *  @author KKest
 *		@created 10.01.2022
 *	
 * Library to control an rpLidar S2
 *
 */
 
#ifndef rplidar_h
#define rplidar_h

#include "rpLidarTypes.h"

class rpLidar{
	public:
	
	/**
	 * Construcor of Class
	 *
	 * @param pointer to used USART
	 * @param Baudrate
	 */
	rpLidar(HardwareSerial *_serial,uint32_t baud, int rx, int tx);

	/**
	 * Gets the device info from rpLidar
	 *
	 *	It hold´s the Model no. , firmware and software vers.
	 *	and the serialnumber
	 *
	 * @return requested Device Info 
	 */
	stDeviceInfo_t getDeviceInfo();

	/**
	 * Gets the Status from rpLidar
	 *	u8 Status
	 *	u16 Errorcode
	 *
	 * @return Status
	 */	
	stDeviceStatus_t getDeviceHealth();
	
	/**
	 * Resets the lidar 
	 *
	 * should be executed if an error happend
	 */	
	void resetDevice();
	
	/**
	 * Stops the lidar if its turning
	 * goes into idle state
	 */	
	void stopDevice();
	
	/**
	 * Starts the Lidar and its measurement system
	 * 
	 * @param modus to run the lidar
	 * @return true if mode started correctly, fals if not 
	 */	
	bool start(uint8_t _mode);
	
	/**
	 * should be excecuted as often as possible to read the data from USART
	 * 
	 * @return status 
	 */	
  sl_result cacheUltraCapsuledScanData();



	//Debug Funktionen
	void DebugPrintMeasurePoints(int16_t _count);	///< prints Standard Data in normal Format in Serial Monitor
	void DebugPrintDeviceErrorStatus(stDeviceStatus_t _status); ///< prints Status of lidar in Serial Monitor
	void DebugPrintDeviceInfo(stDeviceInfo_t _info);	///< prints Device Info in Serial Monitor
	void DebugPrintDescriptor(rp_descriptor_t _descriptor); ///< prints descriptor in Serial Monitor
	void DebugPrintBufferAsHex();				///< prints Standard Data as Hex splitted with ","  in Serial Monitor

	
	//point_t Data[1540]; ///< stores the raw scan data
	//stScanDataPoint_t DataBuffer[1500];	///<Storage to save the Data of a Standard Scan
  sl_lidar_response_measurement_node_hq_t   _cached_scan_node_hq_buf[MAX_SCAN_NODES];
  sl_lidar_response_ultra_capsule_measurement_nodes_t _cached_previous_ultracapsuledata;
  bool                                         _is_previous_capsuledataRdy;
  size_t                                   _cached_scan_node_hq_count;
  sl_u8                                    _cached_capsule_flag;
  uint8_t                                 total_scan_count;
  SemaphoreHandle_t scan_mutex;
	private:
	
	//stExpressDataPacket_t ExpressDataBuffer[79];	///<Storge to save the Data of an Express Scan
	uint16_t interestAngleLeft;		///< left border of needed angle 180-360°
	uint16_t interestAngleRight;	///< right border of needed angle 0-180°

	uint8_t scanMode=stop; 			///< contains the actual scan mode of lidar
	bool status=false; 				///< contains the actual status of lidar
  bool _isScanning = true;
  sl_result _waitUltraCapsuledNode(sl_lidar_response_ultra_capsule_measurement_nodes_t & node, sl_u32 timeout = DEFAULT_TIMEOUT);
void _ultraCapsuleToNormal(const sl_lidar_response_ultra_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount);
	
	/**
	 * Compares two Response Descriptors 
	 * @returns true if equal 
	 */	
	bool compareDescriptor(rp_descriptor_t _descr1,rp_descriptor_t _descr2);
	
	/**
	 * 
	 * Should be the same as mySerial->flush()
	 * but worked not as expected at test time
	 */		
	void clearSerialBuffer();

	/**
	 * Checks if no Serial Data ist available in a given time
	 * 
	 * @param wait time ms 
	 * @param amount of expected bytes
	 * @return true if a timeout happend
	 */	
	bool checkForTimeout(uint32_t _time,size_t _size);
	

	/**
	 * 
	 * @return true if the motor is running
	 */	
	bool isRunning();
	
	/**
	 * 
	 * @return actual running scanmode
	 */	
	uint8_t isScanMode(); 
	
	HardwareSerial *serial;		///< pointer to HardwareSerial USART 
};



#endif
