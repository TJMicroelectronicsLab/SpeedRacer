/*
 *  @author KKest
 *		@created 10.01.2022
 *
 * Library to control an rpLidar S2
 *
 */
#include "rpLidar.h"
#include "Arduino.h"

static sl_u32 _varbitscale_decode(sl_u32 scaled, sl_u32 & scaleLevel)
{
    static const sl_u32 VBS_SCALED_BASE[] = {
        SL_LIDAR_VARBITSCALE_X16_DEST_VAL,
        SL_LIDAR_VARBITSCALE_X8_DEST_VAL,
        SL_LIDAR_VARBITSCALE_X4_DEST_VAL,
        SL_LIDAR_VARBITSCALE_X2_DEST_VAL,
        0,
    };

    static const sl_u32 VBS_SCALED_LVL[] = {
        4,
        3,
        2,
        1,
        0,
    };

    static const sl_u32 VBS_TARGET_BASE[] = {
        (0x1 << SL_LIDAR_VARBITSCALE_X16_SRC_BIT),
        (0x1 << SL_LIDAR_VARBITSCALE_X8_SRC_BIT),
        (0x1 << SL_LIDAR_VARBITSCALE_X4_SRC_BIT),
        (0x1 << SL_LIDAR_VARBITSCALE_X2_SRC_BIT),
        0,
    };

    for (size_t i = 0; i < 5; ++i) {
        int remain = ((int)scaled - (int)VBS_SCALED_BASE[i]);
        if (remain >= 0) {
            scaleLevel = VBS_SCALED_LVL[i];
            return VBS_TARGET_BASE[i] + (remain << scaleLevel);
        }
    }
    return 0;
}
rpLidar::rpLidar(HardwareSerial *_mySerial,uint32_t baud,int rx,int tx)
{
   scan_mutex = xSemaphoreCreateMutex();
	serial=_mySerial;
  serial->setRxBufferSize(256);
	serial->begin(baud, SERIAL_8N1,rx,tx);
}



stDeviceInfo_t rpLidar::getDeviceInfo()
{
	clearSerialBuffer();
	stDeviceInfo_t info;
	rp_descriptor_t descr;
	serial->write((uint8_t*)&req_message[rq_info],2); //send Device Info request
	if(!checkForTimeout(10,27))	//wait for Response
	{
		serial->readBytes((uint8_t*)&descr,7);
		serial->readBytes((uint8_t*)&info,20);
	}
	return info;
}

stDeviceStatus_t rpLidar::getDeviceHealth()
{
	clearSerialBuffer(); //remove old data in SerialBuffer
	rp_descriptor_t descr;
	stDeviceStatus_t deviceStatus;
	serial->write((uint8_t*)&req_message[rq_health],2); //send device health request
	if(!checkForTimeout(400,10)) //wait for response
	{
		serial->readBytes((uint8_t*)&descr,7);
		serial->readBytes((uint8_t*)&deviceStatus,3);
	}
	return deviceStatus;
}

void rpLidar::resetDevice()
{
	serial->write((uint8_t*)&req_message[rq_reset],2); //send reset request
	delay(800); //wait for reboot
	clearSerialBuffer(); //remove old data in SerialBuffer
	status=false;
}

void rpLidar::stopDevice()
{
	serial->write((uint8_t*)&req_message[rq_stop],2);
}

bool rpLidar::start(uint8_t _mode)
{
	resetDevice();
	clearSerialBuffer();
  Serial.write("strt");
  serial->write((uint8_t*)&req_Express[denseVersion],9); //express scan request

	rp_descriptor_t descr;

	if(!checkForTimeout(100,7)) //wait for response
	{
		serial->readBytes((uint8_t*)&descr,7);
    scanMode=_mode;
    status=true;
    Serial.print("1");
    //ToDo - figure out correct response
    return compareDescriptor(descr,resp_descriptor[ultradense]);
	}
  return false;
}

void rpLidar::setAngleOfInterest(uint16_t _left,uint16_t _right)
{
	//setter
	interestAngleLeft=_left;
	interestAngleRight=_right;
}


bool rpLidar::isDataBetweenBorders(stScanDataPoint_t _point)
{
	float angle=calcAngle(_point.angle_low,_point.angle_high);
	if((angle>=interestAngleLeft)&&(angle<=interestAngleRight))
	{
		return true;
	}
	return false;
}

bool rpLidar::isDataBetweenBorders(float _angle)
{
	if((_angle>interestAngleLeft)&&(_angle<interestAngleRight))
	{
		return true;
	}
	return false;
}


bool rpLidar::isDataValid(stScanDataPoint_t _point)
{
	if(calcDistance(_point.distance_low,_point.distance_high)>0)
	{
		return true;
	}
	return false;
}

bool rpLidar::isDataValid(uint16_t _distance)
{
	if(_distance>0)
	{
		return true;
	}
	return false;
}

bool rpLidar::isRunning()
{
	return status;
}

uint8_t rpLidar::isScanMode()
{
	return scanMode;
}

float rpLidar::calcAngle(uint8_t _lowByte,uint8_t _highByte)
{
	uint16_t winkel=_highByte<<7;
	winkel|=_lowByte>>1;
	return winkel/64.0;
}

float rpLidar::calcCapsuledAngle(uint16_t _Wi,uint16_t _Wi2,uint8_t _k)
{
	float angle1=_Wi/64.00;
	float angle2=_Wi2/64.00;
	float result;
	if(angle1<=angle2)
	{
		result=angle1+((angle2-angle1)/40)*_k;
	}
	else
	{
		result=angle1+((360+angle2-angle1)/40)*_k;
	}
	if(result>360.0)
	{
		result=result-360.0;
	}
	return result;
}




float rpLidar::calcDistance(uint8_t _lowByte,uint8_t _highByte)
{
	uint16_t distance=(_highByte)<<8;
	distance|=_lowByte;
	return distance/4.0;
}



//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//
//											 Debug Functions
//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//

void rpLidar::DebugPrintMeasurePoints(int16_t count)
{

  for (int pos = 0; pos < (int)count; ++pos) {
      scanDot dot;
      if (!_cached_scan_node_hq_buf[pos].dist_mm_q2) continue;
      //dot.quality = _cached_scan_node_hq_buf[pos].quality; //quality is broken for some reason
      dot.angle = (((float)_cached_scan_node_hq_buf[pos].angle_z_q14) * 90.0 / 16384.0);
      dot.dist = _cached_scan_node_hq_buf[pos].dist_mm_q2 /4.0f;
      Serial.print(dot.angle);
      Serial.print(":");
      Serial.println(dot.dist);
      //Serial.print(":");
      //Serial.print(dot.quality);
  }
}

void rpLidar::DebugPrintDeviceErrorStatus(stDeviceStatus_t _status)
{
	Serial.println("\n--Device Health--");
	Serial.print("Status:");
	Serial.println(_status.status);
	Serial.print("Error Low:");
	Serial.print(_status.errorCode_low);
	Serial.print("Error High:");
	Serial.println(_status.errorCode_high);
	Serial.println('\n');
}

void rpLidar::DebugPrintDeviceInfo(stDeviceInfo_t _info)
{
	Serial.println("\n--Device Info--");
	Serial.print("Model:");
	Serial.println(_info.model);
	Serial.print("Firmware:");
	Serial.print(_info.firmware_major);
	Serial.print(".");
	Serial.println(_info.firmware_minor);
	Serial.print("Hardware:");
	Serial.println(_info.hardware);
	Serial.print("Serial Number:");
	for(uint16_t i=0;i<16;i++)
	{
		Serial.print(_info.serialnumber[i],HEX);
	}
	Serial.println('\n');

}

void rpLidar::DebugPrintDescriptor(rp_descriptor_t _descriptor)
{
	Serial.print ("Descriptor : ");
	for(uint8_t i=0;i<7;i++)
	{
		Serial.print(_descriptor[i],HEX);
		Serial.print("|");
	}
	Serial.println();
}




//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//
//											End Debug Functions
//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//




bool rpLidar::compareDescriptor(rp_descriptor_t _descr1,rp_descriptor_t _descr2)
{
	for(size_t i=0;i<sizeof(rp_descriptor_t);i++)
	{
		if(_descr1[i]!=_descr2[i])
		{
			return false;
		}
	}
	return true;
}

void rpLidar::clearSerialBuffer()
{
	while(serial->available())//read as long the hardware buffer is not empty
	{
		serial->read();
	}
}

bool rpLidar::checkCRC(stExpressDataPacket_t _package,uint8_t _crc)
{
	uint8_t crc=0;
	crc=(uint8_t)_package.angle&0x00FF;
	crc^=(uint8_t)(_package.angle>>8);
	for(int i=0;i<40;i++)
	{
		crc^=(uint8_t)_package.cabin[i];
		crc^=(uint8_t)(_package.cabin[i]>>8);

	}
	if(_crc==crc)
	{
		return true;
	}
	return false;
}

bool rpLidar::checkForTimeout(uint32_t _time,size_t _size)
{
	float startTime=millis();
	while(!(serial->available()>=_size))
	{
		if(millis()>(startTime+_time)){
			Serial.println("Lidar Timeout");
			return true;
		}
	}
	return false;
}



double  rpLidar::calcAngle(stExpressDataPacket_t* _packets,uint16_t _k)
{
	double  angle1=(_packets->angle&0x7FFF)/64.00;
	_packets++;
	double  angle2=(_packets->angle&0x7FFF)/64.00;
	double  result;
	if(angle1<=angle2)
	{
		result=angle1+((angle2-angle1)/40)*_k;
	}
	else
	{
		result=angle1+((360+angle2-angle1)/40)*_k;
	}
	if(result>360.0)
	{
		result=result-360.0;
	}
	return result;

}
sl_result rpLidar::cacheUltraCapsuledScanData()
{
    sl_lidar_response_ultra_capsule_measurement_nodes_t    ultra_capsule_node;
    sl_lidar_response_measurement_node_hq_t   local_buf[256];
    size_t   count = 256;
    sl_lidar_response_measurement_node_hq_t   local_scan[MAX_SCAN_NODES];
    size_t     scan_count = 0;
    sl_result ans = SL_RESULT_OK;
    memset(local_scan, 0, sizeof(local_scan));

    _waitUltraCapsuledNode(ultra_capsule_node);
    while (_isScanning) {
        ans = _waitUltraCapsuledNode(ultra_capsule_node);
        if (ans !=0) {
            if ((sl_result)ans != SL_RESULT_OPERATION_TIMEOUT && (sl_result)ans != SL_RESULT_INVALID_DATA) {
                //_isScanning = false;
                return (sl_result)ans ;
            }
            else {
                // current data is invalid, do not use it.
                continue;
            }
        }

        _ultraCapsuleToNormal(ultra_capsule_node, local_buf, count);
        for (size_t pos = 0; pos < count; ++pos) {
            if (local_buf[pos].flag & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT) {
                // only publish the data when it contains a full 360 degree scan 

                if ((local_scan[0].flag & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT)) {   
                  //xSemaphoreTake(scan_mutex, 500);  
                    memcpy(_cached_scan_node_hq_buf, local_scan, scan_count * sizeof(sl_lidar_response_measurement_node_hq_t));
                    _cached_scan_node_hq_count = scan_count;
                  //xSemaphoreGive(scan_mutex);
                }
                scan_count = 0;
            }
            local_scan[scan_count++] = local_buf[pos];
            if (scan_count == MAX_SCAN_NODES) scan_count -= 1; // prevent overflow

            //for interval retrieve
           // {
           //     rp::hal::AutoLocker l(_lock);
           //     _cached_scan_node_hq_buf_for_interval_retrieve[_cached_scan_node_hq_count_for_interval_retrieve++] = local_buf[pos];
           //     if (_cached_scan_node_hq_count_for_interval_retrieve == _countof(_cached_scan_node_hq_buf_for_interval_retrieve)) _cached_scan_node_hq_count_for_interval_retrieve -= 1; // prevent overflow
            //}
        }
    }

    _isScanning = false;

    return SL_RESULT_OK;
}

sl_result rpLidar::_waitUltraCapsuledNode(sl_lidar_response_ultra_capsule_measurement_nodes_t & node, sl_u32 timeout)
{

    int  recvPos = 0;
    sl_u32 startTs = millis();
    sl_u8  recvBuffer[sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t)];
    sl_u8 *nodeBuffer = (sl_u8*)&node;
    sl_u32 waitTime;
    while ((waitTime = millis() - startTs) <= DEFAULT_TIMEOUT) {
        size_t remainSize = sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t) - recvPos;
        size_t recvSize;

        recvSize = serial->available();
        if(recvSize ==0) continue;
        if (recvSize > remainSize) recvSize = remainSize;
        recvSize = serial->read(recvBuffer, recvSize);
        for (size_t pos = 0; pos < recvSize; ++pos) {
            sl_u8 currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0: // expect the sync bit 1
            {
                sl_u8 tmp = (currentByte >> 4);
                if (tmp == SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_1) {
                    // pass
                }
                else {
                    _is_previous_capsuledataRdy = false;
                    continue;
                }
            }
            break;
            case 1: // expect the sync bit 2
            {
                sl_u8 tmp = (currentByte >> 4);
                if (tmp == SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                    // pass
                }
                else {
                    recvPos = 0;
                    _is_previous_capsuledataRdy = false;
                    continue;
                }
            }
            break;
            }
            nodeBuffer[recvPos++] = currentByte;
            if (recvPos == sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t)) {
                // calc the checksum ...
                sl_u8 checksum = 0;
                sl_u8 recvChecksum = ((node.s_checksum_1 & 0xF) | (node.s_checksum_2 << 4));

                for (size_t cpos = offsetof(sl_lidar_response_ultra_capsule_measurement_nodes_t, start_angle_sync_q6);
                    cpos < sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t); ++cpos) 
                {
                    checksum ^= nodeBuffer[cpos];
                }

                if (recvChecksum == checksum) {
                    // only consider vaild if the checksum matches...
                    if (node.start_angle_sync_q6 & SL_LIDAR_RESP_MEASUREMENT_EXP_SYNCBIT) {
                        // this is the first capsule frame in logic, discard the previous cached data...
                        _is_previous_capsuledataRdy = false;

                        return SL_RESULT_OK;
                    }
                    return SL_RESULT_OK;
                }
                _is_previous_capsuledataRdy = false;
                return SL_RESULT_INVALID_DATA;
            }
        }
    }
    _is_previous_capsuledataRdy = false;
    return SL_RESULT_OPERATION_TIMEOUT;
}

void rpLidar::_ultraCapsuleToNormal(const sl_lidar_response_ultra_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
{
    nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((capsule.start_angle_sync_q6 & 0x7FFF) << 2);
        int prevStartAngle_q8 = ((_cached_previous_ultracapsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 > currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        int angleInc_q16 = (diffAngle_q8 << 3) / 3;
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < 32/*_countof(_cached_previous_ultracapsuledata.ultra_cabins)*/; ++pos) {
            int dist_q2[3];
            int angle_q6[3];
            int syncBit[3];


            sl_u32 combined_x3 = _cached_previous_ultracapsuledata.ultra_cabins[pos].combined_x3;

            // unpack ...
            int dist_major = (combined_x3 & 0xFFF);

            // signed partical integer, using the magic shift here
            // DO NOT TOUCH

            int dist_predict1 = (((int)(combined_x3 << 10)) >> 22);
            int dist_predict2 = (((int)combined_x3) >> 22);

            int dist_major2;

            sl_u32 scalelvl1, scalelvl2;

            // prefetch next ...
            if (pos == 32/*_countof(_cached_previous_ultracapsuledata.ultra_cabins)*/ - 1) {
                dist_major2 = (capsule.ultra_cabins[0].combined_x3 & 0xFFF);
            }
            else {
                dist_major2 = (_cached_previous_ultracapsuledata.ultra_cabins[pos + 1].combined_x3 & 0xFFF);
            }

            // decode with the var bit scale ...
            dist_major = _varbitscale_decode(dist_major, scalelvl1);
            dist_major2 = _varbitscale_decode(dist_major2, scalelvl2);


            int dist_base1 = dist_major;
            int dist_base2 = dist_major2;

            if ((!dist_major) && dist_major2) {
                dist_base1 = dist_major2;
                scalelvl1 = scalelvl2;
            }


            dist_q2[0] = (dist_major << 2);
            if ((dist_predict1 == 0xFFFFFE00) || (dist_predict1 == 0x1FF)) {
                dist_q2[1] = 0;
            }
            else {
                dist_predict1 = (dist_predict1 << scalelvl1);
                dist_q2[1] = (dist_predict1 + dist_base1) << 2;

            }

            if ((dist_predict2 == 0xFFFFFE00) || (dist_predict2 == 0x1FF)) {
                dist_q2[2] = 0;
            }
            else {
                dist_predict2 = (dist_predict2 << scalelvl2);
                dist_q2[2] = (dist_predict2 + dist_base2) << 2;
            }


            for (int cpos = 0; cpos < 3; ++cpos) {
                syncBit[cpos] = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) ? 1 : 0;

                int offsetAngleMean_q16 = (int)(7.5 * 3.1415926535 * (1 << 16) / 180.0);

                if (dist_q2[cpos] >= (50 * 4))
                {
                    const int k1 = 98361;
                    const int k2 = int(k1 / dist_q2[cpos]);

                    offsetAngleMean_q16 = (int)(8 * 3.1415926535 * (1 << 16) / 180) - (k2 << 6) - (k2 * k2 * k2) / 98304;
                }

                angle_q6[cpos] = ((currentAngle_raw_q16 - int(offsetAngleMean_q16 * 180 / 3.14159265)) >> 10);
                currentAngle_raw_q16 += angleInc_q16;

                if (angle_q6[cpos] < 0) angle_q6[cpos] += (360 << 6);
                if (angle_q6[cpos] >= (360 << 6)) angle_q6[cpos] -= (360 << 6);

                sl_lidar_response_measurement_node_hq_t node;

                node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                node.quality = dist_q2[cpos] ? (0x2F << SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                node.angle_z_q14 = sl_u16((angle_q6[cpos] << 8) / 90);
                node.dist_mm_q2 = dist_q2[cpos];

                nodebuffer[nodeCount++] = node;
            }

        }
    }

    _cached_previous_ultracapsuledata = capsule;
    _is_previous_capsuledataRdy = true;
}
