#include "rpLidar.h"
#include "rpLidarTypes.h"
#include <esp_task_wdt.h>


rpLidar lidar(&Serial2,115200,13,12);

static void readPoints(void * parameter){
  while(true){
    int result = lidar.cacheUltraCapsuledScanData();
    Serial.println(result,HEX);
  }
}
void setup() {
   pinMode(19,OUTPUT);
  digitalWrite(19,HIGH);
  Serial.begin(115200);
  esp_task_wdt_init(36000, false); //turn off watchdog so core 0 task doesn't cause reset
  lidar.stopDevice(); //reset the device to be sure that the status is good
  delay(1);
  if(!lidar.start(express)){
    Serial.println("failed to start");
    return;
  } //start the express scan of the lidar\  esp_task_wdt_init(36000, false); //turn off watchdog so core 0 task doesn't cause reset

  xTaskCreatePinnedToCore(readPoints, "LidarPolling", 65536, NULL, 2, NULL, 0);

}

void loop()
{
// number of data points in cache: lidar._cached_scan_node_hq_count
// float angle = (((float)_cached_scan_node_hq_buf[index].angle_z_q14) * 90.0 / 16384.0);
// float distance = _cached_scan_node_hq_buf[index].dist_mm_q2 /4.0f;
// each cache load contains a full 360 scan. If you slow down the rotations too much it will not fit and data will be lost (too many points per 360 deg for cache size allowable on ESP32)
 lidar.DebugPrintMeasurePoints(lidar._cached_scan_node_hq_count);
 delay(1);
}