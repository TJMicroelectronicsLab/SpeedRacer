#include "rpLidar.h"
#include "rpLidarTypes.h"
#include "Servo.h"
#include <esp_task_wdt.h>
#define MIN_ANGLE 290
#define MAX_ANGLE 70
#define INCLUDED_ANGLE 140
rpLidar lidar(&Serial2,115200,13,12);
int start_time;
Servo steering,speed;

static void readPoints(void * parameter){
  while(true){
    int result = lidar.cacheUltraCapsuledScanData();
	if(result!=0)
		Serial.println(result,HEX);
  }
}

void setup() {
  esp_task_wdt_init(30, false); //turn off watchdog so core 0 task doesn't cause reset
  //attach speed servo control to port
  speed.attach(15);
  speed.writeMicroseconds(1500);
  //attach steering servo control
  steering.attach(19);
  //set pwm pin high (or just attach pin to 5v)
  pinMode(14,OUTPUT);
  digitalWrite(14,HIGH);
  //log output
  Serial.begin(460800);
  //pause that refreshes
  delay(1000);

//start up lidar
  lidar.resetDevice(); //reset the device to be sure that the status is good
  lidar.setAngleOfInterest(0,360); //Set the field of view that is saved to Data
  lidar.start(standard); //start the standard scan of the lidar (if you want more data point, it scans up to 8k, but that is in a different mode, see code and manual)
//create thread on second core to read lidar
  xTaskCreatePinnedToCore(readPoints, "Task_TFT", 4096, NULL, 2, NULL, 0);
    //speed.writeMicroseconds(1550);
}
#include "rpLidar.h"
#include "rpLidarTypes.h"
#include <esp_task_wdt.h>


rpLidar lidar(&Serial2,115200);


void setup() {
 
  Serial.begin(921600);
  esp_task_wdt_init(36000, false); //turn off watchdog so core 0 task doesn't cause reset
  lidar.stopDevice(); //reset the device to be sure that the status is good
  lidar.setAngleOfInterest(5,175); //Set the field of view that is saved to Data
  if(!lidar.start(express)){
    Serial.println("failed to start");
    return;
  } //start the express scan of the lidar\  esp_task_wdt_init(36000, false); //turn off watchdog so core 0 task doesn't cause reset

  xTaskCreatePinnedToCore(readPoints, "Task_TFT", 65536, NULL, 2, NULL, 0);

}

void loop()
{
  //xSemaphoreGive(mutex);
 // xSemaphoreTake(mutex, portMAX_DELAY);
  //lidar.readMeasurePoints();// reads a full scan and save it to Data
 lidar.DebugPrintMeasurePoints(lidar._cached_scan_node_hq_count);
 delay(1);
}