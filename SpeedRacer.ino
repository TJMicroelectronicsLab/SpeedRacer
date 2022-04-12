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
    lidar.readMeasurePoints();
  }
}
void printPoints(int start,int stop){
  for (int i=0;i<70;i++){
    Serial.print("A");
    Serial.print(  i   );
    Serial.print(":");
    Serial.print( lidar.scanPoints[i]);
    Serial.print(":");
    Serial.print(lidar.quality[i]);
    Serial.println();
  }
    for (int i=290;i<360;i++){
    Serial.print("A");
    Serial.print(  i   );
    Serial.print(":");
    Serial.print( lidar.scanPoints[i]);
    Serial.print(":");
    Serial.print(lidar.quality[i]);
    Serial.println();
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
struct run{
    int startAngle;
    int endAngle;
    int count;
} ;
void loop()
{
//lidar.scanPoints[] contains 360 values of distances. 
//scanCount is incremented every time a complete scan if finished.
//no thread synchronization is necessary if you don't write to these values
//control that car
}