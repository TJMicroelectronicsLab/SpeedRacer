#include "rpLidar.h"
#include "rpLidarTypes.h"
#include "Servo.h"
#include <esp_task_wdt.h>
#define MIN_ANGLE 290
#define MAX_ANGLE 70
#define INCLUDED_ANGLE 140
rpLidar lidar(&Serial2,115200,19,18);
int start_time;
Servo steering,speed;
int lastScanNum;
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
  esp_task_wdt_init(36000, false); //turn off watchdog so core 0 task doesn't cause reset
  //attach speed servo control to port
  speed.attach(15);
  speed.writeMicroseconds(1500);
  //attach steering servo control
  steering.attach(27);
  //set pwm pin high (or just attach pin to 5v)
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);
  //log output
  Serial.begin(115200);
  //pause that refreshes
  Serial.print("starting");
  delay(1000);

//start up lidar
  lidar.resetDevice(); //reset the device to be sure that the status is good
  lidar.getDeviceHealth();
  lidar.setAngleOfInterest(0,360); //Set the field of view that is saved to Data
  lidar.start(standard); //start the standard scan of the lidar (if you want more data point, it scans up to 8k, but that is in a different mode, see code and manual)
//create thread on second core to read lidar
  xTaskCreatePinnedToCore(readPoints, "Task_TFT", 4096, NULL, 2, NULL, 0);
    //speed.writeMicroseconds(1550);
    lastScanNum = 0;
}
struct Hole{
    int startAngle;
    int endAngle;
} ;
#define DISTANCE_ENVELOPE 1000
#define LIDAR_WINDOW 70 // +/- this value from zero is the window
void loop()
{
  Hole largestHole = {.startAngle=0,.endAngle=0};
  Hole currentHole;
  bool inHole = false;
//lidar.scanPoints[] contains 360 values of distances. Zero values should be ignored, they are invalid.
//scanCount is incremented every time a complete scan is finished.
//check to make sure new scan is available
//Serial.print("SC");
//printPoints(0,0);
Serial.println(millis());
if(lidar.scanCount > lastScanNum ){
  Serial.println(lidar.scanCount);
}
if(lastScanNum < lidar.scanCount){
  lastScanNum = lidar.scanCount;
  for(int i = (360-DISTANCE_ENVELOPE);i<(360+DISTANCE_ENVELOPE);i++){
      int angle = i % 360;
      //start hole
      if( lidar.scanPoints[angle] > DISTANCE_ENVELOPE){
          if(inHole == false){
              currentHole.startAngle = i;
              inHole = true;
          }
      }//end hole
      else{
        currentHole.endAngle = i;
        if((currentHole.endAngle - currentHole.startAngle )> (largestHole.endAngle - largestHole.startAngle)){
          largestHole= currentHole;
          inHole = false;
        }
      }
  }
  //take largest hole and steer towards center, centerangle is from 0- 2x DISTANCE_ENVELOPE
  int centerAngle = (largestHole.endAngle) - (largestHole.startAngle);
  int steeringAngle = centerAngle/(2 * DISTANCE_ENVELOPE) * 1000 + 1000;
  Serial.println(steeringAngle);
  steering.writeMicroseconds(steeringAngle);
}
//no thread synchronization is necessary if you don't write to these values
//control that car
}