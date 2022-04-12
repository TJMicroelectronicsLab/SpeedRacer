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
  speed.attach(15);
  speed.writeMicroseconds(1500);
  steering.attach(19);
  pinMode(14,OUTPUT);
  digitalWrite(14,HIGH);
  Serial.begin(460800);
  delay(1000);
  start_time = millis();
  lidar.resetDevice(); //reset the device to be sure that the status is good
  lidar.setAngleOfInterest(0,360); //Set the field of view that is saved to Data
  lidar.start(standard); //start the express scan of the lidar

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
  //maximum range we will look at
  int maxDist=700, longcount = 0;
  int maxDistance = 0;// current maximum distance
  int startIndex=0,stopIndex=0;
  int maxLength=0;
  int lastAngle=-1;
  run maxRun = {.startAngle=0,.endAngle=0,.count=0};
  run nextRun = {.count=0};
  int angle = 0;
  int outoforder = 0;
  int variance =0; //variance in current run. if variance gets too large then start new run
  if(millis()-start_time > 5000) {
    speed.writeMicroseconds(1500);
  }
//read whatever is available validate it
   // reads a full scan and save it to Data
  //Serial.print(count);
  /*printPoints(0,8);
    return;*/
 // int starttime = esp_timer_get_time();
  for(int i=-70; i<70;i++)
  {
    angle =(270+angle)%360;
    if(lidar.scanPoints[angle] > 0){

    //count sequences that are farther out than our window
      if(lidar.scanPoints[angle] > maxDist){
          if(maxLength==0) {
            maxRun.startAngle = angle;
            startIndex = angle;
          }
          maxLength++;
      }else{
        //if we had a new long run save it and its angle
          if(maxLength > maxRun.count){
              maxRun.endAngle = angle;
              maxRun.count = maxLength;
              stopIndex = angle;       
          }
          startIndex = 0;
          stopIndex = 0;
          maxLength=0;
      }
    }

  }
  if(maxRun.count > 0 and maxRun.endAngle==0) maxRun.endAngle = angle;

  //take the largest opening and turn the wheel towards it
  //The center angle of the opening is -180 is the degrees off center
  //1950 us is 30 degrees right turn
  //1050 us is 30 deg left turn
  if(maxRun.count > 0){
    if(maxRun.startAngle > MIN_ANGLE){
      maxRun.startAngle -=MIN_ANGLE;
    }else{
      maxRun.startAngle +=MAX_ANGLE;
    }
    if(maxRun.endAngle > MIN_ANGLE){
      maxRun.endAngle -=MIN_ANGLE;
    }else{
      maxRun.endAngle +=MAX_ANGLE;
    }
   // Serial.print("M");
  //  Serial.print(maxRun.startAngle);
   // Serial.println(maxRun.endAngle);
    int angle = (maxRun.startAngle + maxRun.endAngle )/2; //aim for center of gap

    int steeringangle = (angle*1000)/ INCLUDED_ANGLE + 1000;
      steering.writeMicroseconds(steeringangle);
  //  Serial.print("SA");
 //   Serial.print(steeringangle);
    // Serial.println("MaxRun");
     // Serial.println(maxRun.startAngle);
     //Serial.println(maxRun.endAngle);
    //  Serial.println(maxRun.count);
  }else{
    speed.write(1500);
  }
  //starttime = esp_timer_get_time()- starttime;
  //Serial.print(starttime);
  //Serial.print("\t");
    //we have at least one run


}