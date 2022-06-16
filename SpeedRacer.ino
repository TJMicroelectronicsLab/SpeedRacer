/***

LiDar racer

Works with slamtec lidar, tested with A1M8. Strategy is using a sliding window to look for the highest maximum avg distance in any 15 degree window. This window is chosen
based on the minimum distance to include in a window

*/
using namespace std;
#include "rpLidar.h"
#include "rpLidarTypes.h"
#include "Servo.h"
#include <esp_task_wdt.h>
#include <vector>
#define MIN_ANGLE 290
#define CLOSE_DIST 400 //how much room we require as buffer around car
#define MAX_ANGLE 70 //1/2 angle of lidar window

//lidar instantiation, sep thread to check lidar
rpLidar lidar(&Serial2,115200,13,12);
//for calcing time elapsed
int start_time;
//servos to control steering and speed
Servo steering,speed;

//function for thread
static void readPoints(void * parameter){
  while(true){
    int result = lidar.cacheUltraCapsuledScanData();
  }
}
//used to store candidate open gaps
class run{
  public:
    float startAngle;
    float endAngle;
    int distance_count;
    int distance_sum;
    bool big_angle;
    static bool dist_compare(run  &L, run  &R) {
    return (L.endAngle < R.endAngle);
}
} ;

void setup() {
  pinMode(19,OUTPUT);
  digitalWrite(19,HIGH);
  esp_task_wdt_init(36000, false); //turn off watchdog so core 0 task doesn't cause reset
  speed.attach(32);
  speed.writeMicroseconds(1500);
  steering.attach(27);
  steering.writeMicroseconds(1500);
  Serial.begin(921600);
  lidar.stopDevice(); //reset the device to be sure that the status is good
  delay(100);
  if(!lidar.start(express)){
    Serial.println("failed to start");
    return;
  } 
  //this constantly polls the serial stream for lidar data. 65536 is the stack size, necessary for buffers on stack
  xTaskCreatePinnedToCore(readPoints, "Task_TFT", 65536, NULL, 2, NULL, 0);
  start_time = millis();
  //speed.writeMicroseconds(1515);
}

int maxDist=700, longcount = 0;
int maxDistance = 0;// current maximum distance
int startIndex=0,stopIndex=0;
int maxLength=0;
int lastAngle=-1;
//this contains the next potential open angle
run nextRun; //= {.count=0};
bool inRun = false;
//tracks complete turns of lidar so we know when to look
uint8_t last_complete_scan_count=0;
//last millis a complete scan was finished
uint last_scan_time=0;
int steeringangle;
//zero angle distance
//changes speed based on zero angle distance and gap distance
int speedModifier=0;

#define SPEED  1569
#define MIN_DIST 3000
void loop()
{
  std::vector<run> runs;

  nextRun.big_angle = false;
  nextRun.distance_count=0;
  nextRun.distance_sum=0;
    //maximum range we will look at

  int lastDistance=20000;
  //distance zt zero degrees, as a sanity check on running room
  int zero_distance=-1;

  //stop wheels after a while to avoid free running around school
  if(millis()-start_time > 60000) {
    speed.writeMicroseconds(1500);
    while(true);
  }
  float angle; 

  //#of measurements for a gap considered wide enough to go thru
  int gap_width =0;
  //start wheels when we have process first scan
  if(lidar.total_scan_count==2)   speed.writeMicroseconds(SPEED);

  //if there hasn't been a new scan then  (checks for scan count)
  if(lidar.total_scan_count <= last_complete_scan_count){
    return;
  }else{
    last_complete_scan_count = lidar.total_scan_count;
    //Serial.print("TIME:");
    //Serial.println(millis());
	//how large the gap must be in samples (since this is variable based on speed of revolution), based on 12 degree gap
    gap_width = lidar._cached_scan_node_hq_count*12/360;
  }
  int distance=0;
  //loop thru complete scan and find largest gap 
  //angle ~270-359
  uint distance_sum=0;
  //count of distances in average
  uint distance_count=0;
  //the current maximum average distance, for an ~15 degree window
  float cur_max_avg=0;
  int max_start_count=0,max_end_count=0;
  //current distance which is considered a gap
  int current_min_distance = MIN_DIST;
  int loop_count=0;
  int close_right = 99999, close_left = 99999;
  int close_30 = 9999,close_120 = 9999;
do{
	//decrement the maximum scan distance (this is poor placement, obviously)
  current_min_distance -= 400;
  //if gap is too small wait for next scan (this is probably trouble)
  if(current_min_distance < 400){
    return;
  }
  //search first half (assume contains -75 to 0 degrees)
  for(int i=lidar._cached_scan_node_hq_count/2; i<lidar._cached_scan_node_hq_count;i++)
  {
    angle = (((float)lidar._cached_scan_node_hq_buf[i].angle_z_q14) * 90.0 / 16384.0);
	//not interested in these angles
    if(angle < 285 && angle > 75) continue;
    int distance = lidar._cached_scan_node_hq_buf[i].dist_mm_q2 /4.0f;
	//skip bad reads
    if(distance == 0) continue;
	//scale everything to 0 to 150 to make it easier on the calculations
    angle+=75;
    
    //convert left angles to 0-74, right angles 75-150
    if(angle >=360)angle-=360;
	//these are rough checks for close walls
    if(angle >0 && angle < 1 && distance < close_left ) close_left = distance; 
    if(angle >15 && angle < 16 && distance < close_left ) close_left = distance; 
    if(angle >50 && angle < 51 && distance < close_30) close_30= distance;
	//if the read is larger than gap distance, assume we are in a gap
    if(distance > current_min_distance){
        
      //count sequences that are farther out than our window
      if(!inRun) {
        nextRun.startAngle = angle;
       // Serial.print("start angle:");
       // Serial.println(angle);
        inRun=true;
      }
      nextRun.distance_sum+=distance;
      nextRun.distance_count++;
    }else{
      if(inRun){
        nextRun.endAngle = angle - nextRun.startAngle;//technically last angle, oh well
        if(nextRun.endAngle > 17 ||  current_min_distance * sin(nextRun.endAngle) > 275)
            runs.push_back(nextRun);
      }
      inRun = false;
      }
  }
  //do the same for the next set of angles (assume catch 0-75 in here somewhere)
  for(int i=0; i<lidar._cached_scan_node_hq_count/2;i++)
  {
    angle = (((float)lidar._cached_scan_node_hq_buf[i].angle_z_q14) * 90.0 / 16384.0);
    if(angle < 285 && angle > 75) continue;
    int distance = lidar._cached_scan_node_hq_buf[i].dist_mm_q2 /4.0f;
    if(distance == 0) continue;
    angle+=75;
    //take first valid entry around zero degrees and use as front distance, for speed calc
    if(angle >74.5 && zero_distance==-1)zero_distance = distance;

    //convert left angles to 0-74, right angles 75-150
    if(angle >=360)angle-=360;
    //check left and right for close walls 
    if(angle >149 && distance < distance < close_right)close_right = distance;
    if(angle >135 && angle < 136 && distance < close_right ) close_right = distance; 
    if(angle >100 && angle < 101 && distance < close_120) close_120= distance;

    //if current range is greater than our min window count as a gap
    if(distance > current_min_distance){
        
      //count sequences that are farther out than our window
      if(!inRun) {
        nextRun.startAngle = angle;
       // Serial.println(angle);
        inRun=true;
      }
      nextRun.distance_sum+=distance;
      nextRun.distance_count++;
    }else{
      if(inRun){
        nextRun.endAngle = angle - nextRun.startAngle;//technically last angle, oh well
        //make sure this gap is big enough to fit the car
        if(nextRun.endAngle > 17 ||  current_min_distance * sin(nextRun.endAngle) > 275)
            runs.push_back(nextRun);
      }
      inRun = false;
     }
  }
  if(inRun){
        nextRun.endAngle = angle - nextRun.startAngle;//technically last angle, oh well
        if(nextRun.endAngle > 17 ||  current_min_distance * sin(nextRun.endAngle) > 275)
            runs.push_back(nextRun);
  }
inRun = false;
}while(runs.size() == 0);

run best_run;
float big_angle;
int big_gap, big_run=-1;
//take largest gap, which will also be longest gap, for simple mazes we will only have one gap most of the time
for(int i=0;i<runs.size();i++){
  if(runs[i].endAngle > big_run){
    best_run = runs[i];
    big_run = runs[i].endAngle;
  }
}
//what is the avg gap distance, used to use for speed (at least I used to, this is dead at the moment)
cur_max_avg = best_run.distance_sum/best_run.distance_count;
//Serial.print("Dist:");
//Serial.println(cur_max_avg);
   //if the edge of our gap is a wall then steer away from it
 int close_distance;
 float close_angle, left_angle=0,right_angle=0;
 bool found_left=false,found_right=false,found_30 = false,found_120 = false;

if(close_left < CLOSE_DIST) found_left= true;
if(close_right < CLOSE_DIST) found_right= true;
if(close_30 < CLOSE_DIST) found_30= true;
if(close_120 < CLOSE_DIST) found_120= true;

//  if(found_right || found_left){
// Serial.println("CR");
// Serial.println(found_left);
// Serial.println(found_right);
//   }
//aim for center of gap
  angle = (float)(best_run.startAngle + best_run.endAngle/2); //aim for center of gap, normalize -.5 to .5
//check for close left and close right walls, steer away as appropriate
  if(found_right && found_left){
    angle = 75;//subtract right angle, add left angle, steers away from obstance which is closest to 90 degrees
  }else if (found_right) {
    if(angle > 65) angle = 65;
  }else if (found_left){ 
    if(angle <85) angle = 85;
  }
  if(found_30 && found_120){
    angle=75;
  }else if(found_30){
    if(angle <105) angle = 105;
  }else if(found_120){
    if(angle >45) angle=45;
  }
  //normalize to +/- .5 degrees, to make math easy
  angle = angle/(2*75)-.5;
  //take the largest opening and turn the wheel towards it
  //The center angle of the opening is -180 is the degrees off center
  //1950 us is 30 degrees right turn
  //1050 us is 30 deg left turn
  //this could be improved. More or less scaling, also with narrow width lanes it is not stable and the car oscillates until it is out of control (see PID controller...)
  int steering_micro = angle*1200+1500;
  if(steeringangle != steering_micro){
    Serial.print("SA: ");
    Serial.println(steering_micro);
    steeringangle = steering_micro;
    steering.writeMicroseconds(steeringangle);    

  }
//we need at least four hundred to adjust speed
  if(zero_distance > 400){
    int next_speedModifier = (zero_distance - 400)/100;//2000 (max distance for max speed)*500, max speed modifier
    if(next_speedModifier < 0) next_speedModifier=0;
    if(next_speedModifier > speedModifier +30) speedModifier+=30;
    else speedModifier = next_speedModifier;
    if(speedModifier > 60) speedModifier=60;
    //close check
   // if(zero_distance < 600) speedModifier = 0;
   // Serial.print("SM: ");
    //Serial.println(speedModifier);
    int newspeed = SPEED + speedModifier;
    speed.writeMicroseconds(newspeed);

  }else{
	  //stop if we are right on something (or add backup if hits are OK)
    if(zero_distance < 150){
		Serial.println("ZS");
		steering.writeMicroseconds(2000);
		speed.writeMicroseconds(1420);
		delay(10);
		speed.writeMicroseconds(1500);
		steering.writeMicroseconds(1500);
    }
    //while(true);
  }  
  Serial.println(millis());
}