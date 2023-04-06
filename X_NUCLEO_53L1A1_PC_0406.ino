#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <vl53l1x_x_nucleo_53l1a1_class.h>

#define DEV_I2C Wire
#define SerialPort Serial

// People Counting defines
#define NOBODY                  0
#define SOMEONE                 1
#define LEFT                    0
#define RIGHT                   1

#define DOOR_JAM_2400           1     //#define DOOR_JAM_2000 2  #define ON_SIDE  3
#define DISTANCE_MODE_LONG      1     //#define DISTANCE_MODE_LONG        2
#define TRACE_PPC               0

#define DISTANCES_ARRAY_SIZE    10   // nb of samples
#define MAX_DISTANCE            2400 // mm
#define MIN_DISTANCE            0   // mm
#define DIST_THRESHOLD          1000  // mm     //#define DIST_THRESHOLD        1600  // mm
#define ROWS_OF_SPADS           8 // 8x16 SPADs ROI
#define TIMING_BUDGET           15  // in ms possible values [15, 20, 50, 100, 200, 500] 
                                //33  // was 20 ms, I found 33 ms has better succes rate with lower reflectance target     
#define DISTANCE_MODE           DISTANCE_MODE_LONG

#if ROWS_OF_SPADS == 4
#define FRONT_ZONE_CENTER       151
#define BACK_ZONE_CENTER        247
#elif ROWS_OF_SPADS == 6
#define FRONT_ZONE_CENTER       159
#define BACK_ZONE_CENTER        239
#elif ROWS_OF_SPADS == 8
#define FRONT_ZONE_CENTER       175 // was 167, see UM2555 on st.com, centre = 175 has better return signal rate for the ROI #1
#define BACK_ZONE_CENTER        231
#endif

int PplCounter = 0;
int bPplCounter = 9;
int center[2] = {FRONT_ZONE_CENTER, BACK_ZONE_CENTER}; // these are the spad center of the 2 4*16 zones 
int Zone = 0;

STMPE1600DigiOut xshutdown_top(&DEV_I2C, GPIO_15, (0x42 * 2));
VL53L1X_X_NUCLEO_53L1A1 vl53l1x(&DEV_I2C, &xshutdown_top);
//VL53L1X_X_NUCLEO_53L1A1 vl53l1x(&DEV_I2C, 15);

int ProcessPeopleCountingData(int16_t Distance, uint8_t zone, uint8_t RangeStatus) {
  static int PathTrack[] = {0,0,0,0};
  static int PathTrackFillingSize = 1;  // init this to 1 as we start from state where nobody is any of the zones
  static int LeftPreviousStatus = NOBODY;
  static int RightPreviousStatus = NOBODY;
  static int PeopleCount = 0;
  static uint16_t Distances[2][DISTANCES_ARRAY_SIZE];
  static uint8_t DistancesTableSize[2] = {0,0};
  uint16_t MinDistance;
  uint8_t i;

  (void)RangeStatus;

#ifdef TRACE_PPC
#define TIMES_WITH_NO_EVENT 7//10 was 40    
  static uint32_t trace_count = TIMES_WITH_NO_EVENT;  // replace by 0 if you want to trace the first TIMES_WITH_NO_EVENT values
#endif

  int CurrentZoneStatus = NOBODY;
  int AllZonesCurrentStatus = 0;
  int AnEventHasOccured = 0;

  if (DistancesTableSize[zone] < DISTANCES_ARRAY_SIZE){  // Add just picked distance to the table of the corresponding zone
    Distances[zone][DistancesTableSize[zone]] = Distance;
    DistancesTableSize[zone] ++;
  }else{
    for (i=1; i<DISTANCES_ARRAY_SIZE; i++)
      Distances[zone][i-1] = Distances[zone][i];
    Distances[zone][DISTANCES_ARRAY_SIZE-1] = Distance;
  }

  MinDistance = Distances[zone][0];   // pick up the min distance
  if (DistancesTableSize[zone] >= 2){
    for (i=1; i<DistancesTableSize[zone]; i++){
      if (Distances[zone][i] < MinDistance)
        MinDistance = Distances[zone][i];
    }
  }
  if (MinDistance < DIST_THRESHOLD){
    CurrentZoneStatus = SOMEONE;    // Someone is in !
  }
  if(zone == LEFT){ // left zone
    if(CurrentZoneStatus != LeftPreviousStatus){
      AnEventHasOccured = 1;      // event in left zone has occured
      if (CurrentZoneStatus == SOMEONE){
        AllZonesCurrentStatus += 1;
      }
      if (RightPreviousStatus == SOMEONE){      // need to check right zone as well ...
        AllZonesCurrentStatus += 2;        // event in left zone has occured
      }
      LeftPreviousStatus = CurrentZoneStatus;      // remember for next time
    }
  }else{ // right zone
    if (CurrentZoneStatus != RightPreviousStatus){
      AnEventHasOccured = 1;      // event in right zone has occured
      if (CurrentZoneStatus == SOMEONE){
        AllZonesCurrentStatus += 2;
      }
      if (LeftPreviousStatus == SOMEONE){      // need to check left zone as well ...
        AllZonesCurrentStatus += 1;        // event in left zone has occured
      }
      RightPreviousStatus = CurrentZoneStatus;      // remember for next time
    }
  }

#ifdef TRACE_PPC
  trace_count++;    // print debug data only when someone is within the field of view
  if ((CurrentZoneStatus == SOMEONE) || (LeftPreviousStatus == SOMEONE) || (RightPreviousStatus == SOMEONE))
    trace_count = 0;
  if (trace_count < TIMES_WITH_NO_EVENT){
    SerialPort.print(".");    //SerialPort.print(Distance);
    //if(Zone == 1) SerialPort.println();
  }
#endif

  if (AnEventHasOccured){  // if an event has occured
    if (PathTrackFillingSize < 4){
      PathTrackFillingSize++;
    }
    if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY)) {    // if nobody anywhere lets check if an exit or entry has happened
      if (PathTrackFillingSize == 4){      // check exit or entry only if PathTrackFillingSize is 4 (for example 0 1 3 2) and last event is 0 (nobobdy anywhere)
        SerialPort.println();
        if ((PathTrack[1] == 1)  && (PathTrack[2] == 3) && (PathTrack[3] == 2)){        // check exit or entry. no need to check PathTrack[0] == 0 , it is always the case
          PeopleCount ++;          // This an entry
          DistancesTableSize[0] = 0;          // reset the table filling size in case an entry or exit just found
          DistancesTableSize[1] = 0;
          SerialPort.print(" Walk In\t\t");
        } else if ((PathTrack[1] == 2)  && (PathTrack[2] == 3) && (PathTrack[3] == 1)){
          PeopleCount --;             // This an exit
          DistancesTableSize[0] = 0;          // reset the table filling size in case an entry or exit just found
          DistancesTableSize[1] = 0;
          SerialPort.print("   Walk Out\t\t");
        } else {
          DistancesTableSize[0] = 0;          // reset the table filling size also in case of unexpected path
          DistancesTableSize[1] = 0;
          SerialPort.print("\t ERROR\t\t");
        }
        SerialPort.println(PeopleCount);
      }
      PathTrackFillingSize = 1;
    } else{                              // update PathTrack        // 0       // example of PathTrack update
                                                                    // 0 1
                                                                    // 0 1 3
                                                                    // 0 1 3 1
                                                                    // 0 1 3 3
      PathTrack[PathTrackFillingSize-1] = AllZonesCurrentStatus;    // 0 1 3 2 ==> if next is 0 : check if exit
    }
  }
  return(PeopleCount);    // output debug data to main host machine
}

void setup(){
  SerialPort.begin(115200);
  SerialPort.println("Starting...");
  DEV_I2C.begin();

  vl53l1x.begin();         // Configure VL53L1X top component.
  vl53l1x.VL53L1X_Off();  // Switch off VL53L1X top component.
  vl53l1x.InitSensor(0x10);  //Initialize all the sensors 0x10 0x52;0x29
  vl53l1x.VL53L1X_SetDistanceMode(DISTANCE_MODE); /* 1=short, 2=long */
  vl53l1x.VL53L1X_SetTimingBudgetInMs(TIMING_BUDGET); /* in ms possible values [15, 20, 50, 100, 200, 500] */
  vl53l1x.VL53L1X_SetInterMeasurementInMs(TIMING_BUDGET);
  vl53l1x.VL53L1X_SetROI(ROWS_OF_SPADS, 16); /* minimum ROI 4,4 */
  vl53l1x.VL53L1X_StartRanging();   /* This function has to be called to enable the ranging */
}

void loop(){
  int status;
  uint16_t Distance, Signal;
  uint8_t RangeStatus;
  uint8_t dataReady = 0;

  while (dataReady == 0){  //Poll for measurament completion top sensor   측정 완료 상단 센서에 대한 투표
    status = vl53l1x.VL53L1X_CheckForDataReady(&dataReady);
    delay(1);
  }
  status += vl53l1x.VL53L1X_GetRangeStatus(&RangeStatus);
  status += vl53l1x.VL53L1X_GetDistance(&Distance);
  status += vl53l1x.VL53L1X_GetSignalPerSpad(&Signal);
  status += vl53l1x.VL53L1X_ClearInterrupt(); //clear interrupt has to be called to enable next interrupt
                                              //다음 인터럽트를 활성화하려면 클리어 인터럽트를 호출해야 합니다.
//  if (status != 0)   SerialPort.println("Error in operating the device");
  status = vl53l1x.VL53L1X_SetROICenter(center[Zone]);
//  if (status != 0){
//    SerialPort.println("Error in chaning the center of the ROI");
//    while(1){}
//  }
  // inject the new ranged distance in the people counting algorithm
  PplCounter = ProcessPeopleCountingData(Distance, Zone, RangeStatus);
//  SerialPort.print(PplCounter);
  Zone++;
  Zone = Zone%2;
//  if(Zone == 1) SerialPort.println();
}
