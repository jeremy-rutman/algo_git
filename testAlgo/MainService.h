




//@interface MainService : Service <SensorEventListener> {
//  MyApplication * appState;
//  LocationManager * lm;
//  SensorManager * mSensorManager;
//  Sensor * accelerometer;
//  Sensor * magnetometer;
//  AlgoDetection * algoDetection;
//  AlgoVector * accVector;
//  AlgoLocVector * algoLocVector;
//  long lastToastTime;
//  NSString * value;
//  long delayBetweenSamples;
//  long numOfSamples;
//  int dummy1;
//  SimpleDateFormat * sdf;
//  Date * date;
//  IBinder * mBinder;
//  LocationListener * networkLocationListener;
//  LocationListener * gpsLocationListener;
//  NSArray * mGravity;
//  NSArray * mGeomagnetic;
//  BroadcastReceiver * PowerConnectedReceiver;
//  BroadcastReceiver * PowerDisconnectedReceiver;
//}

//- (void) init;
//- (void) onCreate;
//- (int) onStartCommand:(Intent *)intent flags:(int)flags startId:(int)startId;
//- (void) onDestroy;
//- (IBinder *) onBind:(Intent *)intent;
//- (void) writeFile:(NSString *)text;
//- (void) onSensorChanged:(SensorEvent *)event;
//- (void) onAccuracyChanged:(Sensor *)arg0 arg1:(int)arg1;
//- (void) setParkingLocation:(AlgoLocVector *)algoCurrLocVector;
//- (void) getParkingLocation:(AlgoLocVector *)algoCurrLocVector;
//- (void) stopSensor;
//- (void) startCharger;
//- (void) stopCharger;
//- (void) showPopUp:(NSString *)message;
//- (void) SendAlgoLoc;

Boolean Gps_state;
Boolean AGps_state;

@interface AlgoVector : NSObject {
@public
  long long sampleTimeInMsec;
  float x;
  float y;
  float z;
  float size;
  float accuracy;
  float largestSize;
}

- (id) init;
- (void) Copy:(AlgoVector *)algVec;
- (float) ReCalclulateSize;
@end

@interface AlgoLocVector : NSObject {
@public
  long long sampleTimeInMsec;
  float lon;
  float lat;
  float alt;
  float bearing;
  float speed;
  float accuracy;
  float DegreeToMeters;
  float MinAccuracyMeters;
  float lastAccuracy;
  BOOL isGPSLoc;
  float lonRef;
  float latRef;
  float accuracyRef;
  long long sampleTimeImMsecRef;
  long stepsCntRecord;
  long stepsCntRecordRef;
  float lonFactor;
}

- (void) SetCurrLocAsRefLoc;
- (id) init;
- (void) Copy:(AlgoLocVector *)algLocVec;
- (void) CalculateDistanceVector:(AlgoLocVector *)algoLocVecRef algoVec:(AlgoVector *)algoVec;
- (void) InvalidateRefLoc;
- (void) CalculateDistanceVector:(AlgoVector *)algoVec;
@end

typedef enum {
  Steady,
  RockSteady,
  Driving,
  Parking,
  Walking,
  Unknown
} MovementStates;

MovementStates MovementStatesValueOf(NSString *text);
NSString *  MovementStatesDescription(MovementStates value);

typedef enum {
  WalkingToCar50m,
  WalkingToCarUnder10m,
  WalkingFromCar100m,
  WalkingInBuilding,
  WalkingUnknown
} WalkingStates;

WalkingStates WalkingStatesValueOf(NSString *text);
WalkingStates WalkingStatesDescription(WalkingStates value);

typedef enum {
  FastSearch,
  SlowSearch,
  SleepSearch
} AlgoSensorState;

AlgoSensorState AlgoSensorStateValueOf(NSString *text);
AlgoSensorState AlgoSensorStateDescription(AlgoSensorState value);

@interface AlgoDetection : NSObject {
    @public
  AlgoVector * accVecLastPick;
  AlgoVector * accVecCurrPick;
  AlgoVector * accVecCurrMax;
  AlgoVector * distanceToCarVec;
  long long searchStartTimeInMsec;
  long stepsBeforeNextGpsSample;
  float searchForPickLevel;
  float stopSearchForPickLevel;
  float searchForPickLevelStop;
  long searchForPickPeriod;
  long searchForPickMutePeriod;
  Boolean isSearchForHighPick;
  float stepsRate;
  float stepSize;
  Boolean isAbovePickLevel;
  Boolean isWalking;
  Boolean isStateChanged;
  int stepsCnt;
  float stepsRateWalkingThreshold;
  float stepsRateSteadyThreshold;
  long maxTimeToRecalculateStepRate;
  MovementStates movementState;
  MovementStates lastMovementState;
  WalkingStates walkingState;
  WalkingStates lastWalkingState;
  AlgoSensorState algoSensorState;
  AlgoSensorState algoSensorLastState;
  AlgoLocVector * algoCurrLocVector;
  NSMutableArray * lastLocations;
  float minSpeedForDriving;
  float minGpsAccuracyForDriving;
  Boolean isGPSneeded;
  Boolean isGPSneededLastState;
  long long timeWhenEnteredCurrState;
  int stepsCntWhenEnteredCurrState;
  long maxTimeInUknownState;
  long maxTimeForGPSon;
  long long timeWhenGPSactivated;
  long minTimeForGPSon;
  Boolean isRockSteady;
  float rockSteadyMargin;
  float earthGravity;
  BOOL isChargerConnected;
  long periodInCurrAlgoState;
}

- (id) init;
- (void) isChargerConnected:(BOOL)isChargerConnected;
- (void) UpdateCurrLoc:(AlgoLocVector *)algLocVec;
- (void) ProcessNewAccVector:(AlgoVector *)accVec;
- (void) ProcessMovmentState:(long long)currTimeInMs;
- (void) SetAlgoState:(AlgoSensorState)algoState;
@end
