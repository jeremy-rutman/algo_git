#import "MainService.h"


////////////////////////////////////////////////////////////////////////
@implementation AlgoVector

- (id) init {
  if (self = [super init]) {
    largestSize = 0;
    sampleTimeInMsec = 0;
    x = 0;
    y = 0;
    z = 0;
    size = -1;
    accuracy = -1;
  }
  return self;
}

- (void) Copy:(AlgoVector *)algVec {
  sampleTimeInMsec = algVec->sampleTimeInMsec;
  x = algVec->x;
  y = algVec->y;
  z = algVec->z;
  size = algVec->size;
  accuracy = 0;
}

- (float) ReCalclulateSize {
  if (accuracy < 0)
    return -1;
  size = sqrt(x * x + y * y + z * z);
  return size;
}

@end

@implementation AlgoLocVector

- (void) SetCurrLocAsRefLoc {
  lonRef = lon;
  latRef = lat;
  accuracyRef = accuracy;
  sampleTimeImMsecRef = sampleTimeInMsec;
  stepsCntRecordRef = stepsCntRecord;
}

- (id) init {
  if (self = [super init]) {
    DegreeToMeters = 111180;
    MinAccuracyMeters = 30;
    sampleTimeInMsec = 0;
    lon = 0;
    lat = 0;
    alt = 0;
    speed = 0;
    bearing = 0;
    accuracy = 10000;
    latRef = -1;
    lonRef = -1;
    lonFactor = -1;
    isGPSLoc = NO;
    stepsCntRecord = 0;
    sampleTimeImMsecRef = 0;
    stepsCntRecordRef = 0;
  }
  return self;
}

- (void) Copy:(AlgoLocVector *)algLocVec {
  sampleTimeInMsec = algLocVec->sampleTimeInMsec;
  lon = algLocVec->lon;
  lat = algLocVec->lat;
  alt = algLocVec->alt;
  bearing = algLocVec->bearing;
  speed = algLocVec->speed;
  accuracy = algLocVec->accuracy;
  isGPSLoc = algLocVec->isGPSLoc;
  stepsCntRecord = algLocVec->stepsCntRecord;
}

- (void) CalculateDistanceVector:(AlgoLocVector *)algoLocVecRef algoVec:(AlgoVector *)algoVec {
  if (lonFactor == -1)
    lonFactor = cos(2.0 * M_PI / 360.0 * algoLocVecRef->lat);
  algoVec->x = (lon - algoLocVecRef->lon) * lonFactor * DegreeToMeters;
  algoVec->y = (lat - algoLocVecRef->lat) * DegreeToMeters;
  algoVec->size = 0;
  algoVec->accuracy = accuracy + algoLocVecRef->accuracy;
  [algoVec ReCalclulateSize];
}

- (void) InvalidateRefLoc {
  latRef = -1;
  lonRef = -1;
}

- (void) CalculateDistanceVector:(AlgoVector *)algoVec {
  if (latRef == -1) {
    algoVec->size = -1;
    return;
  }
  if (lonFactor == -1)
  lonFactor = cos(2.0 * M_PI / 360.0 * latRef);
  algoVec->x = (lon - lonRef) * lonFactor * DegreeToMeters;
  algoVec->y = (lat - latRef) * DegreeToMeters;
  algoVec->size = 0;
  algoVec->accuracy = 0;
  [algoVec ReCalclulateSize];
}

@end



NSString * MovementStatesDescription(MovementStates value) {
  switch (value) {
    case Steady:
      return @"Steady";
    case RockSteady:
      return @"RockSteady";
    case Driving:
      return @"Driving";
    case Parking:
      return @"Parking";
    case Walking:
      return @"Walking";
    case Unknown:
      return @"Unknown";
  }
  return nil;
}



@implementation AlgoDetection

- (id) init {
  if (self = [super init]) {
    accVecLastPick = [[AlgoVector alloc] init];
    accVecCurrPick = [[AlgoVector alloc] init];
    accVecCurrMax =  [[AlgoVector alloc] init];
    distanceToCarVec = [[AlgoVector alloc] init];
    algoCurrLocVector = [[AlgoLocVector alloc] init];
    lastLocations = [[NSMutableArray alloc] init];

    //TODO: ITAI SET the desired accuracy, so if walking with 30m I won't stop it on driving suspected with 100m accuracy 
  }
  return self;
}

- (void) isChargerConnected:(BOOL)_isChargerConnected {
  if (isChargerConnected != _isChargerConnected)
        NSLog((_isChargerConnected?@"charger connected":@"charger disconnected"));
  isChargerConnected = _isChargerConnected;
}

- (void) UpdateCurrLoc:(AlgoLocVector *)algLocVec {

}

- (void) ProcessNewAccVector:(AlgoVector *)accVec {

}

- (void) ProcessMovmentState:(long long)currTimeInMs {
  isGPSneededLastState = isGPSneeded;
  algoSensorLastState = algoSensorState;
  isStateChanged = NO;
  lastMovementState = movementState;
  lastWalkingState = walkingState;

}

- (void) SetAlgoState:(AlgoSensorState)algoState {
}


@end
