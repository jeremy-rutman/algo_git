
//1369913146 seconds on 30.05.13
//4067  ... 15:21 checked and it restaarted
// objective c tutorial - http://rypress.com/tutorials/objective-c/protocols.html
//  ViewController.m
//  testAlgo
//
//  Created by Itai David on 8/14/12.
//  Copyright (c) 2012 itaidddd@gmail.com. All rights reserved.
//
//
//DONE  get accurate timestamp and base velocity on actual integral (dont assume steady sample rate)
//state button
//logs

//TODO
//run using nstimer
//check battery use of acc, ori, gps, etc
//	on simless iphone3.1

//305.0

// lior neu-ner's iphone
//Hardware Model:      iPhone3,1
//OS Version:          iPhone OS 6.1.3 (10B329)
//Kernel version:      Darwin Kernel Version 13.0.0: Wed Feb 13 21:36:52 PST 2013; root:xnu-2107.7.55.2.2~1/RELEASE_ARM_S5L8930X

//tomer's  phone -
//OS Version:          iPhone OS 5.0.1 (9A405)
//Hardware Model: N90AP

//	YESACC battlevel=-6.507e-5t where t is in seconds for use of 0.1s updates acc (3values), ori this was with two other bgnd apps - parking finder and find my phone
//	YESACC battlevel=-8.42e-5t from 35% to 10%
//	YESACC batt=-8.78e-5 from 65% to 15% 280513    with 'video' recent app
//	NO ACC batt=-1.345e-4 from 100% to 70% 280513  with 'video' recent app

//	now doing acc (6 values), ori with no background apps
//check if runs in background
// http://speirs.org/blog/2012/1/2/misconceptions-about-ios-multitasking.html
//check logs for false positives/negatives
//add Astd min thresholds to walking detector

//dead reckoning
//parallel parking
//better walking detection
//to differentiate bus from car - look at number of stops per time, indirect route, {are u on bus route}

//driver behavior
//from http://www.oryarok.org.il/webfiles/audio_files/safety_caralation_MAtzeget.pdf :
//"The specific IVDR system evaluated in this paper was developed by DriveDiagnostics LTD"
//'risk index' is number of aggressive maneuvers per 10 driving hours....
//The identified maneuvers are also
//classified in terms of their severity based on parameters of the detailed trajectory of
//the vehicle during the maneuver, such as the maneuver duration, extent of sudden
//changes during the maneuver, the speed it is performed at and the magnitude of the
//forced applied on the vehicle.
//....to do this one could use mass from the vehicle type and acceleration for F=mA
//...'snapshot' plugs into car obd http://www.progressive.com/newsroom/images/snapshot_report_final_070812.pdf - look at acceleration, braking, turning, cornering - and speed!
//The American Association of Equipment Management Professionals’ telematics data standard focuses on location and three additional data points: distance traveled, driving time, and fuel consumption. The organization finds that these four data elements support 80% of their constituency’s reporting needs.See Bennink, page 1. Speed, acceleration, deceleration, and braking are less standard elements but are also commonly used in UBI.
//For example, braking events may be identified when certain gravitational forces (gforce) are registered and/or decelerations occur over a prescribed duration (e.g., Kantowitz definedbraking events by a 0.2 g-force deceleration rate over five seconds (http://www.casact.org/pubs/forum/12wforumpt2/Weiss-Smollik.pdf)
//if we use gps - then 'driving while talking' and 'driving while smsing' can be accurately detected (since driving is accurately detected by gps even when playing with phone)


#import <CoreMotion/CoreMotion.h>
#import "ViewController.h"
#import "drivingDetector.h"

//#import "AddTwoTextFieldViewController.h"
//#import "AddTwoTextFieldAppDelegate.h"
//GLOBAL VARIABLES

//static int jmN_samples_taken=0;
//NSString *lglobal_logFileName;



@interface CMDeviceMotion (TransformToReferenceFrame)
-(CMAcceleration)userAccelerationInReferenceFrame;
@end
//and in CMDeviceMotion+TransformToReferenceFrame.m:

//#import "CMDeviceMotion+TransformToReferenceFrame.h"

@implementation CMDeviceMotion (TransformToReferenceFrame)

-(CMAcceleration)userAccelerationInReferenceFrame
{
    CMAcceleration acc = [self userAcceleration];
    CMRotationMatrix rot = [self attitude].rotationMatrix;
    
    CMAcceleration accRef;
    accRef.x = acc.x*rot.m11 + acc.y*rot.m12 + acc.z*rot.m13;
    accRef.y = acc.x*rot.m21 + acc.y*rot.m22 + acc.z*rot.m23;
    accRef.z = acc.x*rot.m31 + acc.y*rot.m32 + acc.z*rot.m33;
    return accRef;
}

@end
//#import "ViewController.h"

@interface ViewController ()

@end

@implementation ViewController


//GLOBAL VARIABLES
BOOL debug;
NSString *claimedUserState;


-(void) getData{
    while (YES) {
		//NSLog(@"1");
        if (!debug &&  [UIApplication sharedApplication].applicationState == UIApplicationStateBackground ){
            debug = YES;
            [locationManager stopUpdatingLocation];
            [locationManager startMonitoringSignificantLocationChanges];
        }
    }
}



- (void)viewDidLoad
{

	
    [super viewDidLoad];
	// Do any additional setup after loading the view, typically from a nib.

    Gps_state = NO;
    AGps_state = NO;
    
    [[UIDevice currentDevice] setBatteryMonitoringEnabled:YES];
	
    NSLog(@"starting - 1");
/*
	SystemSoundID soundID;
	NSString *soundPath = [[NSBundle mainBundle] pathForResource:@"sound2" ofType:@"wav"];
	NSURL *soundUrl = [NSURL fileURLWithPath:soundPath];
	AudioServicesCreateSystemSoundID ((__bridge CFURLRef)soundUrl, &soundID);
	AudioServicesPlaySystemSound(soundID);
	*/
	

	myDrivingDetector=[[drivingDetector alloc] init];
	[myDrivingDetector drivingDetectorInit];

	
#ifdef DEBUG
	NSLog(@"starting - 2");
#endif

	
#ifdef READ_FILE
	NSLog(@"reading file");
	[self displayContent:filetoread ];
	exit(EXIT_SUCCESS);
		
#endif
	
	
#ifdef NO_IPHONE_ATTACHED
	NSLog(@"no iphone attached");
	
	//code from itai for nstimer
	NSTimer* updateTimer_;
	updateTimer_=[NSTimer
				  scheduledTimerWithTimeInterval:(1.0/(myDrivingDetector->mSamplingRate))
		//		  scheduledTimerWithTimeInterval:(1.0/2.0)
				  target:self
				  selector:@selector(timed_action)
				  userInfo:nil
				  repeats:YES];


}

#else
//int mSamplingRate=10;
	motionManager = [[CMMotionManager alloc] init];
    [motionManager setAccelerometerUpdateInterval:1.0/myDrivingDetector->mSamplingRate];
    [motionManager startDeviceMotionUpdates];
	AlgoVector* accVectorWC= [[AlgoVector alloc] init];
	AlgoVector* orientationVector= [[AlgoVector alloc] init];
	//   [motionManager startAccelerometerUpdatesToQueue:[[NSOperationQueue alloc] init]
	//										withHandler:^(CMAccelerometerData *data, NSError *error) {
	//											dispatch_async(dispatch_get_main_queue(), ^{
	
	
    [motionManager startAccelerometerUpdatesToQueue:[[NSOperationQueue alloc] init]
										withHandler:^(CMAccelerometerData *data, NSError *error)
	 {
		 dispatch_async(dispatch_get_main_queue(), ^
						{
							// Collecting data
							
							CMRotationMatrix rot = motionManager.deviceMotion.attitude.rotationMatrix;
							
							accVectorWC->x = data.acceleration.x*rot.m11 + data.acceleration.y*rot.m21 + data.acceleration.z*rot.m31;
							accVectorWC->y = data.acceleration.x*rot.m12 + data.acceleration.y*rot.m22 + data.acceleration.z*rot.m32;
							accVectorWC->z = data.acceleration.x*rot.m13 + data.acceleration.y*rot.m23 + data.acceleration.z*rot.m33;
							
							//give world coordinate accelerations in m/s^2 and no g
							accVectorWC->x *= 9.81f;
							accVectorWC->y *= 9.81f;
							accVectorWC->z *= 9.81f;
							
							// add 9.8 to z to zero effect of gravity
							accVectorWC->z += 9.81f;
							
							//orientation in degrees to be equivalet to android
							orientationVector->x=motionManager.deviceMotion.attitude.yaw*360/3.141;
							orientationVector->y=motionManager.deviceMotion.attitude.pitch*360/3.141;
							orientationVector->z=motionManager.deviceMotion.attitude.roll*360/3.141;
							
							//SEPARATE DRIVING FUNCTION FOR MODULARITY
							[myDrivingDetector drivingDetectorProcessInput:accVectorWC anOrientationVector:orientationVector];
							
							[self drivingDetectorDoGui:myDrivingDetector];
						});
	 }
     ];
    
}

#endif




- (void)locationManager:(CLLocationManager *)manager didUpdateToLocation:(CLLocation *)newLocation fromLocation:(CLLocation *)oldLocation {
	NSLog(@"location manager:%@",[newLocation description]);
	
	CLLocationCoordinate2D currentCoordinates = newLocation.coordinate;
	
	//    [alertLabel setText:@"Location Has been found"];
	//    [alertLabel setHidden:NO];
	//
	algoLocVector->lat = currentCoordinates.latitude;
	algoLocVector->lon = currentCoordinates.longitude;
	algoLocVector->accuracy = newLocation.horizontalAccuracy;
	algoLocVector->alt = newLocation.altitude;
	algoLocVector->isGPSLoc = newLocation.speed == -1.0f ?NO:YES;
	algoLocVector->bearing = newLocation.course;
	algoLocVector->sampleTimeInMsec =newLocation.timestamp.timeIntervalSince1970*1000.0f;
	algoLocVector->speed = newLocation.speed;
	//NSLog(@"gps time diff in MS %lld",(long long)(([[NSDate date] timeIntervalSince1970]*1000) - newLocation.timestamp.timeIntervalSince1970*1000));
	
	if (([[NSDate date] timeIntervalSince1970] - newLocation.timestamp.timeIntervalSince1970) < 1.5f)
		[algoDetection UpdateCurrLoc:algoLocVector];
	
	
	//    algoLocVector.lat = (float)[loc latitude];
	//    algoLocVector.lon = (float)[loc longitude];
	//    algoLocVector.alt = (float)[loc altitude];
	//    algoLocVector.bearing = (float)[loc bearing];
	//    algoLocVector.speed = (float)[loc speed];
	//    algoLocVector.accuracy = (float)[loc accuracy];
	//    algoLocVector.sampleTimeInMsec = [System currentTimeMillis];
	//    algoLocVector.isGPSLoc = NO;
	//    [algoDetection UpdateCurrLoc:algoLocVector];
	//    [loc setSpeed:algoLocVector.speed];
	NSLog(@"Lat%f Long%f Speed%2f Speed_fixed%2f Accuracy%f time %lld", currentCoordinates.latitude, currentCoordinates.longitude,newLocation.speed,algoLocVector->speed,newLocation.horizontalAccuracy,algoLocVector->sampleTimeInMsec);
	//NSLog(@"%f",newLocation.timestamp.timeIntervalSince1970);
}

- (void)locationManager:(CLLocationManager *)manager didFailWithError:(NSError *)error {
	NSLog(@"Unable to start location manager. Error:%@", [error description]);
	
	//    [alertLabel setHidden:NO];
}

- (void)applicationDidEnterBackground:(UIApplication *)application {
	
	UIApplication*    app = [UIApplication sharedApplication];
	
	// it's better to move "dispatch_block_t expirationHandler"
	// into your headerfile and initialize the code somewhere else
	// i.e.
	// - (void)applicationDidFinishLaunching:(UIApplication *)application {
	//
	// expirationHandler = ^{ ... } }
	// because your app may crash if you initialize expirationHandler twice.
	dispatch_block_t __block expirationHandler;
	expirationHandler = ^{
		
		[app endBackgroundTask:bgTask];
		bgTask = UIBackgroundTaskInvalid;
		
		
		bgTask = [app beginBackgroundTaskWithExpirationHandler:expirationHandler];
	};
	
	bgTask = [app beginBackgroundTaskWithExpirationHandler:expirationHandler];
	
	
	// Start the long-running task and return immediately.
	dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
		
		// inform others to stop tasks, if you like
		[[NSNotificationCenter defaultCenter] postNotificationName:@"MyApplicationEntersBackground" object:self];
		
		NSLog(@"ITAI BG");
		// do your background work here
	});
}




- (IBAction)butttonDIDpressed:(id)sender {
	//  NSLog(@"button pressed %d", sender);
	
	UISegmentedControl *segmentedControl = (UISegmentedControl *)sender;
	NSString *labeltext = [segmentedControl titleForSegmentAtIndex: [segmentedControl selectedSegmentIndex]];
	NSLog(@"button pressed %@", labeltext);
	switch ([segmentedControl selectedSegmentIndex])
	{
		case 0:
			claimedUserState=@"W";
			break;
		case 1:
			claimedUserState=@"D";
			break;
		case 2:
			claimedUserState=@"P";
			break;
		case 3:
			claimedUserState=@"Bus";
			break;
		case 4:
			claimedUserState=@"Bike";
			break;
	}
	
}


//Method retrieves content from documents directory and
//displays it in an alert
-(void) displayContent:(NSString*)fName {
	//get the documents directory:
	NSArray *paths = NSSearchPathForDirectoriesInDomains
	(NSDocumentDirectory, NSUserDomainMask, YES);
	NSError *error;
	NSString *documentsDirectory = [paths objectAtIndex:0];
	//NSArray * directoryContents = [[NSFileManager defaultManager] contentsOfDirectoryAtPath:documentsPath error:&error];
	NSArray * directoryContents = [[NSFileManager defaultManager] contentsOfDirectoryAtPath:documentsDirectory error:&error];
	NSLog(@"files:%@",directoryContents);
	
	//make a file name to write the data to using the documents directory:
	NSString *fileName = [NSString stringWithFormat:@"%@/%@",
						  documentsDirectory,fName];
	NSLog(@"READ:%@",fileName);
	
	NSString *content = [[NSString alloc] initWithContentsOfFile:fileName
													usedEncoding:nil
														   error:nil];
	//use simple alert from my library (see previous post for details)
	//[ASFunctions alert:content];
	NSLog(@"CONTENT:%@",content);
}



-(void) drivingDetectorDoGui:(drivingDetector*) theDrivingDetector
{
	NSString *resultString = [[NSString alloc]
							  initWithFormat:theDrivingDetector->statusString];
	_resultLabel0.text = resultString;
	
    //print WC accelerations
    NSString *resultString1 = [[NSString alloc]
                               initWithFormat: @"Ax %.2f", theDrivingDetector->accVectorWC->x];
    _resultLabel1.text = resultString1;
    [_resultLabel1 setTextColor:[UIColor whiteColor]];
    
    //	[CategoryLbl setTextColor:[UIColor colorWithRed:(38/255.f) green:(171/255.f) blue:(226/255.f) alpha:1.0f]];
    
    NSString *resultString2 = [[NSString alloc]
                               initWithFormat: @"Ay %.2f", theDrivingDetector->accVectorWC->y];
    _resultLabel2.text = resultString2;
    [_resultLabel2 setTextColor:[UIColor whiteColor]];
    
    NSString *resultString3 = [[NSString alloc]
                               initWithFormat: @"Az %.2f", theDrivingDetector->accVectorWC->z];
    _resultLabel3.text = resultString3;
    [_resultLabel3 setTextColor:[UIColor whiteColor]];
    
    
    //print unbiased accelerations
    //print orientations
    NSString *resultString4 = [[NSString alloc]
                               initWithFormat: @"Ax %.2f", theDrivingDetector->accVectorWC_unbiased->x];
    _resultLabel4.text = resultString4;
    [_resultLabel4 setTextColor:[UIColor whiteColor]];
    
    NSString *resultString5 = [[NSString alloc]
                               initWithFormat: @"Ay %.2f",theDrivingDetector->accVectorWC_unbiased->y];
    _resultLabel5.text = resultString5;
    [_resultLabel5 setTextColor:[UIColor whiteColor]];
    
    NSString *resultString6 = [[NSString alloc]
                               initWithFormat: @"Az %.2f",theDrivingDetector->accVectorWC_unbiased->z];
    _resultLabel6.text = resultString6;
    [_resultLabel6 setTextColor:[UIColor whiteColor]];
    
    //print orientations
    NSString *resultString7 = [[NSString alloc]
                               initWithFormat: @"Ox %.1f", theDrivingDetector->orientationVector->x];
    _resultLabel7.text = resultString7;
    [_resultLabel7 setTextColor:[UIColor whiteColor]];
    
    NSString *resultString8 = [[NSString alloc]
                               initWithFormat: @"Oy %.1f",theDrivingDetector->orientationVector->y];
    _resultLabel8.text = resultString8;
    [_resultLabel8 setTextColor:[UIColor whiteColor]];
    
    NSString *resultString9 = [[NSString alloc]
                               initWithFormat: @"Oz %.1f",theDrivingDetector->orientationVector->z];
    _resultLabel9.text = resultString9;
    [_resultLabel9 setTextColor:[UIColor whiteColor]];
    
    
    //		NSLog(@"Orientation:Ox %lf Oy %lf Oz %lf",theOrientationVector->x,theOrientationVector->y,theOrientationVector->z);
    
    //print velocities
    /*
     NSString *resultString10 = [[NSString alloc]
     initWithFormat: @"V2x %.1f",velocityVector_2s->x];
     _resultLabelV2x.text = resultString10;
     NSString *resultString11 = [[NSString alloc]
     initWithFormat: @"V2y %.1f",velocityVector_2s->y];
     _resultLabelV2y.text = resultString11;
     NSString *resultString12 = [[NSString alloc]
     initWithFormat: @"V2z %.1f",velocityVector_2s->z];
     _resultLabelV2z.text = resultString12;
     
     */
    //print Vh, Vzavg, Vzstd
    /*		NSString *resultString13 = [[NSString alloc]
     initWithFormat: @"Vh %.1f",mVhCurrent];
     _resultLabel13.text = resultString13;
     if(mVhCurrent>mdriving_velocity_threshold)[_resultLabel13 setTextColor:[UIColor greenColor]];
     else [_resultLabel13 setTextColor:[UIColor redColor]];
     */
    
    NSString *resultString14 = [[NSString alloc]
                                initWithFormat: @"VzAvg %.2f",theDrivingDetector->mVzAvg];
    _resultLabel14.text = resultString14;
    if(theDrivingDetector->mVzAvg<theDrivingDetector->mVzAvgMaxDrivingThreshold)[_resultLabel14 setTextColor:[UIColor greenColor]];
    else [_resultLabel14 setTextColor:[UIColor redColor]];
    
    NSString *resultString15 = [[NSString alloc]
                                initWithFormat: @"Vzstd %.2f",theDrivingDetector->mVzStd];
    _resultLabel15.text = resultString15;
    [_resultLabel15 setTextColor:[UIColor whiteColor]];
    
    if(theDrivingDetector->mVzStd>theDrivingDetector->mVzStdMinWalkingThreshold)[_resultLabel15 setTextColor:[UIColor redColor]];
    else if(theDrivingDetector->mVzStd<theDrivingDetector->mVzStdMaxDrivingThreshold)[_resultLabel15 setTextColor:[UIColor greenColor]];
    else [_resultLabel15 setTextColor:[UIColor whiteColor]];
    
    NSString *resultString16 = [[NSString alloc]
                                initWithFormat: @"Oystd2 %.1f",theDrivingDetector->mOy_std2s];
    _resultLabel16.text = resultString16;
    [_resultLabel16 setTextColor:[UIColor whiteColor]];
    
    if(theDrivingDetector->mOy_std2s>theDrivingDetector->mOyStd2sMinWalkingThreshold || theDrivingDetector->mOy_std2s<theDrivingDetector->mOyStd2sMinDrivingThreshold)
        [_resultLabel16 setTextColor:[UIColor redColor]];
    else if(theDrivingDetector->mOy_std2s<theDrivingDetector->mOyStd2sMaxDrivingThreshold) [_resultLabel16 setTextColor:[UIColor greenColor]];
    else [_resultLabel16 setTextColor:[UIColor whiteColor]];
    
    NSString *resultString17 = [[NSString alloc]
                                initWithFormat: @"Oystd20 %.1f",theDrivingDetector->mOy_std20s];
    _resultLabel17.text = resultString17;
    if(theDrivingDetector->mOy_std20s>theDrivingDetector->mOyStd20sMinWalkingThreshold)[_resultLabel17 setTextColor:[UIColor redColor]];
    else if(theDrivingDetector->mOy_std20s<theDrivingDetector->mOyStd20sMaxDrivingThreshold)[_resultLabel17 setTextColor:[UIColor greenColor]];
    else [_resultLabel17 setTextColor:[UIColor whiteColor]];
    
    if(theDrivingDetector->mWalking)
    {
        NSString *resultString18 = [[NSString alloc]
                                    initWithFormat: @"WALKING"];
        _resultLabel18.text = resultString18;
    setTextColor:[UIColor greenColor];
        theDrivingDetector->timeOfLastNotification=[[NSDate date] timeIntervalSince1970];
    }
    else if(theDrivingDetector->mDriving)
    {
        NSString *resultString18 = [[NSString alloc]
                                    initWithFormat: @"DRIVING"];
        _resultLabel18.text = resultString18;
        [_resultLabel18 setTextColor:[UIColor greenColor]];
        theDrivingDetector->timeOfLastNotification=[[NSDate date] timeIntervalSince1970];
    }
    else
    {
        if([[NSDate date] timeIntervalSince1970]-theDrivingDetector->timeOfLastNotification > 5)
        {
            NSString *resultString18 = [[NSString alloc] initWithFormat: @""];
            _resultLabel18.text = resultString18;
            [_resultLabel1 setTextColor:[UIColor whiteColor]];
            
        }
    }
    
    NSString *resultString19 = [[NSString alloc] initWithFormat: @"Ozstd2 %.1f",theDrivingDetector->mOz_std2s];
    _resultLabel19.text = resultString19;
    if(theDrivingDetector->mOz_std2s>theDrivingDetector->mOzStd2sMinWalkingThreshold || theDrivingDetector->mOz_std2s<theDrivingDetector->mOzStd2sMinDrivingThreshold)
        [_resultLabel19 setTextColor:[UIColor redColor]];
    else if(theDrivingDetector->mOz_std2s<theDrivingDetector->mOzStd2sMaxDrivingThreshold)[_resultLabel19 setTextColor:[UIColor greenColor]];
    else [_resultLabel19 setTextColor:[UIColor whiteColor]];
    
    NSString *resultString20 = [[NSString alloc] initWithFormat: @""];
    _resultLabel20.text = resultString20;
    [_resultLabel20 setTextColor:[UIColor whiteColor]];
    
    NSString *resultString21 = [[NSString alloc] initWithFormat: @"V2x %.2f",theDrivingDetector->velocityVector_2s->x];
    _resultLabelV2x.text = resultString21;
    [_resultLabelV2x setTextColor:[UIColor whiteColor]];
    
    resultString21 = [[NSString alloc] initWithFormat: @"V2y %.3f",theDrivingDetector->velocityVector_2s->y];
    _resultLabelV2y.text = resultString21;
    [_resultLabelV2y setTextColor:[UIColor whiteColor]];
    
    resultString21 = [[NSString alloc] initWithFormat: @"V2z %.3f",theDrivingDetector->velocityVector_2s->z];
    _resultLabelV2z.text = resultString21;
    [_resultLabelV2z setTextColor:[UIColor whiteColor]];
    
    resultString21 = [[NSString alloc] initWithFormat: @"V5x %.3f",theDrivingDetector->velocityVector_5s->x];
    _resultLabelV5x.text = resultString21;
    [_resultLabelV5x setTextColor:[UIColor whiteColor]];
    
    resultString21 = [[NSString alloc] initWithFormat: @"V5y %.3f",theDrivingDetector->velocityVector_5s->y];
    _resultLabelV5y.text = resultString21;
    [_resultLabelV5y setTextColor:[UIColor whiteColor]];
    
    resultString21 = [[NSString alloc] initWithFormat: @"V5z %.3f",theDrivingDetector->velocityVector_5s->z];
    _resultLabelV5z.text = resultString21;
    [_resultLabelV5z setTextColor:[UIColor whiteColor]];
    
    resultString21 = [[NSString alloc] initWithFormat: @"V10x %.3f",theDrivingDetector->velocityVector_10s->x];
    _resultLabelV10x.text = resultString21;
    [_resultLabelV10x setTextColor:[UIColor whiteColor]];
    
    resultString21 = [[NSString alloc] initWithFormat: @"V10y %.3f",theDrivingDetector->velocityVector_10s->y];
    _resultLabelV10y.text = resultString21;
    [_resultLabelV10y setTextColor:[UIColor whiteColor]];
    
    resultString21 = [[NSString alloc] initWithFormat: @"V10z %.3f",theDrivingDetector->velocityVector_10s->z];
    _resultLabelV10z.text = resultString21;
    [_resultLabelV10z setTextColor:[UIColor whiteColor]];
    
    resultString21 = [[NSString alloc] initWithFormat: @"V2h %.3f",theDrivingDetector->V2h];
    _resultLabelV2h.text = resultString21;
    [_resultLabelV2h setTextColor:[UIColor whiteColor]];
    
    resultString21 = [[NSString alloc] initWithFormat: @"V5h %.3f",theDrivingDetector->V5h];
    if(theDrivingDetector->V5h>theDrivingDetector->mdriving_velocity_threshold_5s )[_resultLabelV5h setTextColor:[UIColor greenColor]];
    else [_resultLabelV5h setTextColor:[UIColor redColor]];
    _resultLabelV5h.text = resultString21;
    
    resultString21 = [[NSString alloc] initWithFormat: @"V10h %.3f",theDrivingDetector->V10h];
    _resultLabelV10h.text = resultString21;
    if(theDrivingDetector->V10h>theDrivingDetector->mdriving_velocity_threshold_10s )[_resultLabelV10h setTextColor:[UIColor greenColor]];
    else [_resultLabelV10h setTextColor:[UIColor redColor]];
    
    //dOx/dt
    resultString21 = [[NSString alloc] initWithFormat: @"dOx/dt %3.2f",theDrivingDetector->dOx_dt];
    _resultLabeldOx_dt.text = resultString21;
    [_resultLabeldOx_dt setTextColor:[UIColor whiteColor]];
}


-(void)timed_action //(driving_detector*) theDrivingDetector

{
	NSLog(@"doing timed action");
    //	jstructure.mSamplingRate=100.0;
	

	myDrivingDetector->accVectorWC->x = 1.0+ (double)(arc4random() % 10)/100.0;
	myDrivingDetector->accVectorWC->y = 2.0+(double)(arc4random() % 10)/100.0;
	myDrivingDetector->accVectorWC->z = 3.0+(double)(arc4random() % 10)/100.0;
	myDrivingDetector->orientationVector->x=4+ (double)(arc4random() % 10)/100.0;
	myDrivingDetector->orientationVector->y=5+ (double)(arc4random() % 10)/100.0;
	myDrivingDetector->orientationVector->z=6+ (double)(arc4random() % 10)/100.0;
	
	[myDrivingDetector drivingDetectorProcessInput:myDrivingDetector->accVectorWC anOrientationVector:myDrivingDetector->orientationVector];
	
	[self  drivingDetectorDoGui:myDrivingDetector];

	
}


@end

//remove
/*
 Trivial wrapper around system sound as provided
 by Audio Services. Don’t forget to add the Audio
 Toolbox framework.
 */

//#import "Sound.h"


