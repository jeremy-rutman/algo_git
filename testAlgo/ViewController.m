//#define NO_IPHONE_ATTACHED
//#define DEBUGJ
//#define READ_FILE

//
//  ViewController.m
//  testAlgo
//
//  Created by Itai David on 8/14/12.
//  Copyright (c) 2012 itaidddd@gmail.com. All rights reserved.
//
//
//DONE  get accurate timestamp and base velocity on actual integral (dont assume steady sample rate)
//state button

//TODO
// check if runs in background
//check logs for false positives/negatives
//add Astd min thresholds to walking detector
//dead reckoning
//parallel parking
//better walking detection

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
#import <AudioToolbox/AudioToolbox.h>
#import <AudioToolbox/AudioServices.h>



//#import "AddTwoTextFieldViewController.h"
//#import "AddTwoTextFieldAppDelegate.h"

//Jeremy's structures
typedef struct tag1 {
	int time;
	double vals[10];  //A_wc, A_wc_unbiased, Orientation
    /*	double Ay;
     double Az;
     double Ox;
     double Oy;
     double Oz;
     */} Cdatastructure;

typedef struct tag2
{
	int mNfields;
	double mMaxAcceptedAcceleration;

	int mDuration_of_driving_analysis;
	int mN_samples_in_driving_analysis;
	int mN_samples_in_1s_analysis;
	int mN_samples_in_2s_analysis;
	int mN_samples_in_20s_analysis;
	int mN_samples_in_5s_analysis;
	
//walking thresholds
	double mVzStdMinWalkingThreshold;
	double mOyStd2sMinWalkingThreshold;
	double mOyStd20sMinWalkingThreshold;
	double mOzStd2sMinWalkingThreshold;
	
//driving thresholds
	double mdriving_velocity_threshold_2s;
	double mdriving_velocity_threshold_5s;
	double mdriving_velocity_threshold_10s;
	double mVzStdMaxDrivingThreshold;
	double mVzStdMinDrivingThreshold;
	double mVzAvgMaxDrivingThreshold;
	double mOyStd20sMaxDrivingThreshold;
	double mOyStd2sMaxDrivingThreshold;
	double mOyStd2sMinDrivingThreshold;
	double mOzStd2sMaxDrivingThreshold;
	double mOzStd2sMinDrivingThreshold;
	double time_of_last_mdriving_velocity_threshold_2s;
	double time_of_last_mdriving_velocity_threshold_5s;
	double time_of_last_mdriving_velocity_threshold_10s;
	double mDuration_in_which_to_accept_high_velocities;
		
	
//crazy driver thresholds
	double crazy_driver_velocity_threshold;
	double crazy_driver_dOx_dt_threshold; 
	double crazy_driver_Ah_threshold; 
	

	int mN_samples_taken_initial_value;
	int mN_samples_taken_current_value;
	int mN_samples_taken;
	int mVzCount;
	Cdatastructure mSensor_history[200];  //mN_samples_in_driving_analysis - itay didnt want #defines and I dont want to have to malloc
	int mEnough_driving_samples;
	double V[3];
	double mVz[200];  //mN_samples_in_driving_analysis =
	int mSampleCount20s;
	int mSampleCount2s;
	double mOy_20s[200]; //mN_samples_in_driving_analysis
	double mOy_2s[20]; //mN_samples_in_2s_analysis
	double mOz_2s[20]; //mN_samples_in_2s_analysis
	int mWalking;
	int mDriving;
	int mTimeofLastWalkingDetection;
	int mTimeofLastDrivingDetection;
	int mtime_after_walking_to_filter_driving;
	double mExpectationTimeBetweenDrivingDetections;
	int mSamplingRate; //samples per second
	double mBeta;
	double Vhmax;
	//	double Vh2max;
	double PrevioustimeStamp;
	NSString *thelogFileName;
    Boolean noInterveningWalkingFlag;
} jeremystruct;

static jeremystruct jstructure;
static int jmN_samples_taken=0;
//NSString *logFileName;

//Jeremy




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



BOOL debug;
long acc_i=0;
NSString *popUpString;

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

float batteryLevel=0;
int totalSamples;
Boolean isDriving;
NSString *claimedUserState;

- (void)viewDidLoad
{
	
    [super viewDidLoad];
	// Do any additional setup after loading the view, typically from a nib.
    algoDetection = [[AlgoDetection alloc] init];
    accVector = [[AlgoVector alloc] init];
	
    orientationVector=[[AlgoVector alloc] init];
    accVectorWC= [[AlgoVector alloc] init];
    accVectorWC_unbiased= [[AlgoVector alloc] init];
	algoLocVector = [[AlgoLocVector alloc] init];
    locationManager = [[CLLocationManager alloc] init];
	
    locationManager.delegate = self;
    locationManager.desiredAccuracy = kCLLocationAccuracyThreeKilometers;//kCLLocationAccuracyKilometer;
    locationManager.distanceFilter = 10000;
    //[locationManager setPurpose:@"My Custom Purpose Message..."];
    //[locationManager startUpdatingLocation];
    //[locationManager startMonitoringSignificantLocationChanges];
    Gps_state = NO;
    AGps_state = NO;
    
    [[UIDevice currentDevice] setBatteryMonitoringEnabled:YES];
	
    NSLog(@"starting - 1");
	
	SystemSoundID soundID;
	NSString *soundPath = [[NSBundle mainBundle] pathForResource:@"sound2" ofType:@"wav"];
	NSURL *soundUrl = [NSURL fileURLWithPath:soundPath];
	AudioServicesCreateSystemSoundID ((CFURLRef)soundUrl, &soundID);
	AudioServicesPlaySystemSound(soundID);
	
	
	NSString *resultString = [[NSString alloc]
							  initWithFormat: @"STARTING"];
	_resultLabel0.text = resultString;
	
	
#ifdef DEBUG
	NSLog(@"starting - 2");
#endif

#ifdef NO_IPHONE_ATTACHED
	NSLog(@"no iphone attached");

	while(1)
	{
		jstructure.mSamplingRate=10.0;
		
		accVectorWC->x = 1.0+ (double)(arc4random() % 10)/100.0;
		accVectorWC->y = 2.0+(double)(arc4random() % 10)/100.0;
		accVectorWC->z
		= 3.0+(double)(arc4random() % 10)/100.0;
		orientationVector->x=4+ (double)(arc4random() % 10)/100.0;
		orientationVector->y=5+ (double)(arc4random() % 10)/100.0;
		orientationVector->z=6+ (double)(arc4random() % 10)/100.0;
		[self detectDriving:accVectorWC anOrientationVector:orientationVector ];
		//	[NSThread sleepForTimeInterval:100];
		usleep(100000);
	}
#endif
	
	
#ifdef READ_FILE
	NSLog(@"reading file");
	[self displayContent:@"logfile.txt" ];
	exit(EXIT_SUCCESS);
		
#endif
	
	jstructure.mSamplingRate=10;  //make sure this agrees with no-iphone sampling rate
	motionManager = [[CMMotionManager alloc] init];
    [motionManager setAccelerometerUpdateInterval:1.0/jstructure.mSamplingRate];
    [motionManager startDeviceMotionUpdates];

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
							
							//NEW FUNCTION FOR MODULARITY
							[self detectDriving:accVectorWC anOrientationVector:orientationVector];
							
						});
		 
	 }
     ];
    
}



// new function taking inputs of algovectors for orientation, acceleration so i can fake those if there is no iphone handy
- (void)detectDriving:  (AlgoVector *)theAccelerationVector anOrientationVector:(AlgoVector *)theOrientationVector
{
    [super viewDidLoad];
	// Do any additional setup after loading the view, typically from a nib.
//    algoDetection = [[AlgoDetection alloc] init];
//    accVector = [[AlgoVector alloc] init];
//    orientationVector = theOrientationVector;
	
	AlgoVector * theaccVectorWC_unbiased= [[AlgoVector alloc] init];
//    accVectorWC_unbiased= [[AlgoVector alloc] init];

	velocityVector_1s=[[AlgoVector alloc] init];
	velocityVector_2s=[[AlgoVector alloc] init];
	velocityVector_5s=[[AlgoVector alloc] init];
	velocityVector_10s=[[AlgoVector alloc] init];
	
    algoLocVector = [[AlgoLocVector alloc] init];
    locationManager = [[CLLocationManager alloc] init];
	
	//jeremy
    __block double ax_offset,ay_offset,az_offset; //these are the long-timescale biases of the acceleration vector (assumed to be 'really' 0)
	__block int i;
	__block int j;
	__block double timeStamp;
	__block double timeOfLastNotification;
	
	__block double mVhCurrent;
	__block double mVzStd=0;
	__block double mVzAvg=0;
	__block double mOy_std20s;
	__block double mOy_avg20s;
	__block double mOy_std2s;
	__block double mOy_avg2s;
	__block double mOz_avg2s=0;
	__block double mOz_std2s=0;
	__block double dOx_dt=0;
	__block int mWalking=0;
	__block int mDriving=0;
	__block Boolean mdriving_velocity_threshold_2s_in_recent_history;
	__block Boolean mdriving_velocity_threshold_5s_in_recent_history;
	__block Boolean mdriving_velocity_threshold_10s_in_recent_history;
	
    locationManager.delegate = self;
    locationManager.desiredAccuracy = kCLLocationAccuracyThreeKilometers;//kCLLocationAccuracyKilometer;
    locationManager.distanceFilter = 10000;
    //[locationManager setPurpose:@"My Custom Purpose Message..."];
    //[locationManager startUpdatingLocation];
    //[locationManager startMonitoringSignificantLocationChanges];
    Gps_state = NO;
    AGps_state = NO;
    
    [[UIDevice currentDevice] setBatteryMonitoringEnabled:YES];
	
	isDriving=0;
	
	//impose limits on measured data 
	if(theAccelerationVector->x>jstructure.mMaxAcceptedAcceleration) theAccelerationVector->x=jstructure.mMaxAcceptedAcceleration;
	if(theAccelerationVector->y>jstructure.mMaxAcceptedAcceleration) theAccelerationVector->y=jstructure.mMaxAcceptedAcceleration;
	if(theAccelerationVector->z>jstructure.mMaxAcceptedAcceleration) theAccelerationVector->z=jstructure.mMaxAcceptedAcceleration;
	if(theAccelerationVector->x<-jstructure.mMaxAcceptedAcceleration) theAccelerationVector->x=-jstructure.mMaxAcceptedAcceleration;
	if(theAccelerationVector->y<-jstructure.mMaxAcceptedAcceleration) theAccelerationVector->y=-jstructure.mMaxAcceptedAcceleration;
	if(theAccelerationVector->z<-jstructure.mMaxAcceptedAcceleration) theAccelerationVector->z=jstructure.mMaxAcceptedAcceleration;
	
	
	
	
	//initialize values if first time thru
	if(jmN_samples_taken==0) {
		NSLog(@"First run!");
		jstructure.mNfields=10; //fields in dataq record - Ax Ay Az Ax_no(no offset) Ay_no Az_no Ox Oy Oz
		jstructure.mDuration_of_driving_analysis=20; //20s analysis duration total
		//jstructure.mSamplingRate=10.0; //in samples/second  //defined in calling function since it needs to know for the iphone setup
		jstructure.mN_samples_in_driving_analysis=jstructure.mDuration_of_driving_analysis*jstructure.mSamplingRate;
		jstructure.mN_samples_in_1s_analysis=1*jstructure.mSamplingRate;
		jstructure.mN_samples_in_2s_analysis=2*jstructure.mSamplingRate;
		jstructure.mN_samples_in_5s_analysis=5*jstructure.mSamplingRate;
		jstructure.mN_samples_in_20s_analysis=20*jstructure.mSamplingRate;

		jstructure.mVzStdMinWalkingThreshold=1.0; //.08
		jstructure.mOyStd2sMinWalkingThreshold=10.0;
		jstructure.mOyStd20sMinWalkingThreshold=10.0;
		jstructure.mOzStd2sMinWalkingThreshold=5.0;
		
		jstructure.mdriving_velocity_threshold_2s=0.04; //was 0.5  V10h  reaches abt .016kkk
		jstructure.mdriving_velocity_threshold_5s=0.03; //was 0.5  V10h  reaches abt .016kkk
		jstructure.mdriving_velocity_threshold_10s=0.015; //was 0.5  V10h  reaches abt .016kkk
		jstructure.mVzStdMaxDrivingThreshold=0.5;
		jstructure.mVzAvgMaxDrivingThreshold=0.7;
		jstructure.mOyStd20sMaxDrivingThreshold=7.0;
		jstructure.mOyStd2sMaxDrivingThreshold=5.0;
		jstructure.mOyStd2sMinDrivingThreshold=0.2;
		
		jstructure.mOzStd2sMaxDrivingThreshold=5.0;
		jstructure.mOzStd2sMinDrivingThreshold=0.2;
		jstructure.time_of_last_mdriving_velocity_threshold_2s=0;
		jstructure.time_of_last_mdriving_velocity_threshold_5s=0;
		jstructure.time_of_last_mdriving_velocity_threshold_10s=0;
		jstructure.mDuration_in_which_to_accept_high_velocities=5; //look at last 5 sec
		
		//crazy driver profiling
		jstructure.crazy_driver_velocity_threshold=1.5; //was 0.5
		jstructure.crazy_driver_dOx_dt_threshold=350; //was 0.5
		jstructure.crazy_driver_Ah_threshold=1; //was 0.5

		jstructure.mEnough_driving_samples=0;  //0 false, 1 true
		jstructure.mBeta=0.95; //in samples/second
		jstructure.V[0]=0.0;jstructure.V[1]=0.0;jstructure.V[2]=0.0;
		jstructure.Vhmax=0.0; //jstructure.Vh2max=0.0;
		jstructure.mVzCount=0;
		jstructure.mtime_after_walking_to_filter_driving=5;
		jstructure.mTimeofLastDrivingDetection=0;
		jstructure.mTimeofLastWalkingDetection=0;
		jstructure.mExpectationTimeBetweenDrivingDetections=5*60; //if 5 minutes or less from last driving detection and there is no walking detected, we are still driving
		jstructure.mMaxAcceptedAcceleration=10;
		
		//			[initWithPath (NSString*) @""];
		
		//						mV_uncorrected[0]=0.0;
		
		jstructure.PrevioustimeStamp=[[NSDate date] timeIntervalSince1970]-.1;
		timeStamp=[[NSDate date] timeIntervalSince1970];
		timeOfLastNotification=[[NSDate date] timeIntervalSince1970];
		
		//generate a logfile name from the date and time
		NSDate *today = [NSDate date];
		NSDateFormatter *dateFormat = [[NSDateFormatter alloc] init];
		[dateFormat setDateFormat:@"ddMMYY_HHmm"];
		NSString *dateString = [dateFormat stringFromDate:today];
		global_logFileName=[NSString stringWithFormat:@"%@.txt",dateString];
		jstructure.thelogFileName=[NSString stringWithFormat:@"%@.txt",dateString];

		NSLog(@"global logfilename is:%@ js %@",global_logFileName,jstructure.thelogFileName);
		
		//write initial file lines		

		[self writeToLogFile:dateString aFileName:jstructure.thelogFileName];
		[self writeToLogFile:@"\ntime Ax Ay Az Ox Oy Oz state\n" aFileName:jstructure.thelogFileName];
		
		[self writeToLogFile:dateString aFileName:@"logfile.txt"];
		[self writeToLogFile:@"\ntime Ax Ay Az Ox Oy Oz state\n" aFileName:@"logfile.txt"];
		
		claimedUserState=@"W";
		
	}
	#ifdef DEBUGJ
	NSLog(@"input %d/%d:Ax %.2f Ay %.2f Az %.2f Ox %.2f Oy %.2f Oz %.2f",jmN_samples_taken,jstructure.mN_samples_in_driving_analysis,
		  theAccelerationVector->x,theAccelerationVector->y,theAccelerationVector->z,theOrientationVector->x,theOrientationVector->y,theOrientationVector->z);
	#endif
	
//	NSLog(@"sampling rate %d",jstructure.mSamplingRate);
	jmN_samples_taken++;
	if (jmN_samples_taken>32000) jmN_samples_taken=jstructure.mN_samples_in_driving_analysis+1;
	
	timeStamp=[[NSDate date] timeIntervalSince1970];
	jstructure.PrevioustimeStamp=[[NSDate date] timeIntervalSince1970];
	//find average linear accelerations -  an offset which 'should be' zero
	Ccalc_datastructure_avg2(0,jstructure.mN_samples_in_driving_analysis,jstructure.mN_samples_in_driving_analysis,&ax_offset);
	Ccalc_datastructure_avg2(1,jstructure.mN_samples_in_driving_analysis,jstructure.mN_samples_in_driving_analysis,&ay_offset);
	Ccalc_datastructure_avg2(2,jstructure.mN_samples_in_driving_analysis,jstructure.mN_samples_in_driving_analysis,&az_offset);
	
	theaccVectorWC_unbiased->x=theAccelerationVector->x-ax_offset;
	theaccVectorWC_unbiased->y=theAccelerationVector->y-ay_offset;
	theaccVectorWC_unbiased->z=theAccelerationVector->z-az_offset;

#ifdef DEBUGJ
	NSLog(@"Ax %.2f Axc %.2f Ay %.2f Ayc %.2f",theAccelerationVector->x,theaccVectorWC_unbiased->x,theAccelerationVector->y,theaccVectorWC_unbiased->y);
#endif
	
	jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[0]=theAccelerationVector->x;
	jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[1]=theAccelerationVector->y;
	jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[2]=theAccelerationVector->z;
	jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[3]=theaccVectorWC_unbiased->x;
	jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[4]=theaccVectorWC_unbiased->y;
	jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[5]=theaccVectorWC_unbiased->z;
	jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[6]=theOrientationVector->x;
	jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[7]=theOrientationVector->y;
	jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[8]=theOrientationVector->z;
	jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[9]=timeStamp;
	
	//make this circular buffer for faster performance
	//shift buffer back
	for( i=0;i<jstructure.mN_samples_in_driving_analysis-1;i++)
	{
		for( j=0;j<jstructure.mNfields;j++)
		{
			jstructure.mSensor_history[i].vals[j]=jstructure.mSensor_history[i+1].vals[j];
		}
	}
	
	if (jmN_samples_taken>jstructure.mN_samples_in_driving_analysis) jstructure.mEnough_driving_samples=1;
	else
	{
		jstructure.mEnough_driving_samples=0;
		NSString *resultString = [[NSString alloc]
								  initWithFormat: @"%d/200 SAMPLES",jmN_samples_taken];
		
		//	[resultString initWithFormat: @"time %d NOT ENOUGH SAMPLES",(int)timeStamp];
		_resultLabel0.text = resultString;
		NSLog(@"sample no. %d/%d",jmN_samples_taken,jstructure.mN_samples_in_driving_analysis );
	}
	
	if (jstructure.mEnough_driving_samples)
	{
		
		NSString *resultString = [[NSString alloc]
								  initWithFormat: @"t:%.1f",timeStamp];
		_resultLabel0.text = resultString;
		
		//smoothed  velocity components from acceleration and history
		//moved to average over last N samples instead of  using beta
	//	jstructure.V[0]=jstructure.V[0]*jstructure.mBeta+accVectorWC_unbiased->x/jstructure.mSamplingRate;
//		jstructure.V[1]=jstructure.V[1]*jstructure.mBeta+accVectorWC_unbiased->y/jstructure.mSamplingRate;
//		jstructure.V[2]=jstructure.V[2]*jstructure.mBeta+accVectorWC_unbiased->z/jstructure.mSamplingRate;
		
		//find velocities using integration over different numbers of samples
		[self integrateAcceleration:velocityVector_1s Nsamples:(1*jstructure.mSamplingRate)];
		[self integrateAcceleration:velocityVector_2s Nsamples:(2*jstructure.mSamplingRate)];
		[self integrateAcceleration:velocityVector_5s Nsamples:(5*jstructure.mSamplingRate)];
		[self integrateAcceleration:velocityVector_10s Nsamples:(10*jstructure.mSamplingRate)];

#ifdef DEBUGJ
		NSLog(@"velocity 2s (%.3f %.3f %.3f)",velocityVector_2s->x,velocityVector_2s->y,velocityVector_2s->z);
		NSLog(@"velocity 5s (%.3f %.3f %.3f)",velocityVector_5s->x,velocityVector_5s->y,velocityVector_5s->z);
		NSLog(@"velocity 10s (%.3f %.3f %.3f)",velocityVector_10s->x,velocityVector_10s->y,velocityVector_10s->z);
	#endif
		
		double V2h=sqrt(velocityVector_2s->x*velocityVector_2s->x+velocityVector_2s->y*velocityVector_2s->y);
		double V5h=sqrt(velocityVector_5s->x*velocityVector_5s->x+velocityVector_5s->y*velocityVector_5s->y);
		double V10h=sqrt(velocityVector_10s->x*velocityVector_10s->x+velocityVector_10s->y*velocityVector_10s->y);
		mVhCurrent=V5h;
				
		// calculate Vz (up/down velocity) avg and std for 2s of samples - 
		mVzStd=0;
		mVzAvg=0;
		jstructure.mVz[jstructure.mVzCount]=velocityVector_1s->z; //use of N samples is arbitrary - maybe find optimium?
		for (i=0;i<jstructure.mN_samples_in_2s_analysis;i++)
		{
			mVzAvg+=jstructure.mVz[i];
		}
		mVzAvg=mVzAvg/jstructure.mN_samples_in_2s_analysis;
		
		for (i=0;i<jstructure.mN_samples_in_2s_analysis;i++)
		{
			mVzStd+=(jstructure.mVz[i]-mVzAvg)*(jstructure.mVz[i]-mVzAvg);
		}
		mVzStd=sqrt(mVzStd/jstructure.mN_samples_in_2s_analysis);
		//use absolute value of mVz
		if(mVzAvg<0)mVzAvg=-mVzAvg;
		
		// calculate Oy avg and std for 20s of samples - elevation
		jstructure.mOy_20s[jstructure.mSampleCount20s++]=orientationVector->y;
		if(jstructure.mSampleCount20s>jstructure.mN_samples_in_20s_analysis-1) jstructure.mSampleCount20s=0;
		mOy_std20s=0;
		mOy_avg20s=0;
		for ( i=0;i<jstructure.mN_samples_in_20s_analysis;i++)
		{
			mOy_avg20s+=jstructure.mOy_20s[i];
		}
		mOy_avg20s=mOy_avg20s/jstructure.mN_samples_in_20s_analysis;
		for ( i=0;i<jstructure.mN_samples_in_20s_analysis;i++)
		{
			mOy_std20s+=(jstructure.mOy_20s[i]-mOy_avg20s)*(jstructure.mOy_20s[i]-mOy_avg20s);
		}
		mOy_std20s=sqrt(mOy_std20s/jstructure.mN_samples_in_20s_analysis);
		
		// calculate Oy avg and std for 2s of samples
		jstructure.mOy_2s[jstructure.mSampleCount2s]=orientationVector->y;
		mOy_std2s=0;
		mOy_avg2s=0;
		for ( i=0;i<jstructure.mN_samples_in_2s_analysis;i++)
		{
			mOy_avg2s+=jstructure.mOy_2s[i];
		}
		mOy_avg2s=mOy_avg2s/jstructure.mN_samples_in_2s_analysis;
		for (i=0;i<jstructure.mN_samples_in_2s_analysis;i++)
		{
			mOy_std2s+=(jstructure.mOy_2s[i]-mOy_avg2s)*(jstructure.mOy_2s[i]-mOy_avg2s);
		}
		mOy_std2s=sqrt(mOy_std2s/jstructure.mN_samples_in_2s_analysis);
		
		// calculate Oz avg and std for 2s of samples
		jstructure.mOz_2s[jstructure.mSampleCount2s++]=orientationVector->z;
		if(jstructure.mSampleCount2s>jstructure.mN_samples_in_2s_analysis-1) jstructure.mSampleCount2s=0;
		// only increment for last variable mOz_avg_2s
		mOz_avg2s=0;
		mOz_std2s=0;
		for ( i=0;i<jstructure.mN_samples_in_2s_analysis;i++)
		{
			mOz_avg2s+=jstructure.mOz_2s[i];
		}
		mOz_avg2s=mOz_avg2s/jstructure.mN_samples_in_2s_analysis;
		for (i=0;i<jstructure.mN_samples_in_2s_analysis;i++)
		{
			mOz_std2s+=(jstructure.mOz_2s[i]-mOz_avg2s)*(jstructure.mOz_2s[i]-mOz_avg2s);
		}
		mOz_std2s=sqrt(mOz_std2s/jstructure.mN_samples_in_2s_analysis);

		dOx_dt=[self calc_dOx_dt];
		
		//initialize string w. nothing
		popUpString = [NSString stringWithFormat:@""];
		
		//are we walking??
		if (//mAvgGabs>mGyro_Walking_Threshold &&
			//	mVzStd>jstructure.mVzStdMinWalkingThreshold &&
			//add Astd min thresholds
			mOy_std2s>jstructure.mOyStd2sMinWalkingThreshold &&
			mOz_std2s>jstructure.mOzStd2sMinWalkingThreshold &&
			mOy_std20s>jstructure.mOyStd20sMinWalkingThreshold)
		{
			mWalking=1;
			NSLog(@"jeremy thinks - walking!");
			popUpString = [NSString stringWithFormat:@"jeremy thinks - walking"];
			//playWalk();
			jstructure.mTimeofLastWalkingDetection=timeStamp;
			//get system time, this is probably not right way to do it
			jstructure.noInterveningWalkingFlag=FALSE;
		}
		else mWalking=0;
#ifdef DEBUGJ
		NSLog(@"walking criteria:mOystd2s  %d mOzstd2s %d mOzstd20s %d",mOy_std2s>jstructure.mOyStd2sMinWalkingThreshold,mOz_std2s>jstructure.mOzStd2sMinWalkingThreshold,mOy_std20s>jstructure.mOyStd20sMinWalkingThreshold);
#endif
		
		if(V2h>jstructure.mdriving_velocity_threshold_2s)
		{
			jstructure.time_of_last_mdriving_velocity_threshold_2s=timeStamp;
		}
		if(timeStamp-jstructure.time_of_last_mdriving_velocity_threshold_2s<jstructure.mDuration_in_which_to_accept_high_velocities) mdriving_velocity_threshold_2s_in_recent_history=TRUE;
		else mdriving_velocity_threshold_2s_in_recent_history=FALSE;
 
		if(V5h>jstructure.mdriving_velocity_threshold_5s)
		{
			jstructure.time_of_last_mdriving_velocity_threshold_5s=timeStamp;
		}
		if(timeStamp-jstructure.time_of_last_mdriving_velocity_threshold_5s<jstructure.mDuration_in_which_to_accept_high_velocities) mdriving_velocity_threshold_5s_in_recent_history=TRUE;
		else mdriving_velocity_threshold_5s_in_recent_history=FALSE;

		if(V10h>jstructure.mdriving_velocity_threshold_10s)
		{
			jstructure.time_of_last_mdriving_velocity_threshold_10s=timeStamp;
		}
		if(timeStamp-jstructure.time_of_last_mdriving_velocity_threshold_10s<jstructure.mDuration_in_which_to_accept_high_velocities) mdriving_velocity_threshold_10s_in_recent_history=TRUE;
		else mdriving_velocity_threshold_10s_in_recent_history=FALSE;

		
		//are we driving??
		if (!mWalking&&
			mdriving_velocity_threshold_2s_in_recent_history &&
			mdriving_velocity_threshold_5s_in_recent_history &&
			mdriving_velocity_threshold_10s_in_recent_history &&
			(timeStamp-jstructure.mTimeofLastWalkingDetection)>jstructure.mtime_after_walking_to_filter_driving &&
			mVzStd<jstructure.mVzStdMaxDrivingThreshold &&
			mVzAvg<jstructure.mVzAvgMaxDrivingThreshold &&
			mOy_std20s<jstructure.mOyStd20sMaxDrivingThreshold &&
			mOy_std2s<jstructure.mOyStd2sMaxDrivingThreshold &&
			mOy_std2s>jstructure.mOyStd2sMinDrivingThreshold &&
			mOz_std2s>jstructure.mOzStd2sMinDrivingThreshold &&
			mOz_std2s<jstructure.mOzStd2sMaxDrivingThreshold)
		{
			mDriving=1;
			jstructure.noInterveningWalkingFlag=TRUE;
			jstructure.mTimeofLastDrivingDetection=timeStamp;
			NSLog(@"jeremy thinks - driving!");
			//	popUpString=sprintf("jeremy thinks - driving!");
			popUpString = [NSString stringWithFormat:@"jeremy thinks - driving"];
			//playHonk();
		}
		else mDriving=0;
#ifdef DEBUGJ

		NSLog(@"driving criteria:mVh %d time %d mVzstd %d mVzavg %d mOystd20 %d mOystd2< %d mOystd2> %d mOzstd2> %d mOzstd2 < %d",mVhCurrent>jstructure.mdriving_velocity_threshold_10s,(timeStamp-jstructure.mTimeofLastWalkingDetection)>jstructure.mtime_after_walking_to_filter_driving,mVzStd<jstructure.mVzStdMaxDrivingThreshold,mVzAvg<jstructure.mVzAvgMaxDrivingThreshold,mOy_std20s<jstructure.mOyStd20sMaxDrivingThreshold,mOy_std2s<jstructure.mOyStd2sMaxDrivingThreshold ,mOy_std2s>jstructure.mOyStd2sMinDrivingThreshold ,mOz_std2s>jstructure.mOzStd2sMinDrivingThreshold,mOz_std2s<jstructure.mOzStd2sMaxDrivingThreshold    );
		
#endif

		
		//'nominal driving state' -  if less than some threshold since last driving detection, and no walking detected meantime, then we are still driving (or sitting in car...)
		if (timeStamp-jstructure.mTimeofLastDrivingDetection<jstructure.mExpectationTimeBetweenDrivingDetections && jstructure.noInterveningWalkingFlag==TRUE)
		{
			//dOx_dt
			//crazy driver thresholds
			//		double crazy_driver_velocity_threshold;
			//		double crazy_driver_dOx_dt_threshold;
			//		double crazy_driver_Ah_threshold;
			#ifdef DEBUGJ
			NSLog(@"nominal state is driving");
			#endif
			
			NSString *crazydriverfile	=[[NSString alloc] initWithFormat: @"crazydriverfile.txt"];
			NSString *crazylogString = [[NSString alloc] initWithFormat: @""];
										
			//this first one should ultimately come from actual speed as detected from gps
			if(V5h>jstructure.crazy_driver_velocity_threshold)
			{
				crazylogString = [[NSString alloc] initWithFormat: @"time %.2f excessive horizontal velocity %.2f",timeStamp,V5h];
				[self writeToLogFile:crazylogString aFileName:crazydriverfile];
				NSLog(@"%@",crazylogString);
			}
			if(dOx_dt>jstructure.crazy_driver_dOx_dt_threshold)
			{
				crazylogString = [[NSString alloc] initWithFormat: @"time %.2f excessive turning rate %.2f",timeStamp,dOx_dt];
				[self writeToLogFile:crazylogString aFileName:crazydriverfile];
				NSLog(@"%@",crazylogString);

			}
			if(V5h>jstructure.crazy_driver_velocity_threshold)
			{
				crazylogString = [[NSString alloc] initWithFormat: @"time %.2f excessive horizontal acceleration %.2f",timeStamp,V5h];
				[self writeToLogFile:crazylogString aFileName:crazydriverfile];
			}
		}
				
		if( mWalking  || mDriving)
		{
			NSLog(@"playsound");

			//play system sound
			SystemSoundID soundID;
			NSString *soundPath= [[NSBundle mainBundle] pathForResource:@"zinger1" ofType:@"mp3"];
			if(mWalking) {
				soundPath = [[NSBundle mainBundle] pathForResource:@"zinger2" ofType:@"mp3"];
			}
			else if(mDriving)
			{
				soundPath = [[NSBundle mainBundle] pathForResource:@"zinger1" ofType:@"mp3"];
			}
			NSURL *soundUrl = [NSURL fileURLWithPath:soundPath];
			AudioServicesCreateSystemSoundID ((CFURLRef)soundUrl, &soundID);
			AudioServicesPlaySystemSound(soundID);
		}
		
		
		
		//WRITE TO LOG FILE
		NSString *logString = [[NSString alloc]
							   initWithFormat: @"%.2f %.2f %.2f %.2f %.2f %.2f %.2f %@",timeStamp,theaccVectorWC_unbiased->x,theaccVectorWC_unbiased->y,theaccVectorWC_unbiased->z,theOrientationVector->x,theOrientationVector->y,theOrientationVector->z,claimedUserState];
		NSLog(@"%@",logString);
//		global_logFileName=jstructure.thelogFileName;
//		NSLog(@"glogfilename:%@",global_logFileName);
//		NSLog(@"slogfilename:%@",jstructure.thelogFileName);
		global_logFileName	=[[NSString alloc] initWithFormat: @"logfile.txt"];
#ifdef DEBUGJ
		NSLog(@"string %@ to be written to %@",logString,global_logFileName);
#endif		
	[self writeToLogFile:logString aFileName:global_logFileName];
//		NSLog(@"After writing");
		//	[self displayContent:@"JeremyLog.txt"];
		
		//		[self writeToTextFile];
		//		[self displayContent:@"textfile.txt"];
		
		//[Viewcontroller writestringtofile]
		
		
		
		//print WC accelerations
		NSString *resultString1 = [[NSString alloc]
								   initWithFormat: @"Ax %.2f", theAccelerationVector->x];
		_resultLabel1.text = resultString1;
		
		//	[CategoryLbl setTextColor:[UIColor colorWithRed:(38/255.f) green:(171/255.f) blue:(226/255.f) alpha:1.0f]];
		
		NSString *resultString2 = [[NSString alloc]
								   initWithFormat: @"Ay %.2f", theAccelerationVector->y];
		_resultLabel2.text = resultString2;
		NSString *resultString3 = [[NSString alloc]
								   initWithFormat: @"Az %.2f", theAccelerationVector->z];
		_resultLabel3.text = resultString3;
		
		//print unbiased accelerations
		//print orientations
		NSString *resultString4 = [[NSString alloc]
								   initWithFormat: @"Ax %.2f", theaccVectorWC_unbiased->x];
		_resultLabel4.text = resultString4;
		NSString *resultString5 = [[NSString alloc]
								   initWithFormat: @"Ay %.2f",theaccVectorWC_unbiased->y];
		_resultLabel5.text = resultString5;
		NSString *resultString6 = [[NSString alloc]
								   initWithFormat: @"Az %.2f",theaccVectorWC_unbiased->z];
		_resultLabel6.text = resultString6;
		
		//print orientations
		NSString *resultString7 = [[NSString alloc]
								   initWithFormat: @"Ox %.1f", theOrientationVector->x];
		_resultLabel7.text = resultString7;
		NSString *resultString8 = [[NSString alloc]
								   initWithFormat: @"Oy %.1f",theOrientationVector->y];
		_resultLabel8.text = resultString8;
		NSString *resultString9 = [[NSString alloc]
								   initWithFormat: @"Oz %.1f",theOrientationVector->z];
		_resultLabel9.text = resultString9;
		
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
		if(mVhCurrent>jstructure.mdriving_velocity_threshold)[_resultLabel13 setTextColor:[UIColor greenColor]];
		else [_resultLabel13 setTextColor:[UIColor redColor]];
*/
		
		NSString *resultString14 = [[NSString alloc]
									initWithFormat: @"VzAvg %.1f",mVzAvg];
		_resultLabel14.text = resultString14;
		if(mVzAvg<jstructure.mVzAvgMaxDrivingThreshold)[_resultLabel14 setTextColor:[UIColor greenColor]];
		else [_resultLabel14 setTextColor:[UIColor redColor]];
		
		NSString *resultString15 = [[NSString alloc]
									initWithFormat: @"Vzstd %.1f",mVzStd];
		_resultLabel15.text = resultString15;
		if(mVzStd>jstructure.mVzStdMinWalkingThreshold)[_resultLabel15 setTextColor:[UIColor redColor]];
		else if(mVzStd<jstructure.mVzStdMaxDrivingThreshold)[_resultLabel15 setTextColor:[UIColor greenColor]];
		else [_resultLabel15 setTextColor:[UIColor blackColor]];
		
		NSString *resultString16 = [[NSString alloc]
									initWithFormat: @"Oystd2 %.1f",mOy_std2s];
		_resultLabel16.text = resultString16;
		if(mOy_std2s>jstructure.mOyStd2sMinWalkingThreshold || mOy_std2s<jstructure.mOyStd2sMinDrivingThreshold)
			[_resultLabel16 setTextColor:[UIColor redColor]];
		else if(mOy_std2s<jstructure.mOyStd2sMaxDrivingThreshold) [_resultLabel16 setTextColor:[UIColor greenColor]];
		else [_resultLabel16 setTextColor:[UIColor blackColor]];
		
		NSString *resultString17 = [[NSString alloc]
									initWithFormat: @"Oystd20 %.1f",mOy_std20s];
		_resultLabel17.text = resultString17;
		if(mOy_std20s>jstructure.mOyStd20sMinWalkingThreshold)[_resultLabel17 setTextColor:[UIColor redColor]];
		else if(mOy_std20s<jstructure.mOyStd20sMaxDrivingThreshold)[_resultLabel17 setTextColor:[UIColor greenColor]];
		else [_resultLabel17 setTextColor:[UIColor blackColor]];
		
		if(mWalking)
		{
			NSString *resultString18 = [[NSString alloc]
										initWithFormat: @"WALKING"];
			_resultLabel18.text = resultString18;
		setTextColor:[UIColor greenColor];
			timeOfLastNotification=[[NSDate date] timeIntervalSince1970];
		}
		else if(mDriving)
		{
			NSString *resultString18 = [[NSString alloc]
										initWithFormat: @"DRIVING"];
			_resultLabel18.text = resultString18;
			[_resultLabel18 setTextColor:[UIColor greenColor]];
			timeOfLastNotification=[[NSDate date] timeIntervalSince1970];
		}
		else
		{
			if([[NSDate date] timeIntervalSince1970]-timeOfLastNotification > 5)
			{
				NSString *resultString18 = [[NSString alloc] initWithFormat: @""];
				_resultLabel18.text = resultString18;
			}
			
		}
		
		NSString *resultString19 = [[NSString alloc] initWithFormat: @"Ozstd2 %.1f",mOz_std2s];
		_resultLabel19.text = resultString19;
		if(mOz_std2s>jstructure.mOzStd2sMinWalkingThreshold || mOz_std2s<jstructure.mOzStd2sMinDrivingThreshold)
			[_resultLabel19 setTextColor:[UIColor redColor]];
		else if(mOz_std2s<jstructure.mOzStd2sMaxDrivingThreshold)[_resultLabel19 setTextColor:[UIColor greenColor]];
		else [_resultLabel19 setTextColor:[UIColor blackColor]];
		
		NSString *resultString20 = [[NSString alloc] initWithFormat: @""];
		_resultLabel20.text = resultString20;
		
		NSString *resultString21 = [[NSString alloc] initWithFormat: @"V2x %.2f",velocityVector_2s->x];
		_resultLabelV2x.text = resultString21;
		
		resultString21 = [[NSString alloc] initWithFormat: @"V2y %.3f",velocityVector_2s->y];
		_resultLabelV2y.text = resultString21;
		
		resultString21 = [[NSString alloc] initWithFormat: @"V2z %.3f",velocityVector_2s->z];
		_resultLabelV2z.text = resultString21;
		
		resultString21 = [[NSString alloc] initWithFormat: @"V5x %.3f",velocityVector_5s->x];
		_resultLabelV5x.text = resultString21;
		
		resultString21 = [[NSString alloc] initWithFormat: @"V5y %.3f",velocityVector_5s->y];
		_resultLabelV5y.text = resultString21;
		
		resultString21 = [[NSString alloc] initWithFormat: @"V5z %.3f",velocityVector_5s->z];
		_resultLabelV5z.text = resultString21;
		
		resultString21 = [[NSString alloc] initWithFormat: @"V10x %.3f",velocityVector_10s->x];
		_resultLabelV10x.text = resultString21;
		
		resultString21 = [[NSString alloc] initWithFormat: @"V10y %.3f",velocityVector_10s->y];
		_resultLabelV10y.text = resultString21;
		
		resultString21 = [[NSString alloc] initWithFormat: @"V10z %.3f",velocityVector_10s->z];
		_resultLabelV10z.text = resultString21;
		
		resultString21 = [[NSString alloc] initWithFormat: @"V2h %.3f",V2h];
		_resultLabelV2h.text = resultString21;
		
		resultString21 = [[NSString alloc] initWithFormat: @"V5h %.3f",V5h];
		if(V5h>jstructure.mdriving_velocity_threshold_5s )[_resultLabelV5h setTextColor:[UIColor greenColor]];
		else [_resultLabelV5h setTextColor:[UIColor redColor]];		
		_resultLabelV5h.text = resultString21;
		
		resultString21 = [[NSString alloc] initWithFormat: @"V10h %.3f",V10h];
		_resultLabelV10h.text = resultString21;
		if(V10h>jstructure.mdriving_velocity_threshold_10s )[_resultLabelV10h setTextColor:[UIColor greenColor]];
		else [_resultLabelV10h setTextColor:[UIColor redColor]];
		
		

		
//dOx/dt
		resultString21 = [[NSString alloc] initWithFormat: @"dOx/dt %3.2f",dOx_dt];
		_resultLabeldOx_dt.text = resultString21;
		
		//JEREMY'S CODE ABOVE UNTIL HERE
		
		totalSamples++;
		
		if ([UIDevice currentDevice].batteryLevel != batteryLevel)
		{
			batteryLevel = [UIDevice currentDevice].batteryLevel;
			NSLog(@"battery level: %f  acc samples:%d",batteryLevel,totalSamples);
		}
	};
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


- (void)locationManager:(CLLocationManager *)manager didUpdateToLocation:(CLLocation *)newLocation fromLocation:(CLLocation *)oldLocation {
	//NSLog(@"new:%@",[newLocation description]);
	
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



-(double) calc_dOx_dt {
	//calculate dOx/dt using average over 1s of samples , at points 1s apart
	double average_dOx_dt=0;
	double dOx=0;
	double dOx_dt=0;

	int i;
	int k=jstructure.mN_samples_in_driving_analysis-1-jstructure.mN_samples_in_1s_analysis;
	for ( i=jstructure.mN_samples_in_driving_analysis-1;i>k;i--)
	{
	//	NSLog(@"calcdoX/dt:i=%d ",i);
		dOx+=jstructure.mSensor_history[i].vals[6]-jstructure.mSensor_history[i-jstructure.mN_samples_in_1s_analysis].vals[6];
		dOx_dt=dOx/(jstructure.mSensor_history[i].vals[9]-jstructure.mSensor_history[i-jstructure.mN_samples_in_1s_analysis].vals[9]);  //vals[9] is time
		average_dOx_dt+=dOx_dt;
//		NSLog(@"dOx/dt avg %f dOx %.2f cur %.2f ",average_dOx_dt,dOx,dOx_dt);

	}
	average_dOx_dt=average_dOx_dt/jstructure.mN_samples_in_1s_analysis;
#ifdef DEBUGJ
	NSLog(@"calcdoX/dt avg %f ",average_dOx_dt);
	#endif
	
	return average_dOx_dt;
}


void Cintegrate(double A[],double V[],double beta) {
	//float [] Vp=new float[3];
	int i;
	for ( i=0;i<3;i++)
	{
		//			LOGD("in integrate bef: V[%d] %.3f A[%d] %.3f",i,V[i],i,A[i]);
		V[i]=V[i]*beta+(*(A+i))/jstructure.mSamplingRate;
		//			LOGD("in integrate aft: V[%d] %.3f A[%d] %.3f",i,V[i],i,A[i]);
	}
	//		LOGD("V[%d] %.3f A[%d] ",Ax-ax_offset,Ay-ax_offset,Az-ax_offset);
	
	//		V[0]=-666.0; //test for call by ref or value
	//		return V;
}

void Cintegratepoint(double* A,double* V,double beta) {
	//float [] Vp=new float[3];
	int i;
	for ( i=0;i<3;i++)
	{
		//		LOGD("in integrate bef: V[%d] %.3f A[%d] %.3f",i,*(V+i),i,*(A+i));
		*(V+i)=(*(V+i))*beta+(*(A+i))/jstructure.mSamplingRate;
		//			LOGD("in integrate aft: V[%d] %.3f A[%d] %.3f",i,*(V+i),i,*(A+i));
	}
	
}


double  Ccalc_datastructure_avg2( int field,int stdlength,int totlength, double *answer ){
	double mean=0.0;
	//		int stdlength=10;
	//		int totlength=10;
	int i=0;
	if (stdlength>totlength) {
		mean=-1.0;return mean;
	}
	for ( i=totlength-stdlength;i<totlength;i++)
	{
		mean+=jstructure.mSensor_history[i].vals[field];
	}
	mean=mean/stdlength;
	*answer=mean;
	return (double)mean;
}

// integrate , V=Adt . vals[3-5] are accels with bias removed (all 0 for phone at rest)
// make this better by using larger spacing
-(AlgoVector *)integrateAcceleration:(AlgoVector *)aVector Nsamples:(int)N_samples
{
	int i;
	double dt;
	aVector->x=0;
	aVector->y=0;
	aVector->z=0;
	for (i=0;i<N_samples;i++)
	{
		
		dt=jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-i-1].vals[9]-
		jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-i-2].vals[9];
		
		aVector->x+=jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-i-1].vals[3]*dt;
		aVector->y+=jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-i-1].vals[4]*dt;
		aVector->z+=jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-i-1].vals[5]*dt;
	}
	//this is wrong - integration does not divide by  N
	aVector->x/=N_samples;
	aVector->y/=N_samples;
	aVector->z/=N_samples;
	#ifdef DEBUGJ
	NSLog(@"integrated V:(%.2f %.2f %.2f): %d samples",aVector->x,aVector->y,aVector->z,N_samples);
	#endif
	return(aVector);
}





-(void) writeToLogFile:(NSString*)content aFileName:(NSString *)filename{
	
	
	content = [NSString stringWithFormat:@"%@\n",content];
	
	//get the documents directory:
	NSString *documentsDirectory = [NSHomeDirectory() stringByAppendingPathComponent:@"Documents"];
	NSString *fileName = [NSString stringWithFormat:@"%@/%@", documentsDirectory,filename];

#ifdef DEBUGJ
	NSLog(@"directory:%@ file %@",documentsDirectory,filename);

	//NSLog(@"writing file %@",filename);
#endif

	
	NSFileHandle *fileHandle = [NSFileHandle fileHandleForWritingAtPath:fileName];
	if (fileHandle){
		[fileHandle seekToEndOfFile];
		[fileHandle writeData:[content dataUsingEncoding:NSUTF8StringEncoding]];
		[fileHandle closeFile];
		
	}
	else{
		[content writeToFile:fileName
				  atomically:NO
					encoding:NSStringEncodingConversionAllowLossy
					   error:nil];
	}
}


//Method retrieves content from documents directory and
//displays it in an alert
-(void) displayContent:(NSString*)fName {
	//get the documents directory:
	NSArray *paths = NSSearchPathForDirectoriesInDomains
	(NSDocumentDirectory, NSUserDomainMask, YES);
	NSString *documentsDirectory = [paths objectAtIndex:0];
	
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
	[content release];
	
}


- (void) displayInfo: (id) sender {
	//    double farenheit = [_tempText.text doubleValue];
	double testvalue = 666.6;
	
	NSString *resultString = [[NSString alloc]
							  initWithFormat: @"Celsius %f", testvalue];
	_resultLabel1.text = resultString;
}

- (void) filewrite
{
	//	NSString *filePath = [NSHomeDirectory() stringByAppendingPathComponent:@"Documents/dt"];
	
	//	NSString *WriteThisString = [[NSString alloc] initWithFormat: @"this is a test string"];
	
	//	[WriteThisString writeToFile:@"/file.txt" atomically:YES useAuxiliaryFile encoding:(NSStringEncoding)enc error:(NSError **)error];
	
	//	writeToFile:(NSString *)path atomically:(BOOL)useAuxiliaryFile encoding:(NSStringEncoding)enc error:(NSError **)error
	
	
	//[@"test string" writeToFile:filePath];
}



@end


/*
 Trivial wrapper around system sound as provided
 by Audio Services. Don’t forget to add the Audio
 Toolbox framework.
 */

//#import "Sound.h"
#import <AudioToolbox/AudioToolbox.h>

@implementation Sound {
	SystemSoundID handle;
}

- (id) initWithPath: (NSString*) path
{
	self = [super init];
	NSString *const resourceDir = [[NSBundle mainBundle] resourcePath];
	NSString *const fullPath = [resourceDir stringByAppendingPathComponent:path];
	NSURL *const url = [NSURL fileURLWithPath:fullPath];
	OSStatus errcode = AudioServicesCreateSystemSoundID((CFURLRef) url, &handle);
	NSAssert1(errcode == 0, @"Failed to load sound: %@", path);
	return self;
}

- (void) dealloc
{
	AudioServicesDisposeSystemSoundID(handle);
}

- (void) play
{
	AudioServicesPlaySystemSound(handle);
}

@end

