//
//  ViewController.m
//  testAlgo
//
//  Created by Itai David on 8/14/12.
//  Copyright (c) 2012 itaidddd@gmail.com. All rights reserved.
//

#import <CoreMotion/CoreMotion.h>
#import "ViewController.h"
//#import "AddTwoTextFieldViewController.h"
//#import "AddTwoTextFieldAppDelegate.h"

//Jeremy



typedef struct tag1 {
	int time;
	double vals[9];  //A_wc, A_wc_unbiased, Orientation
    /*	double Ay;
     double Az;
     double Ox;
     double Oy;
     double Oz;
     */} Cdatastructure;

typedef struct tag2
{
	int mNfields;
	int mN_samples_in_driving_analysis;
	int mN_samples_in_2s_analysis;
	int mN_samples_in_20s_analysis;
	double mVzStdMinWalkingThreshold;
	double mOyStd2sMinWalkingThreshold;
	double mOyStd20sMinWalkingThreshold;
	double mdriving_velocity_threshold;
	double mVzStdMaxDrivingThreshold;
	double mVzAvgMaxDrivingThreshold;
	double mOyStd20sMaxDrivingThreshold;
	double mOyStd2sMaxDrivingThreshold;
	int mN_samples_taken_initial_value;
	int mN_samples_taken_current_value;
	int mN_samples_taken;
	int mVzCount;
	Cdatastructure mSensor_history[200];  //mN_samples_in_driving_analysis - itay didnt want #defines and I dont want to have to malloc
	int mEnough_driving_samples;
	double V2[3];
	double V[3];
	double mVz[200];  //mN_samples_in_driving_analysis =
	int mSampleCount20s;
	int mSampleCount2s;
	double mOy_20s[200]; //mN_samples_in_driving_analysis
	double mOy_2s[20]; //mN_samples_in_2s_analysis
	int mWalking;
	int mDriving;
	int mTimeofLastWalkingDetection;
	int mtime_after_walking_to_filter_driving;
	int mSamplingRate;
	double mBeta;
	double Vhmax;
	double Vh2max;
    
} jeremystruct;

static jeremystruct jstructure;
static int jmN_samples_taken=0;

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
- (void)viewDidLoad
{
	
    [super viewDidLoad];
	// Do any additional setup after loading the view, typically from a nib.
    algoDetection = [[AlgoDetection alloc] init];
    accVector = [[AlgoVector alloc] init];
	
    //jeremy added 2 lines below
    orientationVector=[[AlgoVector alloc] init];
    accVectorWC= [[AlgoVector alloc] init];
    accVectorWC_unbiased= [[AlgoVector alloc] init];
	//jeremy
	
    algoLocVector = [[AlgoLocVector alloc] init];
    locationManager = [[CLLocationManager alloc] init];
	
	//jeremy
    __block double ax_offset,ay_offset,az_offset; //these are the long-timescale biases of the acceleration vector (assumed to be 'really' //0)
	__block int i;
	__block int j;
	__block time_t timeStamp;
	//__block double mlinaccvector[3];
	//	__block double V[3];
	//	__block double V2[3];
	//double A[3];double R[9];double O[3];
	//double R11,R12,R13,R21,R22,R23,R31,R32,R33;  //rotation matrix elements
    //	double Ax,Ay,Az;
    //	double Ox,Oy,Oz;
	//__block double Awc[3],Awc_corrected[3];
	//__block double mVhCurrent,
	__block double mVh2Current;
	__block double mVzStd=0;
	__block double mVzAvg=0;
	__block double mOy_std20s;
	__block double mOy_avg20s;
	__block double mOy_std2s;
	__block double mOy_avg2s;
	__block int mWalking=0;
	__block int mDriving=0;
	//jeremy
	
    locationManager.delegate = self;
    locationManager.desiredAccuracy = kCLLocationAccuracyThreeKilometers;//kCLLocationAccuracyKilometer;
    locationManager.distanceFilter = 10000;
    //[locationManager setPurpose:@"My Custom Purpose Message..."];
    //[locationManager startUpdatingLocation];
    //[locationManager startMonitoringSignificantLocationChanges];
    Gps_state = NO;
    AGps_state = NO;
    
    [[UIDevice currentDevice] setBatteryMonitoringEnabled:YES];
	
    NSLog(@"starting - didload");
	
	NSString *resultString = [[NSString alloc]
							  initWithFormat: @"STARTING"];
	_resultLabel0.text = resultString;
	

	
    motionManager = [[CMMotionManager alloc] init];
    [motionManager setAccelerometerUpdateInterval:1.0/10.0];
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
	/*						NSLog(@"------------rot matrix--------------");
							NSLog(@"%lf %lf %lf",motionManager.deviceMotion.attitude.rotationMatrix.m11,
								  motionManager.deviceMotion.attitude.rotationMatrix.m12,
								  motionManager.deviceMotion.attitude.rotationMatrix.m13
								  );
							NSLog(@"%lf %lf %lf",motionManager.deviceMotion.attitude.rotationMatrix.m21,
								  motionManager.deviceMotion.attitude.rotationMatrix.m22,
								  motionManager.deviceMotion.attitude.rotationMatrix.m23
								  );
							NSLog(@"%lf %lf %lf",motionManager.deviceMotion.attitude.rotationMatrix.m31,
								  motionManager.deviceMotion.attitude.rotationMatrix.m32,
								  motionManager.deviceMotion.attitude.rotationMatrix.m33
								  );
							//NSLog(@"pc:Ax %lf Ay %lf Az %lf",data.acceleration.x,data.acceleration.y,data.acceleration.z);
							accVector->x = data.acceleration.x*rot.m11 + data.acceleration.y*rot.m12 + data.acceleration.z*rot.m13;
							accVector->y = data.acceleration.x*rot.m21 + data.acceleration.y*rot.m22 + data.acceleration.z*rot.m23;
							accVector->z = data.acceleration.x*rot.m31 + data.acceleration.y*rot.m32 + data.acceleration.z*rot.m33;
							
							
							accVector->x *= 9.81f;
							accVector->y *= 9.81f;
							accVector->z *= 9.81f;
							
							NSLog(@"wc1:Ax %lf Ay %lf Az %lf",accVector->x,accVector->y,accVector->z);
							
				*/
							
							
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

							
							isDriving=0;
							
							if(jmN_samples_taken==0) {
								jstructure.mNfields=9; //fields in dataq record - Ax Ay Az Ax_no(no offset) Ay_no Az_no Ox Oy Oz
								jstructure.mN_samples_in_driving_analysis=200;
								jstructure.mN_samples_in_2s_analysis=20;
								jstructure.mN_samples_in_20s_analysis=200;
								jstructure.mVzStdMinWalkingThreshold=0.08;
								jstructure.mOyStd2sMinWalkingThreshold=10.0;
								jstructure.mOyStd20sMinWalkingThreshold=10.0;
								jstructure.mdriving_velocity_threshold=0.5;
								jstructure.mVzStdMaxDrivingThreshold=0.2;
								jstructure.mVzAvgMaxDrivingThreshold=0.7;
								jstructure.mOyStd20sMaxDrivingThreshold=5.0;
								jstructure.mOyStd2sMaxDrivingThreshold=5.0;
								jstructure.mEnough_driving_samples=0;  //0 false, 1 true
								jstructure.mSamplingRate=10; //in samples/second
								jstructure.mBeta=0.95; //in samples/second
								jstructure.V[0]=0.0;jstructure.V[1]=0.0;jstructure.V[2]=0.0;
								jstructure.V2[0]=0.0;jstructure.V2[1]=0.0;jstructure.V2[2]=0.0;
								jstructure.Vhmax=0.0;jstructure.Vh2max=0.0;
								jstructure.mVzCount=0;
								jstructure.mtime_after_walking_to_filter_driving=5;
							}
							jmN_samples_taken++;
							if (jmN_samples_taken>32000) jmN_samples_taken=jstructure.mN_samples_in_driving_analysis+1;
							
							//	time(&timeStamp);
							//	([[NSDate date] timeIntervalSince1970]*1000);//data.timestamp*1000.0;
							NSLog(@"time %.1f",data.timestamp);
							timeStamp=data.timestamp;

							//find average linear accelerations - this is an offset which 'should be' zero
							//  ax_offset=
							Ccalc_datastructure_avg2(0,jstructure.mN_samples_in_driving_analysis,jstructure.mN_samples_in_driving_analysis,&ax_offset);
							Ccalc_datastructure_avg2(1,jstructure.mN_samples_in_driving_analysis,jstructure.mN_samples_in_driving_analysis,&ay_offset);
							Ccalc_datastructure_avg2(2,jstructure.mN_samples_in_driving_analysis,jstructure.mN_samples_in_driving_analysis,&az_offset);
							//	LOGD("xoff %.3f yoff %.3f zoff %.3f",ax_offset,ay_offset,az_offset);
							
							//		System.out.println("ax1 "+ax3);
							accVectorWC_unbiased->x=accVectorWC->x-ax_offset;
							accVectorWC_unbiased->y=accVectorWC->y-ay_offset;
							accVectorWC_unbiased->z=accVectorWC->z-az_offset;
							
							//	sensor_data_vector[3]=
							
							//                                                   LOGD("c:after correction: awcx1 %.3f awcy %.3f awcz %.3f ",Awc[0]-ax_offset,Awc[1]-ax_offset,Awc[2]-ax_offset);
							
							jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[0]=accVectorWC->x;
							jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[1]=accVectorWC->y;
							jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[2]=accVectorWC->z;
							jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[3]=accVectorWC_unbiased->x;
							jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[4]=accVectorWC_unbiased->y;
							jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[5]=accVectorWC_unbiased->z;
							jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[6]=orientationVector->x;
							jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[7]=orientationVector->y;
							jstructure.mSensor_history[jstructure.mN_samples_in_driving_analysis-1].vals[8]=orientationVector->z;

							
							//	LOGD("Ax %f",Ax);
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
								NSString *resultString4 = [[NSString alloc]
														   initWithFormat: @"time %d NOT ENOUGH SAMPLES",(int)timeStamp];
								_resultLabel0.text = resultString4;
							}

							if (jstructure.mEnough_driving_samples)  {
								NSString *resultString5 = [[NSString alloc]
														   initWithFormat: @"time %d",(int)timeStamp];

								_resultLabel0.text = resultString5;
					
								// calculate rotation matrix from orientation angles
								//actually dont need to since iphone gets real world acc. data wout gravity
								
								//calculate velocity by iand printout - velocity from unbiased accels.
								
								//                      LOGD("V2xbef %.3f V2ybef %.3f V2zbef %.3f",jstructure.V2[0],jstructure.V2[1],jstructure.V2[2]);
								//NSLog(@"V2xbef %.3f V2y %.3f V2z %.3f",jstructure.V2[0],jstructure.V2[1],jstructure.V2[2]);
								
								jstructure.V2[0]=jstructure.V2[0]*jstructure.mBeta+accVectorWC_unbiased->x/jstructure.mSamplingRate;
								jstructure.V2[1]=jstructure.V2[1]*jstructure.mBeta+accVectorWC_unbiased->y/jstructure.mSamplingRate;
								jstructure.V2[2]=jstructure.V2[2]*jstructure.mBeta+accVectorWC_unbiased->z/jstructure.mSamplingRate;
								
								NSLog(@"Vx %.3f Vy %.3f Vz %.3f",jstructure.V2[0],jstructure.V2[1],jstructure.V2[2]);
								//	LOGD("V2xaft %.3f V2yaft %.3f V2zaft %.3f",jstructure.V2[0],jstructure.V2[1],jstructure.V2[2]);
								
								
								//	mVhCurrent=sqrt(V[0]*V[0]+V[1]*V[1]);
								mVh2Current=sqrt(jstructure.V2[0]*jstructure.V2[0]+jstructure.V2[1]*jstructure.V2[1]);
								//	if(mVhCurrent>jstructure.Vhmax) jstructure.Vhmax=mVhCurrent;
								//	if(mVh2Current>jstructure.Vh2max) jstructure.Vh2max=mVh2Current;
								
								// calculate Vz avg and std for 2s of samples
								//	mVz1=mVz1*alpha1+V2[2]*(1-alpha1);
								mVzStd=0;
								mVzAvg=0;
								jstructure.mVz[jstructure.mVzCount++]=jstructure.V2[2];
								if (jstructure.mVzCount>jstructure.mN_samples_in_2s_analysis-1) jstructure.mVzCount=0;
								
								//calculate Vz avg and std for 2s of samples
								for (i=0;i<jstructure.mN_samples_in_2s_analysis;i++)
								{
									mVzAvg+=jstructure.mVz[i];
								}
								mVzAvg=mVzAvg/jstructure.mN_samples_in_2s_analysis;
								//use absolute value of mVz
								if(mVzAvg<0)mVzAvg=-mVzAvg;
								
								for ( i=0;i<jstructure.mN_samples_in_2s_analysis;i++)
								{
									mVzStd+=(jstructure.mVz[i]-mVzAvg)*(jstructure.mVz[i]-mVzAvg);
								}
								mVzStd=sqrt(mVzStd/jstructure.mN_samples_in_2s_analysis);
								
								// calculate Oy avg and std for 20s of samples
								jstructure.mSampleCount20s++; if(jstructure.mSampleCount20s>jstructure.mN_samples_in_20s_analysis-1) jstructure.mSampleCount20s=0;
								jstructure.mOy_20s[jstructure.mSampleCount20s]=orientationVector->y;
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
								jstructure.mSampleCount2s++; if(jstructure.mSampleCount2s>jstructure.mN_samples_in_2s_analysis-1) jstructure.mSampleCount2s=0;
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
								
								//copy(popUpString,"jeremy thinks - nothing!");
								popUpString = [NSString stringWithFormat:@"jeremy thinks - nothing"];
								
								//are we walking??
								if (//mAvgGabs>mGyro_Walking_Threshold &&
									mVzStd>jstructure.mVzStdMinWalkingThreshold &&
									mOy_std2s>jstructure.mOyStd2sMinWalkingThreshold &&
									mOy_std20s>jstructure.mOyStd20sMinWalkingThreshold)
								{
									mWalking=1;
									NSLog(@"jeremy thinks - walking!");
									popUpString = [NSString stringWithFormat:@"jeremy thinks - walking"];
									//playWalk();
									jstructure.mTimeofLastWalkingDetection=timeStamp;
									//get system time, this is probably not right way to do it
								}
								else mWalking=0;
								
								//are we driving??
								if (!mWalking&&
									mVh2Current>jstructure.mdriving_velocity_threshold &&
									(timeStamp-jstructure.mTimeofLastWalkingDetection)>jstructure.mtime_after_walking_to_filter_driving*1000 &&
									mVzStd<jstructure.mVzStdMaxDrivingThreshold &&
									mVzAvg<jstructure.mVzAvgMaxDrivingThreshold &&
									mOy_std20s<jstructure.mOyStd20sMaxDrivingThreshold &&
									mOy_std2s<jstructure.mOyStd2sMaxDrivingThreshold)
								{
									mDriving=1;
									NSLog(@"jeremy thinks - driving!");
									//	popUpString=sprintf("jeremy thinks - driving!");
									popUpString = [NSString stringWithFormat:@"jeremy thinks - driving"];
									//playHonk();
								}
								else mDriving=0;
							/*
				//			//	[ViewController writeStringToFile:@"test"];
							//	[this ]
							 */
								NSLog(@"%@",popUpString);
								if(/*mWalking ||*/ mDriving)
								{
								UILocalNotification *localNotification = [[UILocalNotification alloc] init];
								// create a calendar to form date
								NSDate *item = [NSDate dateWithTimeIntervalSinceNow:0];
								
								localNotification.fireDate = item;
								localNotification.timeZone = [NSTimeZone defaultTimeZone];
								localNotification.alertBody = popUpString;
								localNotification.soundName = UILocalNotificationDefaultSoundName;
								if(mWalking)localNotification.soundName = @"beepbeep.wav";
								else if(mDriving)localNotification.soundName = @"sound.caf";
							//	else localNotification.soundName=null;
								//localNotification.alertAction = @"Show me";
								localNotification.applicationIconBadgeNumber = 0;
								
								NSDictionary *infoDict = [NSDictionary dictionaryWithObjectsAndKeys:@"Object 1", @"Key 1", @"Object 2", @"Key 2", nil];
								localNotification.userInfo = infoDict;
								
								[[UIApplication sharedApplication] scheduleLocalNotification:localNotification];
								}
								
	
								//print WC accelerations
								NSString *resultString = [[NSString alloc]
														  initWithFormat: @"Ax %.2f", accVectorWC->x];
								_resultLabel1.text = resultString;
								NSString *resultString2 = [[NSString alloc]
														   initWithFormat: @"Ay %.2f", accVectorWC->y];
								_resultLabel2.text = resultString2;
								NSString *resultString3 = [[NSString alloc]
														   initWithFormat: @"Az %.2f", accVectorWC->z];
								_resultLabel3.text = resultString3;
								
								NSLog(@"wc2:Ax %lf Ay %lf Az %lf",accVectorWC->x,accVectorWC->y,accVectorWC->z);

								//print orientations
								NSString *resultString4 = [[NSString alloc]
														   initWithFormat: @"Ox %.1f", orientationVector->x];
								_resultLabel7.text = resultString4;
								NSString *resultString5 = [[NSString alloc]
														   initWithFormat: @"Oy %.1f",orientationVector->y];
								_resultLabel8.text = resultString5;
								NSString *resultString6 = [[NSString alloc]
														   initWithFormat: @"Oz %.1f",orientationVector->z];
								_resultLabel9.text = resultString6;
								
								NSLog(@"Orientation:Ox %lf Oy %lf Oz %lf",orientationVector->x,orientationVector->y,orientationVector->z);
								
										
								
								NSLog(@"wc3:Ax %lf Ay %lf Az %lf",accVectorWC_unbiased->x,accVectorWC_unbiased->y,accVectorWC_unbiased->z);
								NSLog(@"offs:x %lf oy %lf Oz %lf",ax_offset,ay_offset,az_offset);
								
								//print unbiased accelerations
								//print orientations
								NSString *resultString7 = [[NSString alloc]
														   initWithFormat: @"Ax %.2f", accVectorWC_unbiased->x];
								_resultLabel4.text = resultString7;
								NSString *resultString8 = [[NSString alloc]
														   initWithFormat: @"Ay %.2f",accVectorWC_unbiased->y];
								_resultLabel5.text = resultString8;
								NSString *resultString9 = [[NSString alloc]
														   initWithFormat: @"Az %.2f",accVectorWC_unbiased->z];
								_resultLabel6.text = resultString9;
								

								
								
								
								//JEREMY'S CODE ABOVE UNTIL TERMINATING COMMENT
								

								totalSamples++;

								if ([UIDevice currentDevice].batteryLevel != batteryLevel)
								{
									batteryLevel = [UIDevice currentDevice].batteryLevel;
									NSLog(@"battery level: %f  acc samples:%d",batteryLevel,totalSamples);
								}
								accVector->x =  data.acceleration.x * 9.81f;
								accVector->y =  data.acceleration.y * 9.81f;
								accVector->z =  data.acceleration.z * 9.81f;
								accVector->accuracy = 0;
								accVector->sampleTimeInMsec = ([[NSDate date] timeIntervalSince1970]*1000);//data.timestamp*1000.0;
								[algoDetection ProcessNewAccVector:accVector];
								//NSLog(@"x%f y%f z%f time %lld",data.acceleration.x,data.acceleration.y,data.acceleration.z,accVector->sampleTimeInMsec);
								//NSLog(@"x%f y%f z%f time %lld",accVector->x,accVector->y,accVector->z,accVector->sampleTimeInMsec);                                                     //if (acc_i--==0)
								//{
								//    acc_i=600;
								//    NSLog(@"600 acc");
								//}
								//NSLog(@"isWalking = %s size%f",algoDetection->isWalking?"YES":"NO",accVector->size);
								if (Gps_state != algoDetection->isGPSneeded) {
									//[locationManager stopUpdatingLocation];
									if (algoDetection->isGPSneeded)
									{
										[locationManager setDesiredAccuracy:kCLLocationAccuracyThreeKilometers];
										[locationManager setDistanceFilter:10000];
										NSLog(@"GPS ON");
										Gps_state = YES;
										//                                                             if (AGps_state)
										//                                                                 [locationManager stopMonitoringSignificantLocationChanges];
										[locationManager startUpdatingLocation];
										//[self startGps];
									}
									else
									{
										//[locationManager setDesiredAccuracy:kCLLocationAccuracyHundredMeters];
										//[locationManager setDistanceFilter:100];
										NSLog(@"GPS OFF");                                                             [locationManager stopUpdatingLocation];
										//                                                             if (AGps_state)
										//                                                                 [locationManager startMonitoringSignificantLocationChanges];
										Gps_state = NO;
										//[self stopGps];
									}
									//[locationManager startUpdatingLocation];
								}
								if (Gps_state == NO && algoDetection->isStateChanged)
								{
									NSLog(@"movement state %s StepCnt %d",[MovementStatesDescription(algoDetection->movementState) UTF8String],algoDetection->stepsCnt);
									//                                                         if  (algoDetection->movementState == Steady)
									//                                                         {
									//                                                             AGps_state = YES;
									//                                                             [locationManager startMonitoringSignificantLocationChanges];
									//                                                         }
									//                                                         else
									//                                                         {
									//                                                             AGps_state = NO;
									//                                                             [locationManager stopMonitoringSignificantLocationChanges];
									//                                                         }
									
								}
								if (algoDetection->algoSensorState != algoDetection->algoSensorLastState) {
									//[self stopSensor];
									if (algoDetection->algoSensorState == FastSearch) {
										[motionManager setAccelerometerUpdateInterval:1.0/10.0];                                                            //sensorDelayAcc = SensorManager.SENSOR_DELAY_UI;
										//[locationManager startUpdatingLocation];
									}
									else if (algoDetection->algoSensorState == SleepSearch) {
										[motionManager setAccelerometerUpdateInterval:1.0/1.0];
										//[locationManager stopUpdatingLocation];
										//sensorDelayAcc = SensorManager.SENSOR_DELAY_NORMAL;
									}
									//[self startSensor];
								}
								NSString * popUpString = nil;
								if (algoDetection->isStateChanged &&
									algoDetection->lastMovementState == Parking && algoDetection->movementState == Walking) {
									popUpString = @"Did you just park?";
									//[self setParkingLocation:algoDetection->algoCurrLocVector];
								}
								if (algoDetection->isStateChanged &&
									algoDetection->lastMovementState != Parking &&
									algoDetection->movementState  == Driving)
								{
									
									popUpString  = @"Are you driving?";
									
								}
								if (algoDetection->walkingState != algoDetection->lastWalkingState) {
									//SharedPreferences * pref = [self getSharedPreferences:PREFS_NAME param1:MODE_PRIVATE];
									//[[[pref edit] putString:@"walking_state" param1:[algoDetection.walkingState name]] commit];
									if (algoDetection->walkingState == WalkingFromCar100m)
										popUpString = @"Are you moving away from the car(100m)?";
									else if (algoDetection->walkingState == WalkingToCar50m) {
										//[self SendAlgoLoc];
										popUpString = @"Are you approaching your car(50m)?";
									}
								}
								popUpString = @"ititit";
								if (0/*popUpString*/)
								{
									NSLog(@"%@",popUpString);
									UILocalNotification *localNotification = [[UILocalNotification alloc] init];
									// create a calendar to form date
									NSDate *item = [NSDate dateWithTimeIntervalSinceNow:1];
									
									localNotification.fireDate = item;
									localNotification.timeZone = [NSTimeZone defaultTimeZone];
									localNotification.alertBody = popUpString;
									localNotification.soundName = UILocalNotificationDefaultSoundName;
									localNotification.applicationIconBadgeNumber = 1;
									
									NSDictionary *infoDict = [NSDictionary dictionaryWithObjectsAndKeys:@"Object 1", @"Key 1", @"Object 2", @"Key 2", nil];
									localNotification.userInfo = infoDict;
									
									[[UIApplication sharedApplication] scheduleLocalNotification:localNotification];
									[localNotification release];
								}
								
							}
							else
							{//if not enough samples...
								NSLog(@"not enough samples");
								//return(0);
							}
						});
	 }
     ];
    
}

//- (Boolean)jeremyFunction:(AlgoVector*) AccelerationVectorWC wRotationMatrix:(CMRotationMatrix) rotationMatrix wOrientationVector:(AlgoVector*)OrientationVector
//- (boolean)jeremyFunction( double Ax,double Ay,double Az,double Ox,double Oy,double Oz,double R1,double R2,double R3,double R4,double R5,double R6,double R7,double R8,double R9)

//(int)arg1 withArg2:(int)arg2
//to call: [objectWithOurMethod methodName:int1 withArg2:int2];

//) Ax:(double) Ay:(double) Az:(double) Ox:(double) Oy:(double) Oz:(double) R1:(double) R2:(double) R3:(double) R4:(double) R5:(double) R6:(double) R7:(double) R8:(double) R9
//{
//    double x;
//    x=AccelerationVectorWC->x*2;
//    NSLog(@"%lf",x);
//   return(0);
//    // Release any retained subviews of the main view.
//}



- (void)viewDidUnload
{
    [super viewDidUnload];
    // Release any retained subviews of the main view.
}

- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation
{
    return (interfaceOrientation != UIInterfaceOrientationPortraitUpsideDown);
}

- (IBAction)butttonDIDpressed:(id)sender {
    NSLog(@"test");
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


//jeremy

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
//jeremy



-(void)writeStringToFile:(NSString*)aString {
	
    // Build the path, and create if needed.
    NSString* filePath = [NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES) objectAtIndex:0];
    NSString* fileName = @"myTextFile.txt";
    NSString* fileAtPath = [filePath stringByAppendingPathComponent:fileName];
	
    if (![[NSFileManager defaultManager] fileExistsAtPath:fileAtPath]) {
        [[NSFileManager defaultManager] createFileAtPath:fileAtPath contents:nil attributes:nil];
    }
	
    // The main act.
    [[aString dataUsingEncoding:NSUTF8StringEncoding] writeToFile:fileAtPath atomically:NO];
}


- (void) displayInfo: (id) sender {
	//    double farenheit = [_tempText.text doubleValue];
    double testvalue = 666.6;
    
    NSString *resultString = [[NSString alloc]
                              initWithFormat: @"Celsius %f", testvalue];
    _resultLabel1.text = resultString;
}


@end



