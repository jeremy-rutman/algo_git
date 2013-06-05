//
//  driving_detector.m
//  testAlgo
//
//  Created by Lior Neu-ner on 2013/06/02.
//  Copyright (c) 2013 itaidddd@gmail.com. All rights reserved.
//

#import "drivingDetector.h"
#import "MainService.h"
#import "ViewController.h"

#import <AudioToolbox/AudioToolbox.h>
#import <AudioToolbox/AudioServices.h>

@implementation drivingDetector

// function taking inputs of algovectors for orientation, acceleration. one can fake those if there is no iphone handy / testing purposes
- (void)drivingDetectorProcessInput:  (AlgoVector *)theAccelerationVector anOrientationVector:(AlgoVector *)theOrientationVector
{
    //i am not sure what the following line is for....
    //[super viewDidLoad];

	// Do any additional setup after loading the view, typically from a nib.
    //    algoDetection = [[AlgoDetection alloc] init];
    //    accVector = [[AlgoVector alloc] init];
    //    orientationVector = theOrientationVector;
	
    //    accVectorWC_unbiased= [[AlgoVector alloc] init];
    
	
	//jeremy
    __block double ax_offset,ay_offset,az_offset; //these are the long-timescale biases of the acceleration vector (which are assumed to be 'really' 0)
	__block int i;
	__block int j;
	__block double timeStamp;
	
	__block Boolean mdriving_velocity_threshold_2s_in_recent_history;
	__block Boolean mdriving_velocity_threshold_5s_in_recent_history;
	__block Boolean mdriving_velocity_threshold_10s_in_recent_history;
	
	AlgoVector * theaccVectorWC_unbiased= [[AlgoVector alloc] init];
    
    //[locationManager setPurpose:@"My Custom Purpose Message..."];
    //[locationManager startUpdatingLocation];
    //[locationManager startMonitoringSignificantLocationChanges];
    Gps_state = NO;
    AGps_state = NO;
    
    [[UIDevice currentDevice] setBatteryMonitoringEnabled:YES];
	
	isDriving=0;
	
	//impose limits on measured data
	if(theAccelerationVector->x>mMaxAcceptedAcceleration) theAccelerationVector->x=mMaxAcceptedAcceleration;
	if(theAccelerationVector->y>mMaxAcceptedAcceleration) theAccelerationVector->y=mMaxAcceptedAcceleration;
	if(theAccelerationVector->z>mMaxAcceptedAcceleration) theAccelerationVector->z=mMaxAcceptedAcceleration;
	if(theAccelerationVector->x<-mMaxAcceptedAcceleration) theAccelerationVector->x=-mMaxAcceptedAcceleration;
	if(theAccelerationVector->y<-mMaxAcceptedAcceleration) theAccelerationVector->y=-mMaxAcceptedAcceleration;
	if(theAccelerationVector->z<-mMaxAcceptedAcceleration) theAccelerationVector->z=mMaxAcceptedAcceleration;
	
	//determine battery level
	if ([UIDevice currentDevice].batteryLevel != batteryLevel)
	{
		batteryLevel = [UIDevice currentDevice].batteryLevel;
		NSLog(@"battery level: %f  acc samples:%d",batteryLevel,totalSamples);
	}
    
	//initialize values if first time thru
#ifdef DEBUGJ
	NSLog(@"input %d/%d:Ax %.2f Ay %.2f Az %.2f Ox %.2f Oy %.2f Oz %.2f bl %.2f",jmN_samples_taken,mN_samples_in_driving_analysis,
		  theAccelerationVector->x,theAccelerationVector->y,theAccelerationVector->z,theOrientationVector->x,theOrientationVector->y,theOrientationVector->z,batteryLevel );
#endif
	
    //	NSLog(@"sampling rate %d",mSamplingRate);
	timeStamp=[[NSDate date] timeIntervalSince1970];
	PrevioustimeStamp=[[NSDate date] timeIntervalSince1970];
	//find average linear accelerations -  an offset which 'should be' zero
    
	jmN_samples_taken++;
	if (jmN_samples_taken>32000) jmN_samples_taken=mN_samples_in_driving_analysis+1;
    
	if (jmN_samples_taken>mN_samples_in_driving_analysis)
	{
		mEnough_driving_samples=1;
        [self Ccalc_datastructure_avg2:0 slength:mN_samples_in_driving_analysis tlength:mN_samples_in_driving_analysis ans:&ax_offset];
        [self Ccalc_datastructure_avg2:1 slength:mN_samples_in_driving_analysis tlength:mN_samples_in_driving_analysis ans:&ay_offset];
        [self Ccalc_datastructure_avg2:2 slength:mN_samples_in_driving_analysis tlength:mN_samples_in_driving_analysis ans:&az_offset];
        
	}
    

    
    
	else
	{
		mEnough_driving_samples=0;
        statusString = [[NSString alloc]
								  initWithFormat: @"%d/200 SAMPLES",jmN_samples_taken];
		NSLog(@"sample no. %d/%d",jmN_samples_taken,mN_samples_in_driving_analysis );
		ax_offset=0;
		ay_offset=0;
		az_offset=0;
	}
	
	
	accVectorWC->x=theAccelerationVector->x;
    accVectorWC->y=theAccelerationVector->y;
	accVectorWC->z=theAccelerationVector->z;

	theaccVectorWC_unbiased->x=theAccelerationVector->x-ax_offset;
	theaccVectorWC_unbiased->y=theAccelerationVector->y-ay_offset;
	theaccVectorWC_unbiased->z=theAccelerationVector->z-az_offset;

    accVectorWC_unbiased->x=theAccelerationVector->x;
    accVectorWC_unbiased->y=theAccelerationVector->y;
    accVectorWC_unbiased->z=theAccelerationVector->z;
    
    orientationVector->x=theOrientationVector->x;
    orientationVector->y=theOrientationVector->y;
    orientationVector->z=theOrientationVector->z;

#ifdef DEBUGJ
	NSLog(@"Ax %.2f Axc %.2f Ay %.2f Ayc %.2f",theAccelerationVector->x,theaccVectorWC_unbiased->x,theAccelerationVector->y,theaccVectorWC_unbiased->y);
#endif
	
	mSensor_history[mN_samples_in_driving_analysis-1][0]=theAccelerationVector->x;
	mSensor_history[mN_samples_in_driving_analysis-1][1]=theAccelerationVector->y;
	mSensor_history[mN_samples_in_driving_analysis-1][2]=theAccelerationVector->z;
	mSensor_history[mN_samples_in_driving_analysis-1][3]=theaccVectorWC_unbiased->x;
	mSensor_history[mN_samples_in_driving_analysis-1][4]=theaccVectorWC_unbiased->y;
	mSensor_history[mN_samples_in_driving_analysis-1][5]=theaccVectorWC_unbiased->z;
	mSensor_history[mN_samples_in_driving_analysis-1][6]=theOrientationVector->x;
	mSensor_history[mN_samples_in_driving_analysis-1][7]=theOrientationVector->y;
	mSensor_history[mN_samples_in_driving_analysis-1][8]=theOrientationVector->z;
	mSensor_history[mN_samples_in_driving_analysis-1][9]=timeStamp;
	
	//make this circular buffer for faster performance
	//shift buffer back
	for( i=0;i<mN_samples_in_driving_analysis-1;i++)
	{
		for( j=0;j<mNfields;j++)
		{
			mSensor_history[i][j]=mSensor_history[i+1][j];
		}
	}
	
	if (mEnough_driving_samples)
	{
		
		statusString = [[NSString alloc]
								  initWithFormat: @"t:%.1f",timeStamp];
//		_resultLabel0.text = resultString;
	
        
		//smoothed  velocity components from acceleration and history
		//moved to average over last N samples instead of  using beta
        //	V[0]=V[0]*mBeta+accVectorWC_unbiased->x/mSamplingRate;
        //		V[1]=V[1]*mBeta+accVectorWC_unbiased->y/mSamplingRate;
        //		V[2]=V[2]*mBeta+accVectorWC_unbiased->z/mSamplingRate;
		
		//find velocities using integration over different numbers of samples
		[self integrateAcceleration:velocityVector_1s Nsamples:(1*mSamplingRate)];
		[self integrateAcceleration:velocityVector_2s Nsamples:(2*mSamplingRate)];
		[self integrateAcceleration:velocityVector_5s Nsamples:(5*mSamplingRate)];
		[self integrateAcceleration:velocityVector_10s Nsamples:(10*mSamplingRate)];
        
#ifdef DEBUGV
		NSLog(@"velocity 2s (%.3f %.3f %.3f)",velocityVector_2s->x,velocityVector_2s->y,velocityVector_2s->z);
		NSLog(@"velocity 5s (%.3f %.3f %.3f)",velocityVector_5s->x,velocityVector_5s->y,velocityVector_5s->z);
		NSLog(@"velocity 10s (%.3f %.3f %.3f)",velocityVector_10s->x,velocityVector_10s->y,velocityVector_10s->z);
#endif
		
		 V2h=sqrt(velocityVector_2s->x*velocityVector_2s->x+velocityVector_2s->y*velocityVector_2s->y);
		 V5h=sqrt(velocityVector_5s->x*velocityVector_5s->x+velocityVector_5s->y*velocityVector_5s->y);
		 V10h=sqrt(velocityVector_10s->x*velocityVector_10s->x+velocityVector_10s->y*velocityVector_10s->y);
		mVhCurrent=V5h;
        
		// calculate Vz (up/down velocity) avg and std for 2s of samples -
		mVzStd=0;
		mVzAvg=0;
		mVz[mVzCount]=velocityVector_1s->z; //use of N samples is arbitrary - maybe find optimium?
		for (i=0;i<mN_samples_in_2s_analysis;i++)
		{
			mVzAvg+=mVz[i];
		}
		mVzAvg=mVzAvg/mN_samples_in_2s_analysis;
		
		for (i=0;i<mN_samples_in_2s_analysis;i++)
		{
			mVzStd+=(mVz[i]-mVzAvg)*(mVz[i]-mVzAvg);
		}
		mVzStd=sqrt(mVzStd/mN_samples_in_2s_analysis);
		//use absolute value of mVz
		if(mVzAvg<0)mVzAvg=-mVzAvg;
		
		// calculate Oy avg and std for 20s of samples - elevation
		mOy_20s[mSampleCount20s++]=orientationVector->y;
		if(mSampleCount20s>mN_samples_in_20s_analysis-1) mSampleCount20s=0;
		mOy_std20s=0;
		mOy_avg20s=0;
		for ( i=0;i<mN_samples_in_20s_analysis;i++)
		{
			mOy_avg20s+=mOy_20s[i];
		}
		mOy_avg20s=mOy_avg20s/mN_samples_in_20s_analysis;
		for ( i=0;i<mN_samples_in_20s_analysis;i++)
		{
			mOy_std20s+=(mOy_20s[i]-mOy_avg20s)*(mOy_20s[i]-mOy_avg20s);
		}
		mOy_std20s=sqrt(mOy_std20s/mN_samples_in_20s_analysis);
		
		// calculate Oy avg and std for 2s of samples
		mOy_2s[mSampleCount2s]=orientationVector->y;
		mOy_std2s=0;
		mOy_avg2s=0;
		for ( i=0;i<mN_samples_in_2s_analysis;i++)
		{
			mOy_avg2s+=mOy_2s[i];
		}
		mOy_avg2s=mOy_avg2s/mN_samples_in_2s_analysis;
		for (i=0;i<mN_samples_in_2s_analysis;i++)
		{
			mOy_std2s+=(mOy_2s[i]-mOy_avg2s)*(mOy_2s[i]-mOy_avg2s);
		}
		mOy_std2s=sqrt(mOy_std2s/mN_samples_in_2s_analysis);
		
		// calculate Oz avg and std for 2s of samples
		mOz_2s[mSampleCount2s++]=orientationVector->z;
		if(mSampleCount2s>mN_samples_in_2s_analysis-1) mSampleCount2s=0;
		// only increment for last variable mOz_avg_2s
		mOz_avg2s=0;
		mOz_std2s=0;
		for ( i=0;i<mN_samples_in_2s_analysis;i++)
		{
			mOz_avg2s+=mOz_2s[i];
		}
		mOz_avg2s=mOz_avg2s/mN_samples_in_2s_analysis;
		for (i=0;i<mN_samples_in_2s_analysis;i++)
		{
			mOz_std2s+=(mOz_2s[i]-mOz_avg2s)*(mOz_2s[i]-mOz_avg2s);
		}
		mOz_std2s=sqrt(mOz_std2s/mN_samples_in_2s_analysis);
        
		dOx_dt=[self calc_dOx_dt];
		
		//initialize string w. nothing
        popUpString = [NSString stringWithFormat:@""];
		
        mUserState= [NSString stringWithFormat:@"U"];  //default is unknown user state
        
		//are we walking??
		if (//mAvgGabs>mGyro_Walking_Threshold &&
			//	mVzStd>mVzStdMinWalkingThreshold &&
			//add Astd min thresholds
			mOy_std2s>mOyStd2sMinWalkingThreshold &&
			mOz_std2s>mOzStd2sMinWalkingThreshold &&
			mOy_std20s>mOyStd20sMinWalkingThreshold)
		{
			mWalking=1;
			NSLog(@"jeremy thinks - walking!");
			popUpString = [NSString stringWithFormat:@"jeremy thinks - walking"];
			mUserState=@"W";
			//playWalk();
			mTimeofLastWalkingDetection=timeStamp;
			//get system time, this is probably not right way to do it
			noInterveningWalkingFlag=FALSE;
		}
		else mWalking=0;
#ifdef DEBUGJ
		NSLog(@"walking criteria:mOystd2s  %d mOzstd2s %d mOzstd20s %d",mOy_std2s>mOyStd2sMinWalkingThreshold,mOz_std2s>mOzStd2sMinWalkingThreshold,mOy_std20s>mOyStd20sMinWalkingThreshold);
#endif
		
		if(V2h>mdriving_velocity_threshold_2s)
		{
			time_of_last_mdriving_velocity_threshold_2s=timeStamp;
		}
		if(timeStamp-time_of_last_mdriving_velocity_threshold_2s<mDuration_in_which_to_accept_high_velocities) mdriving_velocity_threshold_2s_in_recent_history=TRUE;
		else mdriving_velocity_threshold_2s_in_recent_history=FALSE;
        
		if(V5h>mdriving_velocity_threshold_5s)
		{
			time_of_last_mdriving_velocity_threshold_5s=timeStamp;
		}
		if(timeStamp-time_of_last_mdriving_velocity_threshold_5s<mDuration_in_which_to_accept_high_velocities) mdriving_velocity_threshold_5s_in_recent_history=TRUE;
		else mdriving_velocity_threshold_5s_in_recent_history=FALSE;
        
		if(V10h>mdriving_velocity_threshold_10s)
		{
			time_of_last_mdriving_velocity_threshold_10s=timeStamp;
		}
		if(timeStamp-time_of_last_mdriving_velocity_threshold_10s<mDuration_in_which_to_accept_high_velocities) mdriving_velocity_threshold_10s_in_recent_history=TRUE;
		else mdriving_velocity_threshold_10s_in_recent_history=FALSE;
        
		
		//are we driving??
		if (!mWalking&&
			mdriving_velocity_threshold_2s_in_recent_history &&
			mdriving_velocity_threshold_5s_in_recent_history &&
			mdriving_velocity_threshold_10s_in_recent_history &&
			(timeStamp-mTimeofLastWalkingDetection)>mtime_after_walking_to_filter_driving &&
			mVzStd<mVzStdMaxDrivingThreshold &&
			mVzAvg<mVzAvgMaxDrivingThreshold &&
			mOy_std20s<mOyStd20sMaxDrivingThreshold &&
			mOy_std2s<mOyStd2sMaxDrivingThreshold &&
			mOy_std2s>mOyStd2sMinDrivingThreshold &&
			mOz_std2s>mOzStd2sMinDrivingThreshold &&
			mOz_std2s<mOzStd2sMaxDrivingThreshold)
		{
			mDriving=1;
			mUserState=@"D";
			noInterveningWalkingFlag=TRUE;
			mTimeofLastDrivingDetection=timeStamp;
			NSLog(@"jeremy thinks - driving!");
			//	popUpString=sprintf("jeremy thinks - driving!");
			popUpString = [NSString stringWithFormat:@"jeremy thinks - driving"];
			//playHonk();
		}
		else mDriving=0;
#ifdef DEBUGJ
        
		NSLog(@"driving criteria:mVh %d time %d mVzstd %d mVzavg %d mOystd20 %d mOystd2< %d mOystd2> %d mOzstd2> %d mOzstd2 < %d",mVhCurrent>mdriving_velocity_threshold_10s,(timeStamp-mTimeofLastWalkingDetection)>mtime_after_walking_to_filter_driving,mVzStd<mVzStdMaxDrivingThreshold,mVzAvg<mVzAvgMaxDrivingThreshold,mOy_std20s<mOyStd20sMaxDrivingThreshold,mOy_std2s<mOyStd2sMaxDrivingThreshold ,mOy_std2s>mOyStd2sMinDrivingThreshold ,mOz_std2s>mOzStd2sMinDrivingThreshold,mOz_std2s<mOzStd2sMaxDrivingThreshold    );
		
#endif
        
		
		//'nominal driving state' -  if less than some threshold since last driving detection, and no walking detected meantime, then we are still driving (or sitting in car...)
		if (timeStamp-mTimeofLastDrivingDetection<mExpectationTimeBetweenDrivingDetections && noInterveningWalkingFlag==TRUE)
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
			if(V5h>crazy_driver_velocity_threshold)
			{
				crazylogString = [[NSString alloc] initWithFormat: @"time %.2f excessive horizontal velocity %.2f",timeStamp,V5h];
				[self writeToLogFile:crazylogString aFileName:crazydriverfile];
				NSLog(@"%@",crazylogString);
			}
			if(dOx_dt>crazy_driver_dOx_dt_threshold)
			{
				crazylogString = [[NSString alloc] initWithFormat: @"time %.2f excessive turning rate %.2f",timeStamp,dOx_dt];
				[self writeToLogFile:crazylogString aFileName:crazydriverfile];
				NSLog(@"%@",crazylogString);
                
			}
			if(V5h>crazy_driver_velocity_threshold)
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
			AudioServicesCreateSystemSoundID ((__bridge CFURLRef)soundUrl, &soundID);
			AudioServicesPlaySystemSound(soundID);
		}
		
		
#ifdef DEBUGJ
		NSLog(@"debug before string construction");
		NSLog(@"battlevel %.2f",batteryLevel);
#endif
		
		//WRITE TO LOG FILE
		NSString *logString = [[NSString alloc]
							   initWithFormat: @"%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %@ %@ %.2f",timeStamp,theAccelerationVector->x,theAccelerationVector->y,theAccelerationVector->z,theaccVectorWC_unbiased->x,theaccVectorWC_unbiased->y,theaccVectorWC_unbiased->z,theOrientationVector->x,theOrientationVector->y,theOrientationVector->z,claimedUserState,mUserState,batteryLevel];
#ifdef DEBUGJ
        NSLog(@"log string %@",logString);
		NSLog(@"filename %@",lglobal_logFileName);
#endif

        [self writeToLogFile:logString aFileName:lglobal_logFileName];

		

		totalSamples++;
		
	};
}


-(void) drivingDetectorInit
{

    //mSamplingRate=10.0; //in samples/second  //defined in calling function since it needs to know for the iphone setup
    NSLog(@"First run!");
//    AlgoDetection = [[AlgoDetection alloc] init];
    mSamplingRate=10.0; //in samples/second  //defined in calling function since it needs to know for the iphone setup
 
    
    orientationVector=[[AlgoVector alloc] init];
    accVectorWC= [[AlgoVector alloc] init];
    accVectorWC_unbiased= [[AlgoVector alloc] init];
 //   algoLocVector = [[AlgoLocVector alloc] init];
    locationManager = [[CLLocationManager alloc] init];
    
/*    locationManager = [[CLLocationManager alloc] init];
    locationManager.delegate = self;
    locationManager.desiredAccuracy = kCLLocationAccuracyThreeKilometers;//kCLLocationAccuracyKilometer;
    locationManager.distanceFilter = 10000;
  */  
    velocityVector_1s=[[AlgoVector alloc] init];
    velocityVector_2s=[[AlgoVector alloc] init];
    velocityVector_5s=[[AlgoVector alloc] init];
    velocityVector_10s=[[AlgoVector alloc] init];
    
    mNfields=10; //fields in dataq record - Ax Ay Az Ax_no(no offset) Ay_no Az_no Ox Oy Oz
    mDuration_of_driving_analysis=20; //20s analysis duration total
    mN_samples_in_driving_analysis=mDuration_of_driving_analysis*mSamplingRate;
    mN_samples_in_1s_analysis=1*mSamplingRate;
    mN_samples_in_2s_analysis=2*mSamplingRate;
    mN_samples_in_5s_analysis=5*mSamplingRate;
    mN_samples_in_20s_analysis=20*mSamplingRate;
    
    mVzStdMinWalkingThreshold=1.0; //.08
    mOyStd2sMinWalkingThreshold=10.0;
    mOyStd20sMinWalkingThreshold=10.0;
    mOzStd2sMinWalkingThreshold=5.0;
    
    mdriving_velocity_threshold_2s=0.8; //was 0.5  V10h  reaches abt .016kkk
    mdriving_velocity_threshold_5s=1.5; //was 0.5  V10h  reaches abt .016kkk
    mdriving_velocity_threshold_10s=1.5; //was 0.5  V10h  reaches abt .016kkk
    mVzStdMaxDrivingThreshold=0.5;
    mVzAvgMaxDrivingThreshold=0.7;
    mOyStd20sMaxDrivingThreshold=7.0;
    mOyStd2sMaxDrivingThreshold=5.0;
    mOyStd2sMinDrivingThreshold=0.2;
    
    mOzStd2sMaxDrivingThreshold=5.0;
    mOzStd2sMinDrivingThreshold=0.2;
    time_of_last_mdriving_velocity_threshold_2s=0;
    time_of_last_mdriving_velocity_threshold_5s=0;
    time_of_last_mdriving_velocity_threshold_10s=0;
    mDuration_in_which_to_accept_high_velocities=5; //look at last 5 sec
    
    //crazy driver profiling
    crazy_driver_velocity_threshold=1.5; //was 0.5
    crazy_driver_dOx_dt_threshold=350; //was 0.5
    crazy_driver_Ah_threshold=1; //was 0.5
    
    mEnough_driving_samples=0;  //0 false, 1 true
    mBeta=0.95; //in samples/second
    V[0]=0.0;V[1]=0.0;V[2]=0.0;
    Vhmax=0.0; //Vh2max=0.0;
    mVzCount=0;
    mtime_after_walking_to_filter_driving=5;
    mTimeofLastDrivingDetection=0;
    mTimeofLastWalkingDetection=0;
    mExpectationTimeBetweenDrivingDetections=5*60; //if 5 minutes or less from last driving detection and there is no walking detected, we are still driving
    mMaxAcceptedAcceleration=10;
    
    //			[initWithPath (NSString*) @""];
    
    //						mV_uncorrected[0]=0.0;
    
    PrevioustimeStamp=[[NSDate date] timeIntervalSince1970]-.1;
//    timeStamp=[[NSDate date] timeIntervalSince1970];
    timeOfLastNotification=[[NSDate date] timeIntervalSince1970];
    
    //generate a logfile name from the date and time
    NSDate *today = [NSDate date];
    NSDateFormatter *dateFormat = [[NSDateFormatter alloc] init];
    //	[dateFormat setDateFormat:@"ddMMYY_HHmm"];
    [dateFormat setDateFormat:@"ddMMYY"];
    NSString *dateString = [dateFormat stringFromDate:today];
    global_logFileName=[NSString stringWithFormat:@"%@.txt",dateString];
    
    //	[[NSString alloc] initWithFormat: @"logfile2.txt"];
    
    lglobal_logFileName=[[NSString alloc] initWithFormat:@"%@.txt",dateString];
    
    //thelogFileName=[NSString stringWithFormat:@"%@.txt",dateString];
    
    NSLog(@"global logfilename is:%@ lg %@",global_logFileName,lglobal_logFileName);
    
    //write initial file lines
    
    [self writeToLogFile:dateString aFileName:global_logFileName];
    [self writeToLogFile:@"\ntime Ax Ay Az Ox Oy Oz claimedstate detectedstate battlevel\n" aFileName:global_logFileName];
    
    //		[self writeToLogFile:dateString aFileName:@"logfile2.txt"];
    //		[self writeToLogFile:@"\ntime Ax Ay Az Ox Oy Oz state\n" aFileName:@"logfile2.txt"];
    
    claimedUserState=@"W";
    mUserState=@"U";
    
    
}



-(AlgoVector *)integrateAcceleration:(AlgoVector *)aVector Nsamples:(int)N_samples
{
	int i;
	double dt;
	aVector->x=0;
	aVector->y=0;
	aVector->z=0;
	for (i=0;i<N_samples;i++)
	{
		
		dt=mSensor_history[mN_samples_in_driving_analysis-i-1][9]-
		mSensor_history[mN_samples_in_driving_analysis-i-2][9];
		
		aVector->x+=mSensor_history[mN_samples_in_driving_analysis-i-1][3]*dt;
		aVector->y+=mSensor_history[mN_samples_in_driving_analysis-i-1][4]*dt;
		aVector->z+=mSensor_history[mN_samples_in_driving_analysis-i-1][5]*dt;
	}
	//this is wrong - integration does not divide by  N
    //	aVector->x/=N_samples;
    //	aVector->y/=N_samples;
    //	aVector->z/=N_samples;
#ifdef DEBUGV
	NSLog(@"integrated V:(%.2f %.2f %.2f): %d samples",aVector->x,aVector->y,aVector->z,N_samples);
#endif
	return(aVector);
}



-(double) calc_dOx_dt {
	//calculate dOx/dt using average over 1s of samples , at points 1s apart
	double dOx=0;
    double average_dOx_dt=0;
	int i;
	int k=mN_samples_in_driving_analysis-1-mN_samples_in_1s_analysis;
    
	for ( i=mN_samples_in_driving_analysis-1;i>k;i--)
	{
        //	NSLog(@"calcdoX/dt:i=%d ",i);
		dOx+=mSensor_history[i][6]-mSensor_history[i-mN_samples_in_1s_analysis][6];
		dOx_dt=dOx/(mSensor_history[i][9]-mSensor_history[i-mN_samples_in_1s_analysis][9]);  //vals[9] is time
		average_dOx_dt+=dOx_dt;
        //		NSLog(@"dOx/dt avg %f dOx %.2f cur %.2f ",average_dOx_dt,dOx,dOx_dt);
        
	}
	average_dOx_dt=average_dOx_dt/mN_samples_in_1s_analysis;
#ifdef DEBUGJ
	NSLog(@"calcdoX/dt avg %f ",average_dOx_dt);
#endif
	
	return average_dOx_dt;
}


-(void) writeToLogFile:(NSString*)content aFileName:(NSString *)filename{
	content = [NSString stringWithFormat:@"%@\n",content];
    
	//get the documents directory:
	NSString *documentsDirectory = [NSHomeDirectory() stringByAppendingPathComponent:@"Documents"];
	NSString *fileName = [NSString stringWithFormat:@"%@/%@", documentsDirectory,filename];
    
#ifdef DEBUGLOG
	NSLog(@"%@ written to %@",content,filename);
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




-(double)  Ccalc_datastructure_avg2: (int)field   slength:(int)stdlength tlength:(int)totlength ans:(double *)answer
{
	double mean=0.0;
	//		int stdlength=10;
	//		int totlength=10;
	int i=0;
	if (stdlength>totlength) {
		mean=-1.0;return mean;
	}
	for ( i=totlength-stdlength;i<totlength;i++)
	{
		mean+=mSensor_history[i][field];
	}
	mean=mean/stdlength;
	*answer=mean;
	return (double)mean;
}


// function taking inputs of algovectors for orientation, acceleration. one can fake those if there is no iphone handy / testing purposes



@end


#import <AudioToolbox/AudioToolbox.h>
#import <AudioToolbox/AudioServices.h>

@implementation Sound {
	SystemSoundID handle;
}

- (id) initWithPath: (NSString*) path
{
	self = [super init];
	NSString *const resourceDir = [[NSBundle mainBundle] resourcePath];
	NSString *const fullPath = [resourceDir stringByAppendingPathComponent:path];
	NSURL *const url = [NSURL fileURLWithPath:fullPath];
	OSStatus errcode = AudioServicesCreateSystemSoundID((__bridge CFURLRef) url, &handle);
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



#import <mach/mach.h>
#import <mach/mach_host.h>

+(natural_t) get_free_memory {
    mach_port_t host_port;
    mach_msg_type_number_t host_size;
    vm_size_t pagesize;
    host_port = mach_host_self();
    host_size = sizeof(vm_statistics_data_t) / sizeof(integer_t);
    host_page_size(host_port, &pagesize);
    vm_statistics_data_t vm_stat;
    
    if (host_statistics(host_port, HOST_VM_INFO, (host_info_t)&vm_stat, &host_size) != KERN_SUCCESS) {
        NSLog(@"Failed to fetch vm statistics");
        return 0;
    }
    
    /* Stats in bytes */
    natural_t mem_free = vm_stat.free_count * pagesize;
    return mem_free;
}



//#import "UIDeviceAdditions.h"
#include <sys/sysctl.h>
//#include <mach/mach.h>

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -



- (double)currentMemoryUsage {
    vm_statistics_data_t vmStats;
    mach_msg_type_number_t infoCount = HOST_VM_INFO_COUNT;
    kern_return_t kernReturn = host_statistics(mach_host_self(), HOST_VM_INFO, (host_info_t)&vmStats, &infoCount);
    
    if(kernReturn == KERN_SUCCESS)
        return vmStats.wire_count/1024.0;
    else return 0;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -




@end



