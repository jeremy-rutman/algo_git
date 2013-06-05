//
//  driving_detector.h
//  testAlgo
//
//  Created by Lior Neu-ner on 2013/06/02.
//  Copyright (c) 2013 itaidddd@gmail.com. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "MainService.h"



@interface drivingDetector : NSObject

{
@public
//    typedef struct tag1 {
  //      int time;
    //    double vals[10];  //A_wc, A_wc_unbiased, Orientation
        /*	double Ay;
         double Az;
         double Ox;
         double Oy;
         double Oz;
     //    */
//} Cdatastructure;
    
        int mNfields;
        double mMaxAcceptedAcceleration;
        
        int mDuration_of_driving_analysis;
        int mN_samples_in_driving_analysis;
        int mN_samples_in_1s_analysis;
        int mN_samples_in_2s_analysis;
        int mN_samples_in_20s_analysis;
        int mN_samples_in_5s_analysis;
    int totalSamples;
    int jmN_samples_taken ;    //walking thresholds
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
		
        double timeOfLastNotification;
    
    //
        double mVzAvg;
	 double mVhCurrent;
	 double mVzStd;
	 double mOy_std20s;
	 double mOy_avg20s;
	 double mOy_std2s;
	 double mOy_avg2s;
	 double mOz_avg2s;
	 double mOz_std2s;
	 double dOx_dt;
    double V2h;
    double V5h;
    double V10h;
    
        //crazy driver thresholds
        double crazy_driver_velocity_threshold;
        double crazy_driver_dOx_dt_threshold;
        double crazy_driver_Ah_threshold;
        
        int mN_samples_taken_initial_value;
        int mN_samples_taken_current_value;
        int mN_samples_taken;
        int mVzCount;
        double mSensor_history[2000][10];  //mN_samples_in_driving_analysis - itay didnt want #defines and I dont want to have to malloc
        int mEnough_driving_samples;
        double V[3];
        double mVz[2000];  //mN_samples_in_driving_analysis =
        int mSampleCount20s;
        int mSampleCount2s;
        double mOy_20s[2000]; //mN_samples_in_driving_analysis
        double mOy_2s[200]; //mN_samples_in_2s_analysis
        double mOz_2s[200]; //mN_samples_in_2s_analysis
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
        //NSString *thelogFileName;
        Boolean noInterveningWalkingFlag;
    double  batteryLevel;
    
    
    AlgoVector *accVectorWC;
    AlgoVector *accVectorWC_unbiased;
    AlgoVector *orientationVector;
	AlgoVector *velocityVector_1s;
	AlgoVector *velocityVector_2s;
	AlgoVector *velocityVector_5s;
	AlgoVector *velocityVector_10s;
    NSString *global_logFileName;
    
    
    BOOL debug;
    long acc_i;
    NSString *popUpString;
    NSString *lglobal_logFileName;
    Boolean isDriving;
    NSString *claimedUserState;
    NSString *mUserState;
    NSString *statusString;

}


- (void)drivingDetectorProcessInput:  (AlgoVector *)theAccelerationVector anOrientationVector:(AlgoVector *)theOrientationVector;
- (void) drivingDetectorInit;
- (AlgoVector *)integrateAcceleration:(AlgoVector *)aVector Nsamples:(int)N_samples;
-(double) calc_dOx_dt;
-(void) writeToLogFile:(NSString*)content aFileName:(NSString *)filename;
//-(void)timed_action;
-(double) Ccalc_datastructure_avg2: (int)field   slength:(int)stdlength tlength:(int)totlength ans:(double *)answer;
//+(natural_t) get_free_memory;
//- (double)currentMemoryUsage;


//- (void) Copy:(AlgoVector *)algVec;
//- (float) ReCalclulateSize;


@end



 @interface Sound : NSObject
 
 // Path is relative to the resources dir.
 - (id) initWithPath: (NSString*) path;
 - (void) play;
 
 @end
 

