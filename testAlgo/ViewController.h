//#define NO_IPHONE_ATTACHED
//#define DEBUGJ
//#define DEBUGV
#define DEBUGLOG
//#define READ_FILE
#define filetoread @"050613.txt"

//  ViewController.h
//  testAlgo
//  Created by Itai David on 8/14/12.
//  Copyright (c) 2012 itaidddd@gmail.com. All rights reserved.

#import <UIKit/UIKit.h>
#import <CoreLocation/CoreLocation.h>
#import <CoreMotion/CoreMotion.h>
#import "MainService.h"
#import <Foundation/Foundation.h>
#import "drivingDetector.h"

UIBackgroundTaskIdentifier bgTask;
CLLocationManager *locationManager;
CMMotionManager *motionManager;
Boolean Gps_state;
Boolean AGps_state;

@interface ViewController : UIViewController < CLLocationManagerDelegate >{
    
    AlgoDetection * algoDetection;
    AlgoVector * accVector;

    AlgoLocVector * algoLocVector;
    drivingDetector *myDrivingDetector;
    
 //   IBOutlet UIButton *my_very_first_button;
    
    
//    IBOutlet UILabel *j_label;
    
 //   IBOutlet UITextField *j_textfield;
    
    
}

@property (copy, nonatomic) NSString *userName;

@property (strong, nonatomic) IBOutlet UILabel *resultLabel0;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel1;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel2;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel3;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel4;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel5;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel6;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel7;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel8;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel9;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel10;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel11;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel12;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel13;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel14;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel15;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel16;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel17;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel18;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel19;
@property (strong, nonatomic) IBOutlet UILabel *resultLabel20;
@property (strong, nonatomic) IBOutlet UILabel *resultLabelV2x;
@property (strong, nonatomic) IBOutlet UILabel *resultLabelV2y;
@property (strong, nonatomic) IBOutlet UILabel *resultLabelV2z;
@property (strong, nonatomic) IBOutlet UILabel *resultLabelV2h;
@property (strong, nonatomic) IBOutlet UILabel *resultLabelV5x;
@property (strong, nonatomic) IBOutlet UILabel *resultLabelV5y;
@property (strong, nonatomic) IBOutlet UILabel *resultLabelV5z;
@property (strong, nonatomic) IBOutlet UILabel *resultLabelV5h;
@property (strong, nonatomic) IBOutlet UILabel *resultLabelV10x;
@property (strong, nonatomic) IBOutlet UILabel *resultLabelV10y;
@property (strong, nonatomic) IBOutlet UILabel *resultLabelV10z;
@property (strong, nonatomic) IBOutlet UILabel *resultLabelV10h;
@property (strong, nonatomic) IBOutlet UILabel *resultLabeldOx_dt;




//Method writes a string to a text file
//-(void) writeToLogFile:(NSString*)aString aFileName:(NSString *)filename;
//-(void)viewDidLoad;
//- (void)viewDidUnload;
//- (void) segmentAction:(id)sender;
//-(void) displayContent:(NSString*)fName;
- (IBAction)butttonDIDpressed:(id)sender;

    //+(void) displayContent;

//-(NSString *) aString;


//- (void)viewDidLoad2 ;
//- (void) pickOne:(id)sender;
//- (void)dealloc ;

@end


/*
 @interface Sound : NSObject

// Path is relative to the resources dir.
- (id) initWithPath: (NSString*) path;
- (void) play;

@end
*/
