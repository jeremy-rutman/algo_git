//
//  ViewController.h
//  testAlgo
//
//  Created by Itai David on 8/14/12.
//  Copyright (c) 2012 itaidddd@gmail.com. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <CoreLocation/CoreLocation.h>
#import <CoreMotion/CoreMotion.h>
#import "MainService.h"

UIBackgroundTaskIdentifier bgTask;
CLLocationManager *locationManager;
CMMotionManager *motionManager;
Boolean Gps_state;
Boolean AGps_state;

@interface ViewController : UIViewController < CLLocationManagerDelegate >{
    
    AlgoDetection * algoDetection;
    AlgoVector * accVector;
    AlgoVector *accVectorWC;
    AlgoVector *accVectorWC_unbiased;
    AlgoVector *orientationVector;
    AlgoLocVector * algoLocVector;
    
    
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

- (IBAction)butttonDIDpressed:(id)sender;
+(void)writeStringToFile:(NSString*)aString;
//Method writes a string to a text file
+(void) writeToTextFile;
+(void) writeToLogFile:(NSString*)aString;
//+(void) displayContent;

//-(NSString *) aString;


@end

@interface Sound : NSObject

// Path is relative to the resources dir.
- (id) initWithPath: (NSString*) path;
- (void) play;

@end

