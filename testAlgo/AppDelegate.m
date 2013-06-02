//
//  AppDelegate.m
//  testAlgo
//
//  Created by Itai David on 8/14/12.
//  Copyright (c) 2012 itaidddd@gmail.com. All rights reserved.
//

#import "AppDelegate.h"

#import "ViewController.h"

@implementation AppDelegate

dispatch_block_t expirationHandler = nil;

@synthesize window = _window;
@synthesize viewController = _viewController;




- (BOOL)application:(UIApplication *)application didFinishLaunchingWithOptions:(NSDictionary *)launchOptions
{
    self.window = [[UIWindow alloc] initWithFrame:[[UIScreen mainScreen] bounds]];
    // Override point for customization after application launch.
    self.viewController = [[ViewController alloc] initWithNibName:@"ViewController" bundle:nil];
    self.window.rootViewController = self.viewController;
    [self.window makeKeyAndVisible];
    return YES;
}

- (void)applicationWillResignActive:(UIApplication *)application
{
    // Sent when the application is about to move from active to inactive state. This can occur for certain types of temporary interruptions (such as an incoming phone call or SMS message) or when the user quits the application and it begins the transition to the background state.
    // Use this method to pause ongoing tasks, disable timers, and throttle down OpenGL ES frame rates. Games should use this method to pause the game.
}

//- (void)applicationDidEnterBackground:(UIApplication *)application
//{
//    // Use this method to release shared resources, save user data, invalidate timers, and store enough application state information to restore your application to its current state in case it is terminated later.
//    // If your application supports background execution, this method is called instead of applicationWillTerminate: when the user quits.
//}
// if the iOS device allows background execution,
// this Handler will be called
/*- (void)backgroundHandler {
 
 NSLog(@"### -->VOIP backgrounding callback");
 
 UIApplication*    app = [UIApplication sharedApplication];
 
 bgTask = [app beginBackgroundTaskWithExpirationHandler:^{
 [app endBackgroundTask:bgTask];
 bgTask = UIBackgroundTaskInvalid;
 }];
 
 // Start the long-running task
 dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
 
 while (1) {
 NSLog(@"BGTime left: %f", [UIApplication sharedApplication].backgroundTimeRemaining);
 [self doSomething];
 sleep(1);
 }
 });
 }*/
- (void)applicationDidEnterBackground:(UIApplication *)application {
    
    UIApplication*    app = [UIApplication sharedApplication];
    
    // it's better to move "dispatch_block_t expirationHandler"
    // into your headerfile and initialize the code somewhere else
    // i.e.
    // - (void)applicationDidFinishLaunching:(UIApplication *)application {
    //
    // expirationHandler = ^{ ... } }
    // because your app may crash if you initialize expirationHandler twice.
    if (expirationHandler)
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
//        long i=0;
        while(TRUE)
        {
            NSLog(@"BGTime left: %f", [UIApplication sharedApplication].backgroundTimeRemaining);
            //NSLog(@"loop %ld",i++);
            //NSLog(@"ITAI BG");
            
            //if (--i==10)
            //{
            //
            //}
            //else if (i==0)
            //{
            //    i=60;
            //
            //}
            //if (--i==0)
            //{
            //    i=30;
            //    [locationManager startUpdatingLocation];
            //    [locationManager stopUpdatingLocation];
            //}
            [NSThread sleepForTimeInterval:570];
            if (Gps_state == NO)
            {
                //if (AGps_state)
                //    [locationManager stopMonitoringSignificantLocationChanges];
                [locationManager startUpdatingLocation];
                [locationManager stopUpdatingLocation];
                //if (AGps_state)
                //    [locationManager startMonitoringSignificantLocationChanges];
            }
        }
        // do your background work here
    });
}


- (void)applicationWillEnterForeground:(UIApplication *)application
{
    // Called as part of the transition from the background to the inactive state; here you can undo many of the changes made on entering the background.
}

- (void)applicationDidBecomeActive:(UIApplication *)application
{
    // Restart any tasks that were paused (or not yet started) while the application was inactive. If the application was previously in the background, optionally refresh the user interface.
}

- (void)applicationWillTerminate:(UIApplication *)application
{
    // Called when the application is about to terminate. Save data if appropriate. See also applicationDidEnterBackground:.
}

@end
