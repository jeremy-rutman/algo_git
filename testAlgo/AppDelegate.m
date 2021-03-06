//
//  AppDelegate.m
//  testAlgo
//
//  Created by Itai David on 8/14/12.
//  Copyright (c) 2012 itaidddd@gmail.com. All rights reserved.
//

#import "AppDelegate.h"

#import "ViewController.h"

#import <mach/mach.h>
#import <mach/mach_host.h>

//#import "UIDeviceAdditions.h"
#include <sys/sysctl.h>
//#include <mach/mach.h>


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
  //      long i=57;
        while(TRUE)
        {
    //        NSLog(@"BGTime left1: %f", [UIApplication sharedApplication].backgroundTimeRemaining);
            //NSLog(@"loop %ld",i++);//NSLog(@"ITAI BG");
            //if (--i==10)
            //else if (i==0)
            //    i=60;
            //if (--i==0)
            //{
            //    i=30;
            //    [locationManager startUpdatingLocation];
            //    [locationManager stopUpdatingLocation];
            //}
            //while (i--)
            //{
        //        [NSThread sleepForTimeInterval:570];
            [NSThread sleepForTimeInterval:570];
                NSLog(@"BGTime left2: %f", [UIApplication sharedApplication].backgroundTimeRemaining);
//            if (Gps_state == NO)
            if(YES)
            {
                //if (AGps_state)
                //    [locationManager stopMonitoringSignificantLocationChanges];
                [locationManager startUpdatingLocation];
                [locationManager stopUpdatingLocation];
                NSLog(@"********************************");
                NSLog(@"********************************");
                NSLog(@"********************************");
                NSLog(@"did location manager start/stop");
                NSLog(@"did location manager start/stop");
               NSLog(@"did location manager start/stop");
                NSLog(@"********************************");
                NSLog(@"********************************");
                NSLog(@"********************************");
                
      //          NSLog(@"memory used %f free %d",[self currentMemoryUsage],[self get_free_memory]) ;
          //      NSLog(@"free %d",[self get_free_memory]) ;
               NSLog(@"used memory %f",[self currentMemoryUsage]) ;
                
                //if (AGps_state)
                //    [locationManager startMonitoringSignificantLocationChanges];
            }
            //}
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
