//
//  UseSegmentedControlViewController.m
//  testAlgo
//
//  Created by Lior Neu-ner on 2013/05/14.
//  Copyright (c) 2013 itaidddd@gmail.com. All rights reserved.
//

#import "UseSegmentedControlViewController.h"

@implementation UseSegmentedControlViewController
UILabel *label;
- (void)viewDidLoad {
    [super viewDidLoad];
    //Create label
    label = [[UILabel alloc] init];
    label.frame = CGRectMake(10, 10, 300, 40);
    label.textAlignment = UITextAlignmentCenter;
    [self.view addSubview:label];
    //Create the segmented control
    NSArray *itemArray = [NSArray arrayWithObjects: @"One", @"Two", @"Three", nil];
    UISegmentedControl *segmentedControl = [[UISegmentedControl alloc] initWithItems:itemArray];
    segmentedControl.frame = CGRectMake(35, 200, 250, 50);
    segmentedControl.segmentedControlStyle = UISegmentedControlStylePlain;
    segmentedControl.selectedSegmentIndex = 1;
    [segmentedControl addTarget:self
                         action:@selector(pickOne:)
               forControlEvents:UIControlEventValueChanged];
    [self.view addSubview:segmentedControl];
    [segmentedControl release];
}
//Action method executes when user touches the button
- (void) pickOne:(id)sender{
    UISegmentedControl *segmentedControl = (UISegmentedControl *)sender;
    label.text = [segmentedControl titleForSegmentAtIndex: [segmentedControl selectedSegmentIndex]];
}
- (void)dealloc {
    [label release];
    [super dealloc];
}

@end
