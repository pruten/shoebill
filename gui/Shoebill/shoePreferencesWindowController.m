/*
 * Copyright (c) 2014, Peter Rutenbar <pruten@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#import "shoePreferencesWindowController.h"
#import "shoeApplication.h"

@implementation shoePreferencesWindowController

- (id) init
{
    return [super initWithWindowNibName:@"shoePreferencesWindowController"];
}

- (void)windowDidLoad
{
    NSUserDefaults *defaults = [NSUserDefaults standardUserDefaults];
    
    NSString *rootKernelPathStr = [defaults objectForKey:@"rootKernelPath"];
    NSString *romPathStr = [defaults objectForKey:@"romPath"];
    NSInteger verboseState = [defaults integerForKey:@"verboseState"];
    NSInteger memsize = [defaults integerForKey:@"memorySize"];
    
    if ((memsize < 1) || (memsize > 1024)) {
        memsize = 8;
        [defaults setInteger:memsize forKey:@"memorySize"];
    }
    
    verboseState = (verboseState == NSOnState) ? NSOnState : NSOffState;
    [defaults setInteger:verboseState forKey:@"verboseState"];
    
    NSString *scsiPath0Str = [defaults objectForKey:@"scsiPath0"];
    NSString *scsiPath1Str = [defaults objectForKey:@"scsiPath1"];
    NSString *scsiPath2Str = [defaults objectForKey:@"scsiPath2"];
    NSString *scsiPath3Str = [defaults objectForKey:@"scsiPath3"];
    NSString *scsiPath4Str = [defaults objectForKey:@"scsiPath4"];
    NSString *scsiPath5Str = [defaults objectForKey:@"scsiPath5"];
    NSString *scsiPath6Str = [defaults objectForKey:@"scsiPath6"];

    if (romPath) [romPath setStringValue:romPathStr];
    if (kernelPath) [kernelPath setStringValue:rootKernelPathStr];
    [verbose setState:verboseState];
    [memorySize setStringValue:[NSString stringWithFormat:@"%u", (uint32_t)memsize]];
    
    if (scsiPath0Str) [scsiPath0 setStringValue:scsiPath0Str];
    if (scsiPath1Str) [scsiPath1 setStringValue:scsiPath1Str];
    if (scsiPath2Str) [scsiPath2 setStringValue:scsiPath2Str];
    if (scsiPath3Str) [scsiPath3 setStringValue:scsiPath3Str];
    if (scsiPath4Str) [scsiPath4 setStringValue:scsiPath4Str];
    if (scsiPath5Str) [scsiPath5 setStringValue:scsiPath5Str];
    if (scsiPath6Str) [scsiPath6 setStringValue:scsiPath6Str];
    
    NSInteger screenHeightValue = [defaults integerForKey:@"screenHeight"];
    NSInteger screenWidthValue = [defaults integerForKey:@"screenWidth"];
    
    if ((screenHeightValue < 342) || (screenHeightValue > 0xffff)) {
        screenHeightValue = 480;
        [defaults setInteger:screenHeightValue forKey:@"screenHeight"];
    }
    
    if ((screenWidthValue < 512) || (screenWidthValue > 0xffff)) {
        screenWidthValue = 640;
        [defaults setInteger:screenWidthValue forKey:@"screenWidth"];
    }
    
    [screenWidth setStringValue:[NSString stringWithFormat:@"%u", (uint32_t)screenWidthValue]];
    [screenHeight setStringValue:[NSString stringWithFormat:@"%u", (uint32_t)screenHeightValue]];
    
    [defaults synchronize];
}


- (IBAction)browsePressed:(id)sender
{
    NSOpenPanel* openPanel = [NSOpenPanel openPanel];
    
    [openPanel setCanChooseFiles:YES];
    [openPanel setAllowsMultipleSelection:NO];
    
    if ([openPanel runModal] != NSOKButton)
        return ;
    
    NSArray *urls = [openPanel URLs];
    if ([urls count] != 1)
        return ;
    
    NSURL *url = [urls objectAtIndex:0];
    if (![url isFileURL])
        return ;
    
    NSString *buttonID = [sender identifier];
    NSTextField *field;
    
    if ([buttonID isEqualToString:@"romPathBrowse"])
        field = romPath;
    else if ([buttonID isEqualToString:@"scsiPath0Browse"])
        field = scsiPath0;
    else if ([buttonID isEqualToString:@"scsiPath1Browse"])
        field = scsiPath1;
    else if ([buttonID isEqualToString:@"scsiPath2Browse"])
        field = scsiPath2;
    else if ([buttonID isEqualToString:@"scsiPath3Browse"])
        field = scsiPath3;
    else if ([buttonID isEqualToString:@"scsiPath4Browse"])
        field = scsiPath4;
    else if ([buttonID isEqualToString:@"scsiPath5Browse"])
        field = scsiPath5;
    else if ([buttonID isEqualToString:@"scsiPath6Browse"])
        field = scsiPath6;
    else
        return ;
    
    [field setStringValue: [url path]];
}

- (IBAction)applyPressed:(id)sender
{
    
    NSString *rootKernelPathStr = [kernelPath stringValue];
    NSString *romPathStr = [romPath stringValue];
    NSInteger verboseState = [verbose state];
    NSInteger memsize = [memorySize integerValue];
    
    NSString *scsiPath0Str = [scsiPath0 stringValue];
    NSString *scsiPath1Str = [scsiPath1 stringValue];
    NSString *scsiPath2Str = [scsiPath2 stringValue];
    NSString *scsiPath3Str = [scsiPath3 stringValue];
    NSString *scsiPath4Str = [scsiPath4 stringValue];
    NSString *scsiPath5Str = [scsiPath5 stringValue];
    NSString *scsiPath6Str = [scsiPath6 stringValue];
    
    NSInteger screenHeightValue = [screenHeight integerValue];
    NSInteger screenWidthValue = [screenWidth integerValue];
    
    NSUserDefaults *defaults = [NSUserDefaults standardUserDefaults];
    
    [defaults setObject:rootKernelPathStr forKey:@"rootKernelPath"];
    [defaults setObject:romPathStr forKey:@"romPath"];
    [defaults setInteger:verboseState forKey:@"verboseState"];
    [defaults setInteger:memsize forKey:@"memorySize"];
    
    [defaults setObject:scsiPath0Str forKey:@"scsiPath0"];
    [defaults setObject:scsiPath1Str forKey:@"scsiPath1"];
    [defaults setObject:scsiPath2Str forKey:@"scsiPath2"];
    [defaults setObject:scsiPath3Str forKey:@"scsiPath3"];
    [defaults setObject:scsiPath4Str forKey:@"scsiPath4"];
    [defaults setObject:scsiPath5Str forKey:@"scsiPath5"];
    [defaults setObject:scsiPath6Str forKey:@"scsiPath6"];
    
    [defaults setInteger:screenHeightValue forKey:@"screenHeight"];
    [defaults setInteger:screenWidthValue forKey:@"screenWidth"];
    
    [defaults synchronize];
}

- (IBAction)cancelPressed:(id)sender
{
    [[self window] close];
}
- (IBAction)applyAndRunPressed:(id)sender
{
    shoeApplication *shoeApp = (shoeApplication*) NSApp;
    [self applyPressed:sender];
    [shoeApp startEmulator];
    [[self window] close];
}

@end
