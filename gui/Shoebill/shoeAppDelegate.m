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

#import "shoeAppDelegate.h"
#import "shoeApplication.h"
#import "shoePreferencesWindowController.h"

@implementation shoeAppDelegate

- (void)createFirstTimeUserDefaults
{
    NSUserDefaults *defaults = [NSUserDefaults standardUserDefaults];
    uint32_t i;
    
    [((shoeApplication*)NSApp) zapPram:defaults ptr:nil];
    [defaults setObject:@"/unix" forKey:@"rootKernelPath"];
    [defaults setObject:@"" forKey:@"romPath"];
    [defaults setInteger:NSOnState forKey:@"verboseState"];
    [defaults setInteger:16 forKey:@"memorySize"];
    
    // [defaults setInteger:640 forKey:@"screenWidth"];
    // [defaults setInteger:480 forKey:@"screenHeight"];
    
    for (i=0; i<7; i++)
        [defaults setObject:@"" forKey:[NSString stringWithFormat:@"scsiPath%u", i]];
    
    [defaults setBool:YES forKey:@"defaultsInitialized"];
}

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification
{
    uint32_t i;
    NSUserDefaults *defaults = [NSUserDefaults standardUserDefaults];
    
    BOOL isInitialized = [defaults boolForKey:@"defaultsInitialized"];
    
    if (!isInitialized)
        [self createFirstTimeUserDefaults];
    
    // < 0.0.2 leaves rootKernelPath uninitialized
    if ([defaults objectForKey:@"rootKernelPath"] == nil)
        [defaults setObject:@"/unix" forKey:@"rootKernelPath"];
    
    // < 0.0.3 leaves pramData uninitialized
    if ([defaults objectForKey:@"pramData"] == nil)
        [((shoeApplication*)NSApp) zapPram:defaults ptr:nil];
    
    // < 0.0.5 leaves ethernet settings uninitialized
    if ([defaults objectForKey:@"tapPathE"] == nil) {
        uint8_t mac[6];
        generateMACAddr(mac);
        [defaults setObject:[NSString stringWithFormat:@"%02X:%02X:%02X:%02X:%02X:%02X",
                             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]]
                     forKey:@"macAddressE"];
        [defaults setObject:@"/dev/tap0" forKey:@"tapPathE"];
        [defaults setInteger:0 forKey:@"ethernetEnabledE"];
        
        for (i=0; i<4; i++) {
            [defaults setInteger:640 forKey:[NSString stringWithFormat:@"screenWidth%u", i]];
            [defaults setInteger:480 forKey:[NSString stringWithFormat:@"screenHeight%u", i]];
            [defaults setInteger:1 forKey:[NSString stringWithFormat:@"screenEnabled%u", i]];
        }
        [defaults setInteger:1 forKey:@"screenEnabled0"];
    }
    
    [defaults synchronize];
}



@end
