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


@implementation shoeAppDelegate

- (void)createFirstTimeUserDefaults
{
    NSUserDefaults *defaults = [NSUserDefaults standardUserDefaults];
    uint32_t i;
    
    [defaults setObject:@"" forKey:@"kernelPath"];
    [defaults setObject:@"" forKey:@"romPath"];
    [defaults setInteger:NSOffState forKey:@"verboseState"];
    [defaults setInteger:16 forKey:@"memorySize"];
    
    [defaults setInteger:640 forKey:@"screenWidth"];
    [defaults setInteger:480 forKey:@"screenHeight"];
    
    for (i=0; i<7; i++)
        [defaults setObject:@"" forKey:[NSString stringWithFormat:@"scsiPath%u", i]];
    
    [defaults setBool:YES forKey:@"defaultsInitialized"];
}

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification
{
    NSUserDefaults *defaults = [NSUserDefaults standardUserDefaults];
    
    BOOL isInitialized = [defaults boolForKey:@"defaultsInitialized"];
    
    if (!isInitialized)
        [self createFirstTimeUserDefaults];
}



@end
