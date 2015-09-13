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
#include <ctype.h>

@implementation shoePreferencesWindowController

- (id) init
{
    return [super initWithWindowNibName:@"shoePreferencesWindowController"];
}

- (void)windowDidLoad
{
    uint32_t i;
    NSTextField *screenWidths[4] = {
        screenWidth1, screenWidth2, screenWidth3, screenWidth4};
    NSTextField *screenHeights[4] = {
        screenHeight1, screenHeight2, screenHeight3, screenHeight4};
    NSButton *screenEnableds[4] = {
        enableScreen1, enableScreen2, enableScreen3, enableScreen4};
    
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
    
    for (i=0; i<4; i++) {
        NSInteger height = [defaults integerForKey:[NSString stringWithFormat:@"screenHeight%u", i]];
        NSInteger width = [defaults integerForKey:[NSString stringWithFormat:@"screenWidth%u", i]];
        NSInteger enabled = [defaults integerForKey:[NSString stringWithFormat:@"screenEnabled%u", i]];
        
        if ((height < 342) || (height > 0xffff))
            height = 480;
        if ((width < 342) || (width > 0xffff))
            width = 640;
        
        [screenHeights[i] setStringValue:[NSString stringWithFormat:@"%u", (uint32_t)height]];
        [screenWidths[i] setStringValue:[NSString stringWithFormat:@"%u", (uint32_t)width]];
        [screenEnableds[i] setState:enabled];
    }

    NSString *tapPathStr = [defaults objectForKey:@"tapPathE"];
    NSString *macAddressStr = [defaults objectForKey:@"macAddressE"];
    NSInteger ethernetEnabledState = [defaults integerForKey:@"ethernetEnabledE"];
    
    [tapPath setStringValue:tapPathStr];
    [macAddress setStringValue:macAddressStr];
    [ethernetEnabled setIntegerValue:ethernetEnabledState];
    
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

- (void) complain:(NSString*)str
{
    NSAlert *theAlert = [NSAlert
                         alertWithMessageText:nil
                         defaultButton:nil
                         alternateButton:nil
                         otherButton:nil
                         informativeTextWithFormat:@"%@", str
                         ];
    [theAlert runModal];
}

- (IBAction)applyPressed:(id)sender
{
    uint32_t i;
    NSTextField *screenWidths[4] = {
        screenWidth1, screenWidth2, screenWidth3, screenWidth4};
    NSTextField *screenHeights[4] = {
        screenHeight1, screenHeight2, screenHeight3, screenHeight4};
    NSButton *screenEnableds[4] = {
        enableScreen1, enableScreen2, enableScreen3, enableScreen4};
    
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
    
    NSString *macAddressStr = [macAddress stringValue];
    NSString *tapPathStr = [tapPath stringValue];
    NSInteger ethernetEnabledState = [ethernetEnabled state];
    
    NSUserDefaults *defaults = [NSUserDefaults standardUserDefaults];
    
    uint8_t mac[6];
    if (!parseMACAddr ([macAddressStr UTF8String], mac)) {
        [self complain:@"Bad MAC address"];
    }
    else {
        [macAddress setStringValue:[NSString stringWithFormat:@"%02X:%02X:%02X:%02X:%02X:%02X",
                                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]]];
    }
    
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
    
    for (i=0; i<4; i++) {
        NSInteger height = [screenHeights[i] integerValue];
        NSInteger width = [screenWidths[i] integerValue];
        NSInteger enabled = [screenEnableds[i] state];

        if ((height < 342) || (height > 0xffff))
            height = 480;
        if ((width < 342) || (width > 0xffff))
            width = 640;
        
        [defaults setInteger:height forKey:[NSString stringWithFormat:@"screenHeight%u", i]];
        [defaults setInteger:width forKey:[NSString stringWithFormat:@"screenWidth%u", i]];
        [defaults setInteger:enabled forKey:[NSString stringWithFormat:@"screenEnabled%u", i]];
    }
    
    [defaults setObject:macAddressStr forKey:@"macAddressE"];
    [defaults setObject:tapPathStr forKey:@"tapPathE"];
    [defaults setInteger:ethernetEnabledState forKey:@"ethernetEnabledE"];
    
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

-(IBAction)zapPramPressed:(id)sender
{
    shoeApplication *shoeApp = (shoeApplication*) NSApp;
    [shoeApp zapPram:[NSUserDefaults standardUserDefaults] ptr:nil];
}

void generateMACAddr (uint8_t *mac)
{
    srandom((unsigned)(random() ^ time(NULL)));
    
    /* Generate a MAC address in the range of the original EtherTalk card */
    
    mac[0] = 0x02;
    mac[1] = 0x60;
    mac[2] = 0x8c;
    mac[3] = random() & 0x07;
    mac[4] = random() & 0xff;
    mac[5] = random() & 0xff;
}

_Bool parseMACAddr (const char *str, uint8_t *mac)
{
    uint32_t i, nibbles = 0;
    uint8_t allowed[256];
    
    memset(allowed, 30, 256);
    for (i=0; i<256; i++)
        if (isspace(i))
            allowed[i] = 20;
    allowed[':'] = 20;
    allowed['-'] = 20;
    for (i=0; i<10; i++)
        allowed['0' + i] = i;
    for (i=0; i<6; i++) {
        allowed['a' + i] = 10 + i;
        allowed['A' + i] = 10 + i;
    }
    
    for (i=0; str[i]; i++) {
        const uint8_t v = allowed[str[i]];
        
        if (v == 30)
            return 0;
        else if (v == 20)
            continue;

        if (nibbles >= 12)
            return 0;
        mac[nibbles/2] <<= 4;
        mac[nibbles/2] |= v;
        nibbles++;
    }
    return (nibbles == 12);
}

-(IBAction)newMacAddrPressed:(id)sender
{
    uint8_t mac[6];
    
    generateMACAddr(mac);
    
    [macAddress setStringValue:[NSString stringWithFormat:@"%02X:%02X:%02X:%02X:%02X:%02X",
                                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]]];
}

@end
