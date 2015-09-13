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

#import <Cocoa/Cocoa.h>

@interface shoePreferencesWindowController : NSWindowController {

    IBOutlet __weak NSButton *apply, *cancel, *applyAndRun, *verbose, *ethernetEnabled;
    IBOutlet __weak NSTextField *kernelPath, *romPath, *memorySize;
    IBOutlet __weak NSTextField *scsiPath0, *scsiPath1, *scsiPath2, *scsiPath3, *scsiPath4, *scsiPath5, *scsiPath6;
    IBOutlet __weak NSTextField *macAddress, *tapPath;
    
    IBOutlet __weak NSTextField *screenHeight1, *screenWidth1;
    IBOutlet __weak NSTextField *screenHeight2, *screenWidth2;
    IBOutlet __weak NSTextField *screenHeight3, *screenWidth3;
    IBOutlet __weak NSTextField *screenHeight4, *screenWidth4;
    
    IBOutlet __weak NSButton *enableScreen1, *enableScreen2, *enableScreen3, *enableScreen4;
}


- (IBAction)applyPressed:(id)sender;
- (IBAction)cancelPressed:(id)sender;
- (IBAction)applyAndRunPressed:(id)sender;
- (IBAction)browsePressed:(id)sender;
    
@end

void generateMACAddr (uint8_t *mac);
_Bool parseMACAddr (const char *str, uint8_t *mac);