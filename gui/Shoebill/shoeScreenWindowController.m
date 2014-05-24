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

#import "shoeScreenWindowController.h"
#import "shoeApplication.h"

@interface shoeScreenWindowController ()

@end

@implementation shoeScreenWindowController

- (id)initWithWindowNibName:(NSString *)windowNibName
                    slotnum:(uint8_t)_slotnum
{
    shoeScreenWindowController *result = [super initWithWindowNibName:windowNibName];
    result->slotnum = _slotnum;
    return result;
}

- (void)windowDidLoad
{
    [super windowDidLoad];
    
    shoebill_video_frame_info_t frame = shoebill_get_video_frame(slotnum, 1);
    NSSize size = {
        .height=frame.height,
        .width=frame.width,
    };
    
    [[self window] setContentSize:size];
}

- (NSApplicationPresentationOptions)window:(NSWindow *)window
      willUseFullScreenPresentationOptions:(NSApplicationPresentationOptions)proposedOptions
{

    return (NSApplicationPresentationFullScreen |       // support full screen for this window (required)
            NSApplicationPresentationHideDock |         // completely hide the dock
            NSApplicationPresentationAutoHideMenuBar);  // yes we want the menu bar to show/hide
}

- (NSSize)window:(NSWindow *)window willUseFullScreenContentSize:(NSSize)proposedSize
{
    shoebill_video_frame_info_t frame = shoebill_get_video_frame(slotnum, 1);
    NSSize size = {
        .height=frame.height,
        .width=frame.width,
    };
    
    return size;
}

@end
