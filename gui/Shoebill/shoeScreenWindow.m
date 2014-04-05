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

#import "shoeScreenWindow.h"
#import "shoeApplication.h"
#import "shoeAppDelegate.h"

@implementation shoeScreenWindow

- (void)configure:(uint8_t) _slotnum
{
    slotnum = _slotnum;
    
    shoeApplication *shoeApp = (shoeApplication*) NSApp;
    shoebill_control_t *control = &shoeApp->control;
    shoebill_card_video_t *video = &control->slots[slotnum].card.video;
    NSSize size = {
        .height=video->height,
        .width=video->width
    };
    
     [self setContentSize:size];
}

// Called after all the shoeScreenWindows are created and configured,
// because one of them was already made key while isRunning==NO,
// so -becomeKeyWindow didn't set doCapture=YES
- (void)reevaluateKeyWindowness
{
    shoeApplication *shoeApp = (shoeApplication*)NSApp;
    
    assert(shoeApp->isRunning);
    
    if ([self isKeyWindow]) {
        shoeApp->doCaptureKeys = YES;
    }
    else {
        shoeApp->doCaptureKeys = NO;
        [self uncaptureMouse];
    }
}

- (void)becomeKeyWindow
{
    shoeApplication *shoeApp = (shoeApplication*)NSApp;
    
    if (shoeApp->isRunning) {
        shoeApp->doCaptureKeys = YES;
    }
    
    [super becomeKeyWindow];
}

- (void)resignKeyWindow
{
    shoeApplication *shoeApp = (shoeApplication*)NSApp;
    
    if (shoeApp->isRunning) {
        shoeApp->doCaptureKeys = NO;
        [self uncaptureMouse];
    }
    
    [super resignKeyWindow];
}

- (void) warpToCenter
{
    // Convert the cocoa window frame to quartz global coordinates
    NSRect winrect = [self frame];
    NSScreen *mainScreen = (NSScreen*)[[NSScreen screens] objectAtIndex:0];
    winrect.origin.y = NSMaxY([mainScreen frame]) - NSMaxY(winrect);
    CGRect cgwinrect = NSRectToCGRect(winrect);
    
    // Find the center of the window
    cgwinrect.origin.x += cgwinrect.size.width / 2.0;
    cgwinrect.origin.y += cgwinrect.size.height / 2.0;
    
    CGWarpMouseCursorPosition(cgwinrect.origin);
}

- (void) uncaptureMouse
{
    shoeApplication *shoeApp = (shoeApplication*)NSApp;
    shoeApp->doCaptureMouse = NO;
    CGDisplayShowCursor(0);
    [self setTitle:@"Shoebill - Screen 1"];
}

- (void) captureMouse
{
    shoeApplication *shoeApp = (shoeApplication*)NSApp;
    shoeApp->doCaptureMouse = YES;
    CGDisplayHideCursor(0);
    [self warpToCenter];
    [self setTitle:@"Shoebill - Screen 1 (Ctrl-click to escape)"];
}


@end
