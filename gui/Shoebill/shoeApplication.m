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

#import "shoeApplication.h"
#import "shoeScreenWindow.h"

@implementation shoeApplication

#define mapkeymod(u, a, m) do { \
    assert((a >> 7) == 0); \
    void *value = (void*)(((m) << 8)| (a)); \
    rb_insert(keymap, u, value, NULL); \
} while (0) \

#define mapkey(_u, a) mapkeymod(_u, a, 0)

- (void)initKeyboardMap
{
    keymap = rb_new();
    
    // Letters
    mapkey('a', 0x00);
    mapkey('b', 0x0b);
    mapkey('c', 0x08);
    mapkey('d', 0x02);
    mapkey('e', 0x0e);
    mapkey('f', 0x03);
    mapkey('g', 0x05);
    mapkey('h', 0x04);
    mapkey('i', 0x22);
    mapkey('j', 0x26);
    mapkey('k', 0x28);
    mapkey('l', 0x25);
    mapkey('m', 0x2e);
    mapkey('n', 0x2d);
    mapkey('o', 0x1f);
    mapkey('p', 0x23);
    mapkey('q', 0x0c);
    mapkey('r', 0x0f);
    mapkey('s', 0x01);
    mapkey('t', 0x11);
    mapkey('u', 0x20);
    mapkey('v', 0x09);
    mapkey('w', 0x0d);
    mapkey('x', 0x07);
    mapkey('y', 0x10);
    mapkey('z', 0x06);
    
    // Numbers
    mapkey('0', 0x1d);
    mapkey('1', 0x12);
    mapkey('2', 0x13);
    mapkey('3', 0x14);
    mapkey('4', 0x15);
    mapkey('5', 0x17);
    mapkey('6', 0x16);
    mapkey('7', 0x1a);
    mapkey('8', 0x1c);
    mapkey('9', 0x19);
    
    // Top row symbols
    mapkeymod(')', 0x1d, modShift);
    mapkeymod('!', 0x12, modShift);
    mapkeymod('@', 0x13, modShift);
    mapkeymod('#', 0x14, modShift);
    mapkeymod('$', 0x15, modShift);
    mapkeymod('%', 0x17, modShift);
    mapkeymod('^', 0x16, modShift);
    mapkeymod('&', 0x1a, modShift);
    mapkeymod('*', 0x1c, modShift);
    mapkeymod('(', 0x19, modShift);
    
    // Other symbols (no shift)
    mapkeymod('`', 0x32, 0);
    mapkeymod('-', 0x1b, 0);
    mapkeymod('=', 0x18, 0);
    mapkeymod('[', 0x21, 0);
    mapkeymod(']', 0x1e, 0);
    mapkeymod('\\', 0x2a, 0);
    mapkeymod(';', 0x29, 0);
    mapkeymod('\'', 0x27, 0);
    mapkeymod(',', 0x2b, 0);
    mapkeymod('.', 0x2f, 0);
    mapkeymod('/', 0x2c, 0);
    
    // Other symbols (with shift)
    mapkeymod('~', 0x32, modShift);
    mapkeymod('_', 0x1b, modShift);
    mapkeymod('+', 0x18, modShift);
    mapkeymod('{', 0x21, modShift);
    mapkeymod('}', 0x1e, modShift);
    mapkeymod('|', 0x2a, modShift);
    mapkeymod(':', 0x29, modShift);
    mapkeymod('"', 0x27, modShift);
    mapkeymod('<', 0x2b, modShift);
    mapkeymod('>', 0x2f, modShift);
    mapkeymod('?', 0x2c, modShift);
    
    // Function keys
    mapkey(NSF1FunctionKey, 0x7a);
    mapkey(NSF2FunctionKey, 0x78);
    mapkey(NSF3FunctionKey, 0x63);
    mapkey(NSF4FunctionKey, 0x76);
    mapkey(NSF5FunctionKey, 0x60);
    mapkey(NSF6FunctionKey, 0x61);
    mapkey(NSF7FunctionKey, 0x62);
    mapkey(NSF8FunctionKey, 0x64);
    mapkey(NSF9FunctionKey, 0x65);
    mapkey(NSF10FunctionKey, 0x6d);
    mapkey(NSF11FunctionKey, 0x67);
    mapkey(NSF12FunctionKey, 0x6f);
    mapkey(NSF13FunctionKey, 0x69);
    mapkey(NSF14FunctionKey, 0x6b);
    mapkey(NSF15FunctionKey, 0x71);
    
    // Arrows
    mapkey(NSUpArrowFunctionKey, 0x3e);
    mapkey(NSDownArrowFunctionKey, 0x3d);
    mapkey(NSRightArrowFunctionKey, 0x3c);
    mapkey(NSLeftArrowFunctionKey, 0x3b);
    
    // Delete
    mapkey(NSDeleteFunctionKey, 0x75);
    mapkey(NSBackspaceCharacter, 0x33);
    mapkey(NSDeleteCharacter, 0x33);
    
    // Enter, NL, CR
    mapkey(NSCarriageReturnCharacter, 0x24);
    mapkey(NSNewlineCharacter, 0x24);
    mapkey(NSEnterCharacter, 0x24);
    
    // Other keys
    mapkey(0x1b, 0x35); // escape
    mapkey(' ', 0x31); // space
    mapkey(NSTabCharacter, 0x30); // tab
}

- (void)sendEvent:(NSEvent *)event
{
    if (doCaptureKeys) {
        assert(isRunning);
        NSEventType type = [event type];
        if (type == NSFlagsChanged) {
            NSUInteger modifierFlags = [event modifierFlags];
            shoebill_key_modifier(modifierFlags >> 16);
            
            // Block any key-related event while the command key is down
            if (modifierFlags & NSCommandKeyMask)
                return ;
            
            [super sendEvent:event];
        }
        else if (type == NSKeyDown || type == NSKeyUp) {
            NSString *chars = [[event charactersIgnoringModifiers] lowercaseString];
            NSUInteger modifierFlags = [event modifierFlags];
            unichar c = [chars characterAtIndex:0];
            void *_value;
            
            if (keymap == NULL)
                [self initKeyboardMap];
            
            if (rb_find(keymap, c, &_value)) {
                uint16_t value = (uint16_t)_value;
                shoebill_key_modifier((value >> 8) | (modifierFlags >> 16));
                shoebill_key((type == NSKeyDown), value & 0xff);
                
            }
            
            // Block any key-related event while the command key is down
            if (modifierFlags & NSCommandKeyMask)
                return ;
            
            [super sendEvent:event];
            
        }
    }
    
    [super sendEvent:event];
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


- (BOOL) fetchUserDefaults:(uint16_t*)height width:(uint16_t*)width
{
    uint32_t i;
    NSUserDefaults *defaults = [NSUserDefaults standardUserDefaults];
    
    NSString *kernelPathStr = [defaults objectForKey:@"kernelPath"];
    NSString *romPathStr = [defaults objectForKey:@"romPath"];
    NSInteger verboseState = [defaults integerForKey:@"verboseState"];
    NSInteger memsize = [defaults integerForKey:@"memorySize"];
    
    if (kernelPathStr == Nil || [kernelPathStr length]==0) {
        [self complain:@"Kernel path invalid!"];
        return NO;
    }
    
    else if (romPathStr == Nil || [romPathStr length] == 0) {
        [self complain:@"ROM path invalid!"];
        return NO;
    }
    
    if ((memsize < 1) || (memsize > 1024))
        memsize = 8;
    
    NSInteger screenHeightValue = [defaults integerForKey:@"screenHeight"];
    NSInteger screenWidthValue = [defaults integerForKey:@"screenWidth"];
    
    if ((screenHeightValue < 342) || (screenHeightValue > 0xffff))
        screenHeightValue = 480;
    
    if ((screenWidthValue < 512) || (screenWidthValue > 0xffff))
        screenWidthValue = 640;
    
    
    for (i=0; i<7; i++) {
        NSString *str = [defaults objectForKey:[NSString stringWithFormat:@"scsiPath%u", i]];
        if (str == nil || [str length] == 0)
            control.scsi_devices[i].path = NULL;
        else
            control.scsi_devices[i].path = strdup([str UTF8String]);
        
    }
    
    char *kernelPathCString = strdup([kernelPathStr UTF8String]);
    char *romPathCString = strdup([romPathStr UTF8String]);
    
    // FIXME: I'm leaking these strings. Stop leaking stuff when the UI is more finalized
    
    control.aux_verbose = (verboseState == NSOnState);
    control.ram_size = (uint32_t)memsize * 1024 * 1024;
    control.aux_kernel_path = kernelPathCString;
    control.rom_path = romPathCString;
    
    *width = screenWidthValue;
    *height = screenHeightValue;
    
    return YES;
}

- (void) createScreenWindow:(uint8_t)slotnum
                     height:(uint16_t)height
                      width:(uint16_t)width
               refresh_freq:(double)refresh_freq
{
    shoebill_install_video_card(&control,
                                slotnum,
                                width,
                                height,
                                refresh_freq);
    
    windowController[slotnum] = [[NSWindowController alloc] initWithWindowNibName:@"shoeScreenView"];
    shoeScreenWindow *win = (shoeScreenWindow*)windowController[slotnum].window;
    [win configure:slotnum];
}

- (void) startEmulator
{
    if (isRunning)
        return;
    
    uint16_t width, height;
    uint32_t i;
    
    bzero(&control, sizeof(shoebill_control_t));
    
    [self fetchUserDefaults:&height width:&width];
    
    uint32_t result = shoebill_initialize(&control);
    
    if (!result) {
        [self complain:[NSString stringWithFormat:@"%s", control.error_msg]];
        return ;
    }
    
    [self createScreenWindow:9 height:height width:width refresh_freq:200.0/3.0];
    /*[self createScreenWindow:10 height:height width:width refresh_freq:200.0/3.0];
    [self createScreenWindow:11 height:height width:width refresh_freq:200.0/3.0];
    [self createScreenWindow:12 height:height width:width refresh_freq:200.0/3.0];
    [self createScreenWindow:13 height:height width:width refresh_freq:200.0/3.0];
    [self createScreenWindow:14 height:height width:width refresh_freq:200.0/3.0];*/
    
    shoebill_start();
    isRunning = true;
    
    for (i=0; i<16; i++)
        if (windowController[i]) {
            shoeScreenWindow *win = (shoeScreenWindow*)[windowController[i] window];
            [win reevaluateKeyWindowness];
        }

}

- (IBAction)runMenuItem:(id)sender
{
    [self startEmulator];
}


@end
