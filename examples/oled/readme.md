This example implements an OLED display with framebuffer located in RAM.

The OLED is 16-bit color at a size of 128 x 128 pixel using a SSD1351 controller.

uGUI is used for simple drawing routines and can be used up to complex interfaces
with buttons, windows, pup-ups, etc.

Connecting a waveshare 1.5" Full-color OLED display to AMAPHEVB (Apollo2 EVB):
- SCK (Display Clock)        -> 39
- DIN (Display Data-IN)      -> 44
- CS  (Dsiplay Chipselect)   -> 17
- DC  (Display Data/Command) -> 14
- RST (Display Reset)        -> 15

used IOM:                  IOM4
used SPI frequency         24MHz

The example is using uGUI v0.3 under following license:
```
/* -------------------------------------------------------------------------------- */
/* -- µGUI - Generic GUI module (C)Achim Döbler, 2015                            -- */
/* -------------------------------------------------------------------------------- */
// µGUI is a generic GUI module for embedded systems.
// This is a free software that is open for education, research and commercial
// developments under license policy of following terms.
//
//  Copyright (C) 2015, Achim Döbler, all rights reserved.
//  URL: http://www.embeddedlightning.com/
//
// * The µGUI module is a free software and there is NO WARRANTY.
// * No restriction on use. You can use, modify and redistribute it for
//   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
// * Redistributions of source code must retain the above copyright notice.
```
 
