// ==================================================================
// We need to derive our own MTKView to capture mouse & keyboard input
// ==================================================================

@import MetalKit;
@class MetalRenderer;

@interface MetalMTKView : MTKView
- (void)setRenderer:(MetalRenderer*)r;
- (void)keyDown:(NSEvent *)theEvent;
- (BOOL)acceptsFirstResponder;
@end
