// ==================================================================
// Apple Metal Rendering of the Mesh Data
// ==================================================================

@import MetalKit;
@class MetalMTKView;

@interface MetalRenderer : NSObject<MTKViewDelegate>
- (nonnull instancetype)initWithMetalKitView:(nonnull MetalMTKView *)mtkView;
- (void)reGenerate;
@end

