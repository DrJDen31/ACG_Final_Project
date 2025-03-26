// ==================================================================
// Apple Metal View Controller
// ==================================================================

@import AppKit;
#define PlatformViewController NSViewController
@import MetalKit;

#import "MetalRenderer.h"
#import "MetalMTKView.h"
#import "meshdata.h"

@interface MetalViewController : PlatformViewController
@end

MetalRenderer* GLOBAL_renderer;

extern MeshData *mesh_data;

extern void PackMesh();
extern void Load();
extern bool DrawPixel();
extern void RadiosityIterate();

void myCFTimerCallback()
{
  if (mesh_data->raytracing_animation) {
    // draw 100 pixels and then refresh the screen and handle any user input
    for (int i = 0; i < 100; i++) {
      if (!DrawPixel()) {
        mesh_data->raytracing_animation = false;
        break;
      }
    }
    PackMesh();
    [GLOBAL_renderer reGenerate];
  }
  
  if (mesh_data->radiosity_animation) {
    RadiosityIterate();
    PackMesh();
    [GLOBAL_renderer reGenerate];
  }
}


@implementation MetalViewController
{
  MetalMTKView *_view;
  MetalRenderer *_renderer;
}


- (void)viewDidLoad
{
  [super viewDidLoad];

    // Set the view to use the default device
    _view = (MetalMTKView *)self.view;
    _view.device = MTLCreateSystemDefaultDevice();
    
    if(!_view.device)
    {
        NSLog(@"Metal is not supported on this device");
        return;
    }

    _renderer = [[MetalRenderer alloc] initWithMetalKitView:_view];
    GLOBAL_renderer = _renderer;
    
    if(!_renderer)
    {
        NSLog(@"Renderer failed initialization");
        return;
    }

    // Initialize our renderer with the view size
    [_renderer mtkView:_view drawableSizeWillChange:_view.drawableSize];

    _view.delegate = _renderer;

    // setup timer for animation loop
    CFRunLoopRef runLoop = CFRunLoopGetCurrent();
    CFRunLoopTimerContext  context = {0, self, NULL, NULL, NULL};
    CFRunLoopTimerRef timer = CFRunLoopTimerCreate(kCFAllocatorDefault, 0.1, 0.001, 0, 0,
                                                   &myCFTimerCallback, &context);
    CFRunLoopAddTimer(runLoop, timer, kCFRunLoopCommonModes);
}

@end
