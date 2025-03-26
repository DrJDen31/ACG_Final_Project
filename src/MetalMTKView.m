// ==================================================================
// We need to derive our own MTKView to capture mouse & keyboard input
// ==================================================================

#import "MetalMTKView.h"
#import "MetalKeys.h"
#import "MetalRenderer.h"
#import "meshdata.h"

extern MeshData *mesh_data;


// =======================
// state variables
// =======================
@implementation MetalMTKView {
  bool shift_pressed;
  bool control_pressed;
  bool option_pressed;
  bool command_pressed;
  float mouse_x;
  float mouse_y;
  MetalRenderer *renderer;
}

- (void) setRenderer:(MetalRenderer*)r {
  renderer = r;
}

- (BOOL)acceptsFirstResponder
{
  return YES;
}

// =======================
// modifier keys
// =======================
- (void) flagsChanged:(NSEvent *) event {
  shift_pressed   = [event modifierFlags] & NSEventModifierFlagShift;
  control_pressed = [event modifierFlags] & NSEventModifierFlagControl;
  option_pressed  = [event modifierFlags] & NSEventModifierFlagOption;
  command_pressed = [event modifierFlags] & NSEventModifierFlagCommand;
}

extern void PackMesh();
extern void Load();
extern void RayTreeActivate();
extern void RayTreeDeactivate();
extern void PhotonMappingTracePhotons();
extern void RadiosityIterate();
extern void RadiositySubdivide();
extern void RadiosityClear();
extern void RaytracerClear();
extern void PhotonMappingClear();

extern void cameraTranslate(float x, float y);
extern void cameraRotate(float x, float y);
extern void cameraZoom(float y);
extern void TraceRay(float x, float y);

// =======================
// regular keys
// =======================
- (void)keyDown:(NSEvent *) event
  {
    switch (event.keyCode) {

    case (KEY_R): case (KEY_G): {
      mesh_data->radiosity_animation = false;
      if (event.keyCode == KEY_R) {
        if (!mesh_data->raytracing_animation) {
          printf ("raytracing animation started, press 'X' to stop\n");
          mesh_data->raytracing_animation = true;
        } else {
          printf ("raytracing animation re-started, press 'X' to stop\n");
        }
        mesh_data->gather_indirect = false;
        RaytracerClear();
      } else {
        if (!mesh_data->raytracing_animation) {
          printf ("photon mapping animation started, press 'X' to stop\n");
          mesh_data->raytracing_animation = true;
        } else {
          printf ("photon mapping animation re-started, press 'X' to stop\n");
        }
        mesh_data->gather_indirect = true;
        RaytracerClear();
      }
      if (mesh_data->width <= mesh_data->height) {
        mesh_data->raytracing_divs_x = 10;
        mesh_data->raytracing_divs_y = 10 * mesh_data->height / (float) (mesh_data->width);
      } else {
        mesh_data->raytracing_divs_x = 10 * mesh_data->width / (float) (mesh_data->height);
        mesh_data->raytracing_divs_y = 10;
      }
      mesh_data->raytracing_x = 0;
      mesh_data->raytracing_y = 0;
      break;
    }

    case (KEY_T): {
      // visualize the ray tree for the pixel at the current mouse position
      RayTreeActivate();
      mesh_data->raytracing_divs_x = -1;
      mesh_data->raytracing_divs_y = -1;

      NSPoint touchPoint = [event locationInWindow];
      mouse_x = touchPoint.x;
      mouse_y = touchPoint.y;
  
      TraceRay(mouse_x,mouse_y);
      RayTreeDeactivate();

      PackMesh();
      [renderer reGenerate];
      break;
    }

    case (KEY_L): {
      // toggle photon rendering
      mesh_data->render_photons = !mesh_data->render_photons;
      PackMesh();
      [renderer reGenerate];
      break;
    }
    case (KEY_D): {
      // toggle photon direction rendering
      mesh_data->render_photon_directions = !mesh_data->render_photon_directions;
      PackMesh();
      [renderer reGenerate];
      break;
    }
    case (KEY_K): {
      // toggle kd tree rendering
      mesh_data->render_kdtree = !mesh_data->render_kdtree;
      PackMesh();
      [renderer reGenerate];
      break;
    }
    case (KEY_P): {
      // trace photons
      PhotonMappingTracePhotons();
      PackMesh();
      [renderer reGenerate];
      break;
    }
      
      // RADIOSITY STUFF
    case (KEY_SPACE): {
       // a single step of radiosity
      RadiosityIterate();
      PackMesh();
      [renderer reGenerate];
      break;
    }
    case (KEY_A): {
      // animate radiosity solution
      mesh_data->raytracing_animation = false;
      if (!mesh_data->radiosity_animation) {
        printf ("radiosity animation started, press 'X' to stop\n");
        mesh_data->radiosity_animation = true;
      } else {
        printf ("radiosity animation re-started, press 'X' to stop\n");
      }
      RadiosityClear();
      PackMesh();
      [renderer reGenerate];
      break;
    }
    case (KEY_S): {
      // subdivide the mesh for radiosity
      RadiositySubdivide();
      PackMesh();
      [renderer reGenerate];
      break;
    }
    case(KEY_C): {
      mesh_data->raytracing_animation = false;
      mesh_data->radiosity_animation = false;
      RaytracerClear();
      RadiosityClear();
      PhotonMappingClear();
      PackMesh();
      [renderer reGenerate];
      break;
    }
    case(KEY_X): {
      if (mesh_data->raytracing_animation) {
        if (!mesh_data->gather_indirect) {
          printf ("raytracing animation stopped, press 'R' to start\n");
        } else {
          printf ("photon mapping animation stopped, press 'G' to start\n");
        }
      }
      if (mesh_data->radiosity_animation) {
        printf ("radiosity animation stopped, press 'A' to start\n");
      }
      mesh_data->raytracing_animation = false;
      mesh_data->radiosity_animation = false;
      break;
    }

      // VISUALIZATIONS
      
    case (KEY_W): {
      // render wireframe mode
      mesh_data->wireframe = !mesh_data->wireframe;
      PackMesh();
      [renderer reGenerate];
      break;
    }

    case (KEY_V): {
      // toggle the different visualization modes
      mesh_data->render_mode = (mesh_data->render_mode+1)%NUM_RENDER_MODES;
      switch (mesh_data->render_mode) {
      case RENDER_MATERIALS: printf("RENDER_MATERIALS\n"); break;
      case RENDER_LIGHTS: printf("RENDER_LIGHTS\n"); break;
      case RENDER_UNDISTRIBUTED: printf("RENDER_UNDISTRIBUTED\n"); break;
      case RENDER_ABSORBED: printf("RENDER_ABSORBED\n"); break;
      case RENDER_RADIANCE: printf("RENDER_RADIANCE\n"); break;
      case RENDER_FORM_FACTORS: printf("RENDER_FORM_FACTORS\n"); break;
      default: assert(0); }
      PackMesh();
      [renderer reGenerate];
      break;
    }
    case (KEY_I): {
      // interpolate patch illumination values
      mesh_data->interpolate = !mesh_data->interpolate;
      PackMesh();
      [renderer reGenerate];
      break;
    }
    case (KEY_B): {
      // toggle backfacing triangle rendering
      mesh_data->intersect_backfacing = !mesh_data->intersect_backfacing;
      PackMesh();
      [renderer reGenerate];
      break;
    }

    case (KEY_F): {
      mesh_data->bounding_box_frame = !mesh_data->bounding_box_frame;
      PackMesh();
      [renderer reGenerate];
      break;
    }

    case (KEY_Q): 
      { printf ("quit\n"); exit(0); }
    default:
      { printf ("UNKNOWN key down %d\n", event.keyCode); break; }
    }
  }

- (void)keyUp:(NSEvent *) event
  {
    switch (event.keyCode) {
    default:
      { /*printf ("UNKNOWN key up %d\n", event.keyCode); */ break; }
    }
  }


// =======================
// mouse actions
// =======================
- (void)mouseDown:(NSEvent *) event {
  NSPoint touchPoint = [event locationInWindow];
  int which = event.buttonNumber;
  mouse_x = touchPoint.x;
  mouse_y = touchPoint.y;
  mesh_data->raytracing_animation = false;
}
- (void)rightMouseDown:(NSEvent *) event { [self mouseDown:event]; }
- (void)otherMouseDown:(NSEvent *) event { [self mouseDown:event]; }
 
- (void)mouseDragged:(NSEvent *) event {
  NSPoint touchPoint = [event locationInWindow];
  float delta_x = mouse_x-touchPoint.x;
  float delta_y = mouse_y-touchPoint.y;
  mouse_x = touchPoint.x;
  mouse_y = touchPoint.y;
  int which = event.buttonNumber;
  // to fake other mouse buttons...
  //if (shift_pressed) which = 0;
  if (control_pressed) which = 0;
  if (option_pressed) which = 2;
  if (command_pressed) which = 1;
  if (which == 0) {
    cameraRotate(delta_x,-delta_y);
  } else if (which == 2) {
    cameraZoom(-delta_y);
  } else {
    assert (which == 1);
    cameraTranslate(delta_x,delta_y);
  }
}
- (void)rightMouseDragged:(NSEvent *) event { [self mouseDragged:event]; }
- (void)otherMouseDragged:(NSEvent *) event { [self mouseDragged:event]; }

- (void)mouseUp:(NSEvent *) event {
  NSPoint touchPoint = [event locationInWindow];
  mouse_x = touchPoint.x;
  mouse_y = touchPoint.y;
}
- (void)rightMouseUp:(NSEvent *) event { [self mouseUp:event]; }
- (void)otherMouseUp:(NSEvent *) event { [self mouseUp:event]; }

@end
