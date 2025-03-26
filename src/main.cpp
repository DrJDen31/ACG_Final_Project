#include "argparser.h"
#include "meshdata.h"


// =========================================================
// OS specific rendering for OpenGL and Apple Metal
// =========================================================
#if __APPLE__
extern "C" {
int NSApplicationMain(int argc, const char * argv[]);
}
#else
#include "OpenGLRenderer.h"
#endif


// The one significant global variable
MeshData *mesh_data;



int main(int argc, const char * argv[]) {

  
  // parse the command line arguments and initialize the MeshData
  MeshData mymesh_data;
  mesh_data = &mymesh_data;
  ArgParser args(argc, argv, mesh_data);


  // launch the OS specific renderer
#if __APPLE__
  return NSApplicationMain(argc, argv);
#else
  OpenGLRenderer opengl_renderer(mesh_data,&args);
#endif
}
