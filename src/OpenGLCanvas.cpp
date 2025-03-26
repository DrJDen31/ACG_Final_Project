#include <vector>

#include "OpenGLCanvas.h"
#include "meshdata.h"
#include "argparser.h"
#include "camera.h"
#include "OpenGLRenderer.h"
#include "mesh.h"
#include "raytracer.h"

// ========================================================
// static variables of OpenGLCanvas class

ArgParser* OpenGLCanvas::args = NULL;
MeshData* OpenGLCanvas::mesh_data = NULL;
OpenGLRenderer* OpenGLCanvas::renderer = NULL;
GLFWwindow* OpenGLCanvas::window = NULL;

// mouse position
int OpenGLCanvas::mouseX = 0;
int OpenGLCanvas::mouseY = 0;
// which mouse button
bool OpenGLCanvas::leftMousePressed = false;
bool OpenGLCanvas::middleMousePressed = false;
bool OpenGLCanvas::rightMousePressed = false;
// current state of modifier keys
bool OpenGLCanvas::shiftKeyPressed = false;
bool OpenGLCanvas::controlKeyPressed = false;
bool OpenGLCanvas::altKeyPressed = false;
bool OpenGLCanvas::superKeyPressed = false;

// ========================================================
// Initialize all appropriate OpenGL variables, set
// callback functions, and start the main event loop.
// This function will not return but can be terminated
// by calling 'exit(0)'
// ========================================================

void OpenGLCanvas::initialize(ArgParser *_args, MeshData *_mesh_data, OpenGLRenderer *_renderer) {
  args = _args;
  mesh_data = _mesh_data;
  renderer = _renderer;
  
  glfwSetErrorCallback(error_callback);

  // Initialize GLFW
  if( !glfwInit() ) {
    std::cerr << "ERROR: Failed to initialize GLFW" << std::endl;
    exit(1);
  }
  
  // We will ask it to specifically open an OpenGL 3.2 context
  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  // Create a GLFW window
  window = glfwCreateWindow(mesh_data->width,mesh_data->height, "ACG HW1 MESHES", NULL, NULL);
  if (!window) {
    std::cerr << "ERROR: Failed to open GLFW window" << std::endl;
    glfwTerminate();
    exit(1);
  }
  glfwMakeContextCurrent(window);
  HandleGLError("in glcanvas first");

  // Initialize GLEW
  glewExperimental = true; // Needed for core profile
  if (glewInit() != GLEW_OK) {
    std::cerr << "ERROR: Failed to initialize GLEW" << std::endl;
    glfwTerminate();
    exit(1);
  }

  // there seems to be a "GL_INVALID_ENUM" error in glewInit that is a
  // know issue, but can safely be ignored
  HandleGLError("after glewInit()",true);

  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << "OpenGL Version: " << (char*)glGetString(GL_VERSION) << '\n';
  std::cout << "-------------------------------------------------------" << std::endl;

  // Initialize callback functions
  glfwSetCursorPosCallback(OpenGLCanvas::window,OpenGLCanvas::mousemotionCB);
  glfwSetMouseButtonCallback(OpenGLCanvas::window,OpenGLCanvas::mousebuttonCB);
  glfwSetKeyCallback(OpenGLCanvas::window,OpenGLCanvas::keyboardCB);

  GLOBAL_args->mesh->camera->glPlaceCamera();
  
  HandleGLError("finished glcanvas initialize");
}

// ========================================================
// Callback function for mouse click or release
// ========================================================

void OpenGLCanvas::mousebuttonCB(GLFWwindow* /*window*/, int which_button, int action, int /*mods*/) {
  // store the current state of the mouse buttons
  if (which_button == GLFW_MOUSE_BUTTON_1) {
    if (action == GLFW_PRESS) {
      leftMousePressed = true;
    } else {
      assert (action == GLFW_RELEASE);
      leftMousePressed = false;
    }
  } else if (which_button == GLFW_MOUSE_BUTTON_2) {
    if (action == GLFW_PRESS) {
      rightMousePressed = true;
    } else {
      assert (action == GLFW_RELEASE);
      rightMousePressed = false;
    }
  } else if (which_button == GLFW_MOUSE_BUTTON_3) {
    if (action == GLFW_PRESS) {
      middleMousePressed = true;
    } else {
      assert (action == GLFW_RELEASE);
      middleMousePressed = false;
    }
  }
}	

// ========================================================
// Callback function for mouse drag
// ========================================================

void OpenGLCanvas::mousemotionCB(GLFWwindow* /*window*/, double x, double y) {
  // camera controls that work well for a 3 button mouse
  if (!shiftKeyPressed && !controlKeyPressed && !altKeyPressed) {
    if (leftMousePressed) {
      GLOBAL_args->mesh->camera->rotateCamera(mouseX-x,mouseY-y);
    } else if (middleMousePressed)  {
      GLOBAL_args->mesh->camera->truckCamera(mouseX-x, y-mouseY);
    } else if (rightMousePressed) {
      GLOBAL_args->mesh->camera->dollyCamera(mouseY-y);
    }
  }

  if (leftMousePressed || middleMousePressed || rightMousePressed) {
    if (shiftKeyPressed) {
      GLOBAL_args->mesh->camera->zoomCamera(mouseY-y);
    }
    // allow reasonable control for a non-3 button mouse
    if (controlKeyPressed) {
      GLOBAL_args->mesh->camera->truckCamera(mouseX-x, y-mouseY);    
    }
    if (altKeyPressed) {
      GLOBAL_args->mesh->camera->dollyCamera(y-mouseY);    
    }
  }
  mouseX = x;
  mouseY = y;
  mesh_data->raytracing_animation = false;
}

// ========================================================
// Callback function for keyboard events
// ========================================================


// NOTE: These functions are also called by the Mac Metal Objective-C
// code, so we need this extern to allow C code to call C++ functions
// (without function name mangling confusion).

extern "C" {
void Load();
void RayTreeActivate();
void RayTreeDeactivate();
void PhotonMappingTracePhotons();
void RadiosityIterate();
void RadiositySubdivide();
void RadiosityClear();
void RaytracerClear();
void PhotonMappingClear();
void PackMesh();
}

void OpenGLCanvas::keyboardCB(GLFWwindow* /*window*/, int key, int /*scancode*/, int action, int mods) {
  // store the modifier keys
  shiftKeyPressed = (GLFW_MOD_SHIFT & mods);
  controlKeyPressed = (GLFW_MOD_CONTROL & mods);
  altKeyPressed = (GLFW_MOD_ALT & mods);
  superKeyPressed = (GLFW_MOD_SUPER & mods);
  // non modifier key actions
  if (key == GLFW_KEY_ESCAPE || key == 'q' || key == 'Q') {
    glfwSetWindowShouldClose(OpenGLCanvas::window, GL_TRUE);
    // force quit
    exit(0);
  }
  if ((action == GLFW_PRESS || action == GLFW_REPEAT) && key < 256) {
    switch (key) {

    case 'r': case 'R': case 'g': case 'G': {
      mesh_data->radiosity_animation = false;
      if (key == 'r' || key == 'R') {
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

    case 't':  case 'T': {
      // visualize the ray tree for the pixel at the current mouse position
      RayTreeActivate();
      mesh_data->raytracing_divs_x = -1;
      mesh_data->raytracing_divs_y = -1;
      VisualizeTraceRay(mouseX,args->mesh_data->height-mouseY);
      RayTreeDeactivate();
      break; 
    }
        
    case 'l':  case 'L': {
      // toggle photon rendering
      mesh_data->render_photons = !mesh_data->render_photons;
      break;
    }
    case 'd':  case 'D': {
      // toggle photon rendering
      mesh_data->render_photon_directions = !mesh_data->render_photon_directions;
      break;
    }
    case 'k':  case 'K': {
      // toggle kd tree rendering
      mesh_data->render_kdtree = !mesh_data->render_kdtree;
      break;
    }
    case 'p':  case 'P': {
      // trace photons
      PhotonMappingTracePhotons();
      break; 
    }

    case ' ': {
      // a single step of radiosity
      RadiosityIterate();
      break; 
    }
    case 'a': case 'A': {
      // animate radiosity solution
      mesh_data->raytracing_animation = false;
      if (!mesh_data->radiosity_animation) {
        printf ("radiosity animation started, press 'X' to stop\n");
        mesh_data->radiosity_animation = true;
      } else {
        printf ("radiosity animation re-started, press 'X' to stop\n");
      }
      RadiosityClear();
      break; 
    }
    case 's':  case 'S': {
      // subdivide the mesh for radiosity
      RadiositySubdivide();
      break; 
    }
    case 'c':  case 'C': {
      mesh_data->raytracing_animation = false;
      mesh_data->radiosity_animation = false;
      RadiosityClear();
      RaytracerClear();
      PhotonMappingClear();
      break; 
    }
    case 'x':  case 'X': {
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
      
    case 'w': case 'W': {
      // render wireframe mode
      mesh_data->wireframe = !mesh_data->wireframe;
      break;
    }

    case 'v': case 'V': {
      // toggle the different visualization modes
      mesh_data->render_mode = (RENDER_MODE)((mesh_data->render_mode+1)%NUM_RENDER_MODES);
      switch (mesh_data->render_mode) {
      case RENDER_MATERIALS: printf("RENDER_MATERIALS\n"); break;
      case RENDER_LIGHTS: printf("RENDER_LIGHTS\n"); break;
      case RENDER_UNDISTRIBUTED: printf("RENDER_UNDISTRIBUTED\n"); break;
      case RENDER_ABSORBED: printf("RENDER_ABSORBED\n"); break;
      case RENDER_RADIANCE: printf("RENDER_RADIANCE\n"); break;
      case RENDER_FORM_FACTORS: printf("RENDER_FORM_FACTORS\n"); break;
      default: assert(0); }
      break;
    }
    case 'i': case 'I': {
      // interpolate patch illumination values
      mesh_data->interpolate = !mesh_data->interpolate;
      break;
    }
    case 'b': case 'B': {
      // toggle backfacing triangle rendering
      mesh_data->intersect_backfacing = !mesh_data->intersect_backfacing;
      break;
    }

    case 'f':  case 'F':
      mesh_data->bounding_box_frame = !mesh_data->bounding_box_frame;
      break;
    
    case 'q':  case 'Q': {
      exit(0);
      break;
    }
    default:
      std::cout << "UNKNOWN KEYBOARD INPUT  '" << (char)key << "'" << std::endl;
    }
    PackMesh();
    renderer->updateVBOs();
  }
}

// ========================================================
// Load the vertex & fragment shaders
// ========================================================

GLuint LoadShaders(const std::string &vertex_file_path,const std::string &fragment_file_path){

  // Create the shaders
  GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
  GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
  
  // Read the Vertex Shader code from the file
  std::string VertexShaderCode;
  std::ifstream VertexShaderStream(vertex_file_path.c_str(), std::ios::in);
  if (VertexShaderStream.is_open()){
    std::string Line = "";
    while(getline(VertexShaderStream, Line))
      VertexShaderCode += "\n" + Line;
    VertexShaderStream.close();
  } else {
    std::cerr << "ERROR: cannot open " << vertex_file_path << std::endl;
    exit(0);
  }
  // Read the Fragment Shader code from the file
  std::string FragmentShaderCode;
  std::ifstream FragmentShaderStream(fragment_file_path.c_str(), std::ios::in);
  if(FragmentShaderStream.is_open()){
    std::string Line = "";
    while(getline(FragmentShaderStream, Line))
      FragmentShaderCode += "\n" + Line;
    FragmentShaderStream.close();
  } else {
    std::cerr << "ERROR: cannot open " << vertex_file_path << std::endl;
    exit(0);
  }
  
  GLint Result = GL_FALSE;
  int InfoLogLength;
  
  // Compile Vertex Shader
  std::cout << "Compiling shader : " << vertex_file_path << std::endl;
  char const * VertexSourcePointer = VertexShaderCode.c_str();
  glShaderSource(VertexShaderID, 1, &VertexSourcePointer , NULL);
  glCompileShader(VertexShaderID);
  // Check Vertex Shader
  glGetShaderiv(VertexShaderID, GL_COMPILE_STATUS, &Result);
  glGetShaderiv(VertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
  if ( InfoLogLength > 0 ){
    std::vector<char> VertexShaderErrorMessage(InfoLogLength+1);
    glGetShaderInfoLog(VertexShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
    //std::cerr << "ERROR: " << VertexShaderErrorMessage[0] << std::endl;
  }
  
  // Compile Fragment Shader
  std::cout << "Compiling shader : " << fragment_file_path << std::endl;
  char const * FragmentSourcePointer = FragmentShaderCode.c_str();
  glShaderSource(FragmentShaderID, 1, &FragmentSourcePointer , NULL);
  glCompileShader(FragmentShaderID);
  // Check Fragment Shader
  glGetShaderiv(FragmentShaderID, GL_COMPILE_STATUS, &Result);
  glGetShaderiv(FragmentShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
  if ( InfoLogLength > 0 ){
    std::vector<char> FragmentShaderErrorMessage(InfoLogLength+1);
    glGetShaderInfoLog(FragmentShaderID, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
    //std::cerr << "ERROR: " << FragmentShaderErrorMessage[0] << std::endl;
  }
  
  // Link the program
  std::cout << "Linking program" << std::endl;
  GLuint ProgramID = glCreateProgram();
  glAttachShader(ProgramID, VertexShaderID);
  glAttachShader(ProgramID, FragmentShaderID);
  glLinkProgram(ProgramID);
  // Check the program
  glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
  glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
  if ( InfoLogLength > 0 ){
    std::vector<char> ProgramErrorMessage(InfoLogLength+1);
    glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
    //std::cerr << "ERROR: " << ProgramErrorMessage[0] << std::endl;
  }
  
  glDeleteShader(VertexShaderID);
  glDeleteShader(FragmentShaderID);
  
  return ProgramID;
}

// ========================================================
// Functions related to error handling
// ========================================================

void OpenGLCanvas::error_callback(int /*error*/, const char* description) {
  std::cerr << "ERROR CALLBACK: " << description << std::endl;
}

std::string WhichGLError(GLenum &error) {
  switch (error) {
  case GL_NO_ERROR:
    return "NO_ERROR";
  case GL_INVALID_ENUM:
    return "GL_INVALID_ENUM";
  case GL_INVALID_VALUE:
    return "GL_INVALID_VALUE";
  case GL_INVALID_OPERATION:
    return "GL_INVALID_OPERATION";
  case GL_INVALID_FRAMEBUFFER_OPERATION:
    return "GL_INVALID_FRAMEBUFFER_OPERATION";
  case GL_OUT_OF_MEMORY:
    return "GL_OUT_OF_MEMORY";
  case GL_STACK_UNDERFLOW:
    return "GL_STACK_UNDERFLOW";
  case GL_STACK_OVERFLOW:
    return "GL_STACK_OVERFLOW";
  default:
    return "OTHER GL ERROR";
  }
}

int HandleGLError(const std::string &message, bool ignore) {
  GLenum error;
  int i = 0;
  while ((error = glGetError()) != GL_NO_ERROR) {
    if (!ignore) {
      if (message != "") {
	std::cout << "[" << message << "] ";
      }
      std::cout << "GL ERROR(" << i << ") " << WhichGLError(error) << std::endl;
    }
    i++;
  }
  if (i == 0) return 1;
  return 0;
}



// ====================================================================
// Construct the ViewMatrix & ProjectionMatrix for GL Rendering
// ====================================================================

void convert(Matrix &b, const glm::mat4 &a);

void OrthographicCamera::glPlaceCamera() {

  if (OpenGLCanvas::window == NULL) return;
    
  glfwGetWindowSize(OpenGLCanvas::window, &GLOBAL_args->mesh_data->width, &GLOBAL_args->mesh_data->height);
  float aspect = GLOBAL_args->mesh_data->width / (float)GLOBAL_args->mesh_data->height;
  float w;
  float h;
  // handle non square windows
  if (aspect < 1.0) {
    w = size / 2.0;
    h = w / aspect;
  } else {
    h = size / 2.0;
    w = h * aspect;
  }

  glm::mat4 projmat = glm::ortho<float>(-w,w,-h,h, 0.1f, 100.0f) ;
  glm::vec3 campos(camera_position.x(),camera_position.y(), camera_position.z());
  glm::vec3 poi(point_of_interest.x(),point_of_interest.y(),point_of_interest.z());
  glm::vec3 screenup(getScreenUp().x(),getScreenUp().y(),getScreenUp().z());
  glm::mat4 viewmat = glm::lookAt(campos,poi,screenup);

  convert(ProjectionMatrix,projmat);
  convert(ViewMatrix,viewmat);
}

void PerspectiveCamera::glPlaceCamera() {
  
  if (OpenGLCanvas::window == NULL) return;

  glfwGetWindowSize(OpenGLCanvas::window, &GLOBAL_args->mesh_data->width, &GLOBAL_args->mesh_data->height);
  float aspect = GLOBAL_args->mesh_data->width / (float)GLOBAL_args->mesh_data->height;
  // must convert angle from degrees to radians

  glm::vec3 campos(camera_position.x(),camera_position.y(), camera_position.z());
  glm::vec3 poi(point_of_interest.x(),point_of_interest.y(),point_of_interest.z());
  glm::vec3 screenup(getScreenUp().x(),getScreenUp().y(),getScreenUp().z());

  glm::mat4 projmat = glm::perspective<float>(glm::radians(angle), aspect, 0.1f, 1000.0f);
  convert (ProjectionMatrix,projmat);
  glm::mat4 viewmat =  glm::lookAt(campos,poi,screenup);
  convert (ViewMatrix,viewmat);
}

// ========================================================
// ========================================================
