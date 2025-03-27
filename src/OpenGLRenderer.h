// ==================================================================
// OpenGL Rendering of the Mesh Data
// ==================================================================

#include "OpenGLCanvas.h"
#include "argparser.h"
#include "meshdata.h"
#include "mpm.h"

class OpenGLRenderer {

public:

  OpenGLRenderer(MeshData *_mesh_data, ArgParser *args);

  // HELPER FUNCTIONS FOR RENDERING
  void setupVBOs();
  void updateVBOs();
  void drawVBOs(const glm::mat4 &MVP,const glm::mat4 &M,const glm::mat4 &V);
  void cleanupVBOs();
  
private:

  // private helper functions for VBOs
  void setupMesh();
  void drawMesh() const;
  void cleanupMesh();

  // mpm-related helpers
  void setupMPM();
  void drawMPM() const;
  void cleanupMPM();

  // REPRESENTATION
  MeshData *mesh_data;

  GLuint mesh_tris_VBO;
  GLuint mesh_points_VBO;
  
  GLuint mesh_tris_VaoId;
  GLuint mesh_points_VaoId;

  GLuint MatrixID;
  GLuint LightID;
  GLuint ViewMatrixID;
  GLuint ModelMatrixID;
  GLuint wireframeID;
  GLuint renderModeID;

  // MPM simulation instance (private like mesh_data)
  MPM *mpm_sim;
GLuint mpm_debug_VAO;
GLuint mpm_debug_VBO;

};
