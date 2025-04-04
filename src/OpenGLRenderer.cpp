// ==================================================================
// OpenGL Rendering of the MeshData data
// ==================================================================

#include "OpenGLRenderer.h"
#include "OpenGLCanvas.h"
#include "camera.h"
#include "mesh.h"
#include "meshdata.h"
#include "matrix.h"
#include "boundingbox.h"

#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>

// NOTE: These functions are also called by the Mac Metal Objective-C
// code, so we need this extern to allow C code to call C++ functions
// (without function name mangling confusion).

extern "C" {
void RayTreeActivate();
void RayTreeDeactivate();
void PhotonMappingTracePhotons();
void RadiosityIterate();
void RadiositySubdivide();
void RadiosityClear();
void RaytracerClear();
void PackMesh();
bool DrawPixel();
}

// ==================================================================
// helper functions to convert between matrix formats

void convert(Matrix &b, const glm::mat4 &a) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            b.set(j,i,a[i][j]);
        }
    }
}

void convert(glm::mat4 &a, const Matrix &b) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            a[i][j] = b.get(j,i);
        }
    }
}

// ==================================================================
// update the raytracing and/or radiosity simulation

void Animate() {
  if (GLOBAL_args->mesh_data->raytracing_animation) {
    // draw 100 pixels and then refresh the screen and handle any user input
    for (int i = 0; i < 100; i++) {
      if (!DrawPixel()) {
        GLOBAL_args->mesh_data->raytracing_animation = false;
        break;
      }
    }
    PackMesh();
  }
  
  if (GLOBAL_args->mesh_data->radiosity_animation) {
    RadiosityIterate();
    PackMesh();
  }
}


// ====================================================================
// ====================================================================

void updateFPSCounter() {
    static auto lastTime = std::chrono::high_resolution_clock::now();
    static int frameCount = 0;

    frameCount++;
    auto currentTime = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(currentTime - lastTime).count();

    if (elapsed >= 1.0f) {
        std::cout << "\rFPS: " << std::setw(3) << frameCount << std::flush;
        frameCount = 0;
        lastTime = currentTime;
    }
}


OpenGLRenderer::OpenGLRenderer(MeshData *_mesh_data, ArgParser *args) {
  mesh_data = _mesh_data;

  OpenGLCanvas::initialize(args,mesh_data,this);

  // Initialize the MeshData
  setupVBOs();
  setupMPM();

  glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS); 
  glDisable(GL_CULL_FACE);

  // Create and compile our GLSL program from the shaders
  GLuint programID = LoadShaders( args->path+"/OpenGL.vertexshader", args->path+"/OpenGL.fragmentshader" );
  
  // Get handles for our uniforms
  MatrixID = glGetUniformLocation(programID, "MVP");
  LightID = glGetUniformLocation(programID, "LightPosition_worldspace");
  ViewMatrixID = glGetUniformLocation(programID, "V");
  ModelMatrixID = glGetUniformLocation(programID, "M");
  wireframeID = glGetUniformLocation(programID, "wireframe");
  renderModeID = glGetUniformLocation(programID, "renderMode");

  while (!glfwWindowShouldClose(OpenGLCanvas::window))  {
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(programID);

    GLOBAL_args->mesh->camera->glPlaceCamera();

    Matrix ProjectionMatrix = GLOBAL_args->mesh->camera->getProjectionMatrix();
    Matrix ViewMatrix = GLOBAL_args->mesh->camera->getViewMatrix();

    glm::mat4 ModelMatrix(1.0);
    glm::mat4 ProjectionMatrix_mat4;
    glm::mat4 ViewMatrix_mat4;
    convert(ProjectionMatrix_mat4,ProjectionMatrix);
    convert(ViewMatrix_mat4,ViewMatrix);    
    glm::mat4 MVP = ProjectionMatrix_mat4 * ViewMatrix_mat4 * ModelMatrix;
    
    Animate();
    updateVBOs();
    
    // pass the matrix to the draw routines (for further editing)
    drawVBOs(MVP,ModelMatrix,ViewMatrix_mat4);

    // Swap buffers
    glfwSwapBuffers(OpenGLCanvas::window);
    glfwPollEvents();  
  }
  
  cleanupVBOs();
  glDeleteProgram(programID);
  
  // Close OpenGL window and terminate GLFW
  glfwDestroyWindow(OpenGLCanvas::window);
  glfwTerminate();
  exit(EXIT_SUCCESS);
}

// ====================================================================

void OpenGLRenderer::setupVBOs() {
  HandleGLError("enter setupVBOs");
  glGenVertexArrays(1, &mesh_tris_VaoId);
  glGenVertexArrays(1, &mesh_points_VaoId);
  glGenBuffers(1, &mesh_tris_VBO);
  glGenBuffers(1, &mesh_points_VBO);
  HandleGLError("leaving setupVBOs");
}

void OpenGLRenderer::drawVBOs(const glm::mat4 &mvp,const glm::mat4 &m,const glm::mat4 &v) {
  HandleGLError("enter drawVBOs");
  Vec3f lightPos = Vec3f(4,4,4);
  glUniform3f(LightID, lightPos.x(), lightPos.y(), lightPos.z());
  glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mvp[0][0]);
  glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &m[0][0]);
  glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &v[0][0]);
  glUniform1i(wireframeID, mesh_data->wireframe);
  glUniform1i(renderModeID, mesh_data->render_mode);
  drawMesh();
  drawMPM();
  HandleGLError("leaving drawVBOs");
}

void OpenGLRenderer::cleanupVBOs() {
  HandleGLError("enter cleanupVBOs");
  cleanupMesh();
  cleanupMPM();
  HandleGLError("leaving cleanupVBOs");
}

// ====================================================================


void OpenGLRenderer::updateVBOs() {
  HandleGLError("enter updateVBOs");
  
  glBindVertexArray(mesh_tris_VaoId);
  glBindBuffer(GL_ARRAY_BUFFER, mesh_tris_VBO);
  int sizeOfVertices = 3*sizeof(glm::vec4) * mesh_data->meshTriCount * 3;
  glBufferData(GL_ARRAY_BUFFER, sizeOfVertices, mesh_data->meshTriData, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 3*sizeof(glm::vec4), 0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 3*sizeof(glm::vec4), (void*)sizeof(glm::vec4));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 3*sizeof(glm::vec4), (void*)(sizeof(glm::vec4)*2));

  glBindVertexArray(mesh_points_VaoId);
  glBindBuffer(GL_ARRAY_BUFFER, mesh_points_VBO);
  sizeOfVertices = 3*sizeof(glm::vec4) * mesh_data->meshPointCount;
  glBufferData(GL_ARRAY_BUFFER, sizeOfVertices, mesh_data->meshPointData, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 3*sizeof(glm::vec4), 0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 3*sizeof(glm::vec4), (void*)sizeof(glm::vec4));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 3*sizeof(glm::vec4), (void*)(sizeof(glm::vec4)*2));

  HandleGLError("leaving updateVBOs");
}

void OpenGLRenderer::drawMesh() const {
  HandleGLError("in drawMesh");

  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
    
  glBindVertexArray(mesh_tris_VaoId);
  glBindBuffer(GL_ARRAY_BUFFER, mesh_tris_VBO);
  glDrawArrays(GL_TRIANGLES, 0, 3 * mesh_data->meshTriCount);

  glBindVertexArray(mesh_points_VaoId);
  glBindBuffer(GL_ARRAY_BUFFER, mesh_points_VBO);
  glPointSize(2.0);
  glDrawArrays(GL_POINTS, 0, mesh_data->meshPointCount);

  HandleGLError("leaving drawMesh");
}

void OpenGLRenderer::cleanupMesh() {
  glDeleteBuffers(1, &mesh_tris_VBO);
  glDeleteBuffers(1, &mesh_points_VBO);
  glDeleteBuffers(1, &mesh_tris_VaoId);
  glDeleteBuffers(1, &mesh_points_VaoId);
}

// ====================================================================

void OpenGLRenderer::setupMPM() {
    int numParticles = GLOBAL_args->mesh_data->num_particles;
    int gridSize = 64;
    mpm_sim = new MPM(gridSize, numParticles);

    GLOBAL_args->mpm = mpm_sim;

    glGenVertexArrays(1, &mpm_debug_VAO);
    glGenBuffers(1, &mpm_debug_VBO);

    glBindVertexArray(mpm_debug_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, mpm_debug_VBO);

    // Allocate space for positions, will update later
    glBufferData(GL_ARRAY_BUFFER, numParticles * 3 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);


    glBindVertexArray(0);
}

// Call this once before usage to create output dir: `mkdir frames`
void saveFrame(int frameCount, GLFWwindow* window) {
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    std::vector<unsigned char> pixels(3 * width * height);

    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    // Flip vertically
    for (int y = 0; y < height / 2; ++y) {
        for (int x = 0; x < width * 3; ++x) {
            std::swap(pixels[y * width * 3 + x], pixels[(height - 1 - y) * width * 3 + x]);
        }
    }

    std::ostringstream filename;
    filename << "frames/frame_" << std::setw(4) << std::setfill('0') << frameCount << ".ppm";

    std::ofstream file(filename.str(), std::ios::binary);
    file << "P6\n" << width << " " << height << "\n255\n";
    file.write(reinterpret_cast<char*>(pixels.data()), pixels.size());
    file.close();
}

void OpenGLRenderer::drawMPM() const {
    /*if (!OpenGLCanvas::sim_paused || OpenGLCanvas::sim_step_once) {*/
        static float total_slowdown = 0.0f;
        static int frame_count = 0;
        static auto last_print = std::chrono::high_resolution_clock::now();

        auto start = std::chrono::high_resolution_clock::now();
        mpm_sim->step();
        auto end = std::chrono::high_resolution_clock::now();

        float real_time = std::chrono::duration<float>(end - start).count();
        float slowdown = real_time / dt;

        total_slowdown += slowdown;
        frame_count++;

        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<float>(now - last_print).count() >= 1.0f) {
            float avg_slowdown = total_slowdown / frame_count;
            std::cout << std::fixed << std::setprecision(5)
                      << "\rAvg slowdown: " << avg_slowdown << "x" << std::flush;
            last_print = now;
        }
        OpenGLCanvas::sim_step_once = false;
    /*}*/

    const std::vector<Particle>& particles = mpm_sim->getParticles();
    std::vector<float> data;
    data.reserve(particles.size() * 2);

    for (const auto& p : particles) {
        float x = p.x.x / 64.0f * 2.0f - 1.0f;
        float y = p.x.y / 64.0f * 2.0f - 1.0f;
        float z = p.x.z / 64.0f * 2.0f - 1.0f;
        data.push_back(x);
        data.push_back(y);
        data.push_back(z);
    }

    glBindBuffer(GL_ARRAY_BUFFER, mpm_debug_VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, data.size() * sizeof(float), data.data());
    glBindVertexArray(mpm_debug_VAO);
    glPointSize(4.0f);
    glDrawArrays(GL_POINTS, 0, mpm_sim->getParticles().size());

    // NOTE: for recording. Not important to sim
    static int frameID = 0;
    saveFrame(frameID++, OpenGLCanvas::window);

    glBindVertexArray(0);
}


void OpenGLRenderer::cleanupMPM() {
    delete mpm_sim;
    setupMPM();
}

// ====================================================================
