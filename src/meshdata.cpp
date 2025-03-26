#include <string.h>
#include <string.h>
#include <cassert>
#include <iostream>
#include <fstream>
#include <random>

#include "matrix.h"
#include "mesh.h"
#include "meshdata.h"
#include "argparser.h"
#include "raytracer.h"
#include "raytree.h"
#include "radiosity.h"
#include "photon_mapping.h"
#include "camera.h"

// ====================================================================
// ====================================================================

// NOTE: These functions are called by the Objective-C code, so we
// need this extern to allow C code to call C++ functions (without
// function name mangling confusion).

// Also, they use global variables...  

extern "C" {

  void RayTreeActivate() {
    RayTree::Activate();
  }

  void RayTreeDeactivate() {
    RayTree::Deactivate();
  }

  void PhotonMappingTracePhotons() {
    GLOBAL_args->photon_mapping->TracePhotons();
  }

  void RadiosityIterate() {
    GLOBAL_args->radiosity->Iterate();
  }

  void RadiositySubdivide() {
    GLOBAL_args->radiosity->Cleanup();
    GLOBAL_args->radiosity->getMesh()->Subdivision();
    GLOBAL_args->radiosity->Reset();
  }

  void RadiosityClear() {
    GLOBAL_args->radiosity->Reset();
  }

  void RaytracerClear() {
    GLOBAL_args->raytracer->pixels_a.clear();
    GLOBAL_args->raytracer->pixels_b.clear();
    GLOBAL_args->raytracer->render_to_a = true;
  }

  void PhotonMappingClear() {
    GLOBAL_args->photon_mapping->Clear();
  }
  
  void PackMesh() {
    packMesh(GLOBAL_args->mesh_data, GLOBAL_args->raytracer, GLOBAL_args->radiosity, GLOBAL_args->photon_mapping);
  }

  void Load() {
    GLOBAL_args->Load();
  }

  bool DrawPixel() {
    return (bool)RayTraceDrawPixel();
  }

  void cameraTranslate(float x, float y) {
    GLOBAL_args->mesh->camera->truckCamera(x,y);
  }
  void cameraRotate(float x, float y) {
    GLOBAL_args->mesh->camera->rotateCamera(x,y);
  }
  void cameraZoom(float y) {
    GLOBAL_args->mesh->camera->zoomCamera(y);
  }

  void TraceRay(float x, float y) {
    VisualizeTraceRay(x,y);
  }
  
  void placeCamera() {
    GLOBAL_args->mesh->camera->glPlaceCamera();
  }

}

// ====================================================================
// ====================================================================
