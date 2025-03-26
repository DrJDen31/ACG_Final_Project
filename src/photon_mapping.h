#ifndef _PHOTON_MAPPING_H_
#define _PHOTON_MAPPING_H_

#include <vector>

#include "photon.h"

class Mesh;
class ArgParser;
class KDTree;
class Ray;
class Hit;
class RayTracer;
class Radiosity;

// =========================================================================
// The basic class to shoot photons within the scene and collect and
// process the nearest photons for use in the raytracer

class PhotonMapping {

 public:

  // CONSTRUCTOR & DESTRUCTOR
  PhotonMapping(Mesh *_mesh, ArgParser *_args) {
    mesh = _mesh;
    args = _args;
    raytracer = NULL;
    kdtree = NULL;
  }
  ~PhotonMapping() { Clear(); }
  void setRayTracer(RayTracer *r) { raytracer = r; }
  void setRadiosity(Radiosity *r) { radiosity = r; }

  // step 1: send the photons throughout the scene
  void TracePhotons();
  // step 2: collect the photons and return the contribution from indirect illumination
  Vec3f GatherIndirect(const Vec3f &point, const Vec3f &normal, const Vec3f &direction_from) const;

  void Clear();
  
  int triCount() const;
  int pointCount() const;
  void packMesh(float* &current, float* &current_points);
  
 private:

  // trace a single photon
  void TracePhoton(const Vec3f &position, const Vec3f &direction, const Vec3f &energy, int iter);

  // REPRESENTATION
  KDTree *kdtree;
  Mesh *mesh;
  ArgParser *args;
  RayTracer *raytracer;
  Radiosity *radiosity;
};

// =========================================================================

#endif
