#ifndef _RAY_TRACER_
#define _RAY_TRACER_

#include <vector>
#include "ray.h"
#include "hit.h"
#include "meshdata.h"

class Mesh;
class ArgParser;
class Radiosity;
class PhotonMapping;

// ====================================================================
// ====================================================================
// This class manages the ray casting and ray tracing work.

class Pixel {
public:
  Vec3f v1,v2,v3,v4;
  Vec3f color;

};


class RayTracer {

public:

  // CONSTRUCTOR & DESTRUCTOR
  RayTracer(Mesh *m, ArgParser *a) {
    mesh = m;
    args = a;
    render_to_a = true;
  }  
  // set access to the other modules for hybrid rendering options
  void setRadiosity(Radiosity *r) { radiosity = r; }
  void setPhotonMapping(PhotonMapping *pm) { photon_mapping = pm; }

  // casts a single ray through the scene geometry and finds the closest hit
  bool CastRay(const Ray &ray, Hit &h, bool use_sphere_patches) const;

  // does the recursive work
  Vec3f TraceRay(Ray &ray, Hit &hit, int bounce_count = 0) const;

private:

  void drawVBOs_a();
  void drawVBOs_b();

  // REPRESENTATION
  Mesh *mesh;
  ArgParser *args;
  Radiosity *radiosity;
  PhotonMapping *photon_mapping;

public:
  bool render_to_a;

  std::vector<Pixel> pixels_a;
  std::vector<Pixel> pixels_b;

  int triCount();
  void packMesh(float* &current);
};

// ====================================================================
// ====================================================================

int RayTraceDrawPixel();

Vec3f VisualizeTraceRay(double i, double j);


#endif
