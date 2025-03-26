#include "utils.h"
#include "material.h"
#include "argparser.h"
#include "sphere.h"
#include "vertex.h"
#include "mesh.h"
#include "ray.h"
#include "hit.h"


#include "raytree.h"

// ====================================================================
// ====================================================================

bool Sphere::intersect(const Ray &r, Hit &h) const {

    // ==========================================
    // ASSIGNMENT:  IMPLEMENT SPHERE INTERSECTION
    // ==========================================

    // plug the explicit ray equation into the implict sphere equation and solve

    // H(P) = ((P - center) * (P - center)) - r^2 = 0
    // P(t) = ray_origin + (t * ray_direction)
    // ==> (((ray_origin + (t * ray_direction)) - center) * ((ray_origin + (t * ray_direction)) - center)) - r^2 = 0
    // ==> 

    double a = 1;
    double b = ((2.0 * r.getDirection().Dot3(r.getOrigin()))) - (2.0 * center.Dot3(r.getDirection()));
    double c = ((-2.0 * center.Dot3(r.getOrigin())) + (r.getOrigin().Dot3(r.getOrigin())) + (center.Dot3(center))) - (radius * radius);
    double pre_d = (b * b) - (4.0 * a * c);
    if (pre_d < 0) { return false; }
    double d = sqrt(pre_d);
    double lower = ((- 1.0 * b) - d) / (2.0 * a);
    double upper = ((-1.0 * b) + d) / (2.0 * a);
    if (lower < 0 || upper < 0) { return false; }

    // use the smaller (closer) t
    double t = std::min(upper, lower);

    // get the point and normal
    Vec3f point = r.getOrigin() + (t * r.getDirection());

    Vec3f normal = (point - center);
    normal.Normalize();
    h.set(t, this->material, normal);

    // return true if the sphere was intersected, and update the hit
    // data structure to contain the value of t for the ray at the
    // intersection point, the material, and the normal

    return true;

} 

// ====================================================================
// ====================================================================

// helper function to place a grid of points on the sphere
Vec3f ComputeSpherePoint(float s, float t, const Vec3f center, float radius) {
  float angle = 2*M_PI*s;
  float y = -cos(M_PI*t);
  float factor = sqrt(1-y*y);
  float x = factor*cos(angle);
  float z = factor*-sin(angle);
  Vec3f answer = Vec3f(x,y,z);
  answer *= radius;
  answer += center;
  return answer;
}

void Sphere::addRasterizedFaces(Mesh *m, ArgParser *args) {
  
  // and convert it into quad patches for radiosity
  int h = args->mesh_data->sphere_horiz;
  int v = args->mesh_data->sphere_vert;
  assert (h % 2 == 0);
  int i,j;
  int va,vb,vc,vd;
  Vertex *a,*b,*c,*d;
  int offset = m->numVertices(); //vertices.size();

  // place vertices
  m->addVertex(center+radius*Vec3f(0,-1,0));  // bottom
  for (j = 1; j < v; j++) {  // middle
    for (i = 0; i < h; i++) {
      float s = i / float(h);
      float t = j / float(v);
      m->addVertex(ComputeSpherePoint(s,t,center,radius));
    }
  }
  m->addVertex(center+radius*Vec3f(0,1,0));  // top

  // the middle patches
  for (j = 1; j < v-1; j++) {
    for (i = 0; i < h; i++) {
      va = 1 +  i      + h*(j-1);
      vb = 1 + (i+1)%h + h*(j-1);
      vc = 1 +  i      + h*(j);
      vd = 1 + (i+1)%h + h*(j);
      a = m->getVertex(offset + va);
      b = m->getVertex(offset + vb);
      c = m->getVertex(offset + vc);
      d = m->getVertex(offset + vd);
      m->addRasterizedPrimitiveFace(a,b,d,c,material);
    }
  }

  for (i = 0; i < h; i+=2) {
    // the bottom patches
    va = 0;
    vb = 1 +  i;
    vc = 1 + (i+1)%h;
    vd = 1 + (i+2)%h;
    a = m->getVertex(offset + va);
    b = m->getVertex(offset + vb);
    c = m->getVertex(offset + vc);
    d = m->getVertex(offset + vd);
    m->addRasterizedPrimitiveFace(d,c,b,a,material);
    // the top patches
    va = 1 + h*(v-1);
    vb = 1 +  i      + h*(v-2);
    vc = 1 + (i+1)%h + h*(v-2);
    vd = 1 + (i+2)%h + h*(v-2);
    a = m->getVertex(offset + va);
    b = m->getVertex(offset + vb);
    c = m->getVertex(offset + vc);
    d = m->getVertex(offset + vd);
    m->addRasterizedPrimitiveFace(b,c,d,a,material);
  }
}
