#include "vectors.h"
#include "radiosity.h"
#include "mesh.h"
#include "face.h"
#include "sphere.h"
#include "raytree.h"
#include "raytracer.h"
#include "utils.h"

// ================================================================
// CONSTRUCTOR & DESTRUCTOR
// ================================================================
Radiosity::Radiosity(Mesh *m, ArgParser *a) {
  mesh = m;
  args = a;
  num_faces = -1;  
  formfactors = NULL;
  area = NULL;
  undistributed = NULL;
  absorbed = NULL;
  radiance = NULL;
  max_undistributed_patch = -1;
  total_area = -1;
  Reset();
}

Radiosity::~Radiosity() {
  Cleanup();
}

void Radiosity::Cleanup() {
  delete [] formfactors;
  delete [] area;
  delete [] undistributed;
  delete [] absorbed;
  delete [] radiance;
  num_faces = -1;
  formfactors = NULL;
  area = NULL;
  undistributed = NULL;
  absorbed = NULL;
  radiance = NULL;
  max_undistributed_patch = -1;
  total_area = -1;
}

void Radiosity::Reset() {
  delete [] area;
  delete [] undistributed;
  delete [] absorbed;
  delete [] radiance;

  // create and fill the data structures
  num_faces = mesh->numFaces();
  area = new float[num_faces];
  undistributed = new Vec3f[num_faces];
  absorbed = new Vec3f[num_faces];
  radiance = new Vec3f[num_faces];
  for (int i = 0; i < num_faces; i++) {
    Face *f = mesh->getFace(i);
    f->setRadiosityPatchIndex(i);
    setArea(i,f->getArea());
    Vec3f emit = f->getMaterial()->getEmittedColor();
    setUndistributed(i,emit);
    setAbsorbed(i,Vec3f(0,0,0));
    setRadiance(i,emit);
  }

  // find the patch with the most undistributed energy
  findMaxUndistributed();
}


// =======================================================================================
// =======================================================================================

void Radiosity::findMaxUndistributed() {
  // find the patch with the most undistributed energy 
  // don't forget that the patches may have different sizes!
  max_undistributed_patch = -1;
  total_undistributed = 0;
  total_area = 0;
  float max = -1;
  for (int i = 0; i < num_faces; i++) {
    float m = getUndistributed(i).Length() * getArea(i);
    total_undistributed += m;
    total_area += getArea(i);
    if (max < m) {
      max = m;
      max_undistributed_patch = i;
    }
  }
  assert (max_undistributed_patch >= 0 && max_undistributed_patch < num_faces);
}


void Radiosity::ComputeFormFactors() {
  assert (formfactors == NULL);
  assert (num_faces > 0);
  formfactors = new float[num_faces*num_faces];


  // =====================================
  // ASSIGNMENT:  COMPUTE THE FORM FACTORS
  // =====================================

  // for each face
  for (int i = 0; i < num_faces; i++) {
      Face* fi = mesh->getFace(i);
      // for each face to face pair
      for (int j = 0; j < num_faces; j++) {
          Face* fj = mesh->getFace(j);
          if (i == j) {
              // 0 for a face to itself
              formfactors[(i * num_faces) + j] = 0;
              continue;
          }

          // tracking variables
          int point_samples = 10;
          int count = 0;
          double ff = 0;

          // do point_samples^2 samples
          for (int k = 0; k < point_samples; k++) {
              Vec3f start = fj->RandomPoint();
              for (int l = 0; l < point_samples; l++) {
                  // get a random point from each face
                  Vec3f end = fi->RandomPoint();
                  Vec3f p2p = end - start;
                  Vec3f direction = p2p;
                  direction.Normalize();
                  Vec3f other_way = direction;
                  other_way.Negate();
                  Ray sample(start, direction);
                  Hit sample_hit = Hit();
                  bool blocked = this->raytracer->CastRay(sample, sample_hit, false);

                  // check if nothing blocking the two faces
                  if (blocked) {
                      if ((sample.pointAtParameter(sample_hit.getT()) - end).Length() < 0.001) {
                          // visibility tracking
                          count++;
                      }

                      // sample of ff calc for these points
                      double theta_i = acos((fi->computeNormal().Dot3(other_way)));
                      double theta_j = acos((fj->computeNormal().Dot3(direction)));

                      ff += (((cos(theta_i)) * (cos(theta_j))) / (p2p.Length() * p2p.Length() * M_PI)) * ((fi->getArea()) / point_samples) * (fj->getArea() / point_samples);
                      
                  }
              }
          }
          
          // scale form factor
          double visibility = (1.0f * count) / (1.0f * point_samples * point_samples);
          double area_factor = (1.0f / (fi->getArea())) * visibility;
          formfactors[(i * num_faces) + j] = std::max(area_factor * (ff), 0.0); // * count_factor
      }
  }
}


// ================================================================
// ================================================================

float Radiosity::Iterate() {
    if (formfactors == NULL) {
        ComputeFormFactors();
    }
    assert (formfactors != NULL);

    // ==========================================
    // ASSIGNMENT:  IMPLEMENT RADIOSITY ALGORITHM
    // ==========================================

    // get max undistributed
    findMaxUndistributed();
    Vec3f undistributed = getUndistributed(max_undistributed_patch);


    float total_redistributed = 0;

    // distribute based on form factor
    for (int i = 0; i < num_faces; i++) {
        double ff = formfactors[(i * num_faces) + max_undistributed_patch];
        Vec3f to_i = undistributed * ff;
        Vec3f redistributed = to_i * mesh->getFace(i)->getMaterial()->getDiffuseColor();
        total_redistributed += redistributed.Length();

        setUndistributed(i, getUndistributed(i) + redistributed);
        setAbsorbed(i, getAbsorbed(i) + (to_i - redistributed));
        setRadiance(i, getRadiance(i) + redistributed);
    }

    // reset patch as it's energy has been distributed
    setUndistributed(max_undistributed_patch, Vec3f(0, 0, 0));


    // return the total light yet undistributed
    // (so we can decide when the solution has sufficiently converged)
    total_undistributed -= total_redistributed;

    return total_undistributed;


}



// =======================================================================================
// HELPER FUNCTIONS FOR RENDERING
// =======================================================================================

// for interpolation
void CollectFacesWithVertex(Vertex *have, Face *f, std::vector<Face*> &faces) {
  for (unsigned int i = 0; i < faces.size(); i++) {
    if (faces[i] == f) return;
  }
  if (have != (*f)[0] && have != (*f)[1] && have != (*f)[2] && have != (*f)[3]) return;
  faces.push_back(f);
  for (int i = 0; i < 4; i++) {
    Edge *ea = f->getEdge()->getOpposite();
    Edge *eb = f->getEdge()->getNext()->getOpposite();
    Edge *ec = f->getEdge()->getNext()->getNext()->getOpposite();
    Edge *ed = f->getEdge()->getNext()->getNext()->getNext()->getOpposite();
    if (ea != NULL) CollectFacesWithVertex(have,ea->getFace(),faces);
    if (eb != NULL) CollectFacesWithVertex(have,eb->getFace(),faces);
    if (ec != NULL) CollectFacesWithVertex(have,ec->getFace(),faces);
    if (ed != NULL) CollectFacesWithVertex(have,ed->getFace(),faces);
  }
}

// different visualization modes
Vec3f Radiosity::setupHelperForColor(Face *f, int i, int j) {
  assert (mesh->getFace(i) == f);
  assert (j >= 0 && j < 4);
  if (args->mesh_data->render_mode == RENDER_MATERIALS) {
    return f->getMaterial()->getDiffuseColor();
  } else if (args->mesh_data->render_mode == RENDER_RADIANCE && args->mesh_data->interpolate == true) {
    std::vector<Face*> faces;
    CollectFacesWithVertex((*f)[j],f,faces);
    float total = 0;
    Vec3f color = Vec3f(0,0,0);
    Vec3f normal = f->computeNormal();
    for (unsigned int i = 0; i < faces.size(); i++) {
      Vec3f normal2 = faces[i]->computeNormal();
      float area = faces[i]->getArea();
      if (normal.Dot3(normal2) < 0.5) continue;
      assert (area > 0);
      total += area;
      color += float(area) * getRadiance(faces[i]->getRadiosityPatchIndex());
    }
    assert (total > 0);
    color /= total;
    return color;
  } else if (args->mesh_data->render_mode == RENDER_LIGHTS) {
    return f->getMaterial()->getEmittedColor();
  } else if (args->mesh_data->render_mode == RENDER_UNDISTRIBUTED) { 
    return getUndistributed(i);
  } else if (args->mesh_data->render_mode == RENDER_ABSORBED) {
    return getAbsorbed(i);
  } else if (args->mesh_data->render_mode == RENDER_RADIANCE) {
    return getRadiance(i);
  } else if (args->mesh_data->render_mode == RENDER_FORM_FACTORS) {
    if (formfactors == NULL) ComputeFormFactors();
    float scale = 0.2 * total_area/getArea(i);
    float factor = scale * getFormFactor(max_undistributed_patch,i);
    return Vec3f(factor,factor,factor);
  } else {
    assert(0);
  }
  exit(0);
}

// =======================================================================================

int Radiosity::triCount() {
  return 12*num_faces;
}

void Radiosity::packMesh(float* &current) {
  
  for (int i = 0; i < num_faces; i++) {
    Face *f = mesh->getFace(i);
    Vec3f normal = f->computeNormal();

    //double avg_s = 0;
    //double avg_t = 0;

    // wireframe is normally black, except when it's the special
    // patch, then the wireframe is red
    Vec3f wireframe_color(0,0,0);
    if (args->mesh_data->render_mode == RENDER_FORM_FACTORS && i == max_undistributed_patch) {
      wireframe_color = Vec3f(1,0,0);
    }

    // 4 corner vertices
    Vec3f a_pos = ((*f)[0])->get();
    Vec3f a_color = setupHelperForColor(f,i,0);
    a_color = Vec3f(linear_to_srgb(a_color.r()),linear_to_srgb(a_color.g()),linear_to_srgb(a_color.b()));
    Vec3f b_pos = ((*f)[1])->get();
    Vec3f b_color = setupHelperForColor(f,i,1);
    b_color = Vec3f(linear_to_srgb(b_color.r()),linear_to_srgb(b_color.g()),linear_to_srgb(b_color.b()));
    Vec3f c_pos = ((*f)[2])->get();
    Vec3f c_color = setupHelperForColor(f,i,2);
    c_color = Vec3f(linear_to_srgb(c_color.r()),linear_to_srgb(c_color.g()),linear_to_srgb(c_color.b()));
    Vec3f d_pos = ((*f)[3])->get();
    Vec3f d_color = setupHelperForColor(f,i,3);
    d_color = Vec3f(linear_to_srgb(d_color.r()),linear_to_srgb(d_color.g()),linear_to_srgb(d_color.b()));

    Vec3f avg_color = 0.25f * (a_color+b_color+c_color+d_color);
    
    // the centroid (for wireframe rendering)
    Vec3f centroid = f->computeCentroid();
    
    AddWireFrameTriangle(current,
                         a_pos,b_pos,centroid,
                         normal,normal,normal,
                         wireframe_color,
                         a_color,b_color,avg_color);
    AddWireFrameTriangle(current,
                         b_pos,c_pos,centroid,
                         normal,normal,normal,
                         wireframe_color,
                         b_color,c_color,avg_color);
    AddWireFrameTriangle(current,
                         c_pos,d_pos,centroid,
                         normal,normal,normal,
                         wireframe_color,
                         c_color,d_color,avg_color);
    AddWireFrameTriangle(current,
                         d_pos,a_pos,centroid,
                         normal,normal,normal,
                         wireframe_color,
                         d_color,a_color,avg_color);

  }
}

// =======================================================================================
