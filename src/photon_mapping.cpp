#include <iostream>
#include <algorithm>
#include <cstring>
#include <set>

#include "argparser.h"
#include "photon_mapping.h"
#include "mesh.h"
#include "face.h"
#include "primitive.h"
#include "kdtree.h"
#include "utils.h"
#include "raytracer.h"


// ==========
// Clear/reset
void PhotonMapping::Clear() {
  // cleanup all the photons
  delete kdtree;
  kdtree = NULL;
}


// ========================================================================
// Recursively trace a single photon

void PhotonMapping::TracePhoton(const Vec3f &position, const Vec3f &direction,
				const Vec3f &energy, int iter) {

    // ==============================================
    // ASSIGNMENT: IMPLEMENT RECURSIVE PHOTON TRACING
    // ==============================================

    // Trace the photon through the scene.  At each diffuse or
    // reflective bounce, store the photon in the kd tree.

    // One optimization is to *not* store the first bounce, since that
    // direct light can be efficiently computed using classic ray
    // tracing.


    Ray trace(position, direction);
    Hit trace_hit = Hit();

    bool hit = raytracer->CastRay(trace, trace_hit, false);

    if (!hit) { return; }

    // add to tree
    Vec3f hit_position = trace.pointAtParameter(trace_hit.getT());
    
    
    // don't add first hit
    if (iter > 0) {
        kdtree->AddPhoton(Photon(hit_position, direction, energy, iter));
    }

    // trace next photon
    Vec3f new_direction(0, 0, 0);
    Vec3f new_energy(0, 0, 0);

    Material* hit_material = trace_hit.getMaterial();

    if (hit_material->getReflectiveColor().Length() > 0.0001) {
        // reflective surface, direct reflection
        Vec3f last_normal = trace_hit.getNormal();
        new_direction = direction - (2.0 * (direction.Dot3(last_normal) * last_normal));
        new_direction.Normalize();
        new_energy = energy * hit_material->getReflectiveColor();

    } else {
        // diffuse surface, random reflection
        float new_x = trace_hit.getNormal().x();
        float new_y = trace_hit.getNormal().y();
        float new_z = trace_hit.getNormal().z();

        float x = (GLOBAL_args->rand() * 2) - 1;
        float y = (GLOBAL_args->rand() * 2) - 1;
        float z = (GLOBAL_args->rand() * 2) - 1;

        new_direction = Vec3f(new_x + x, new_y + y, new_z + z);
        new_energy = energy * hit_material->getDiffuseColor();
    }

    new_direction.Normalize();

    // make the trace if energy is still above a threshold
    if (new_energy.Length() > DBL_EPSILON) {
        TracePhoton(hit_position, new_direction, new_energy, iter + 1);
    }
}


// ========================================================================
// Trace the specified number of photons through the scene

void PhotonMapping::TracePhotons() {

  // first, throw away any existing photons
  delete kdtree;

  // consruct a kdtree to store the photons
  BoundingBox *bb = mesh->getBoundingBox();
  Vec3f min = bb->getMin();
  Vec3f max = bb->getMax();
  Vec3f diff = max-min;
  min -= 0.001f*diff;
  max += 0.001f*diff;
  kdtree = new KDTree(BoundingBox(min,max));

  // photons emanate from the light sources
  const std::vector<Face*>& lights = mesh->getLights();

  // compute the total area of the lights
  float total_lights_area = 0;
  for (unsigned int i = 0; i < lights.size(); i++) {
    total_lights_area += lights[i]->getArea();
  }

  // shoot a constant number of photons per unit area of light source
  // (alternatively, this could be based on the total energy of each light)
  for (unsigned int i = 0; i < lights.size(); i++) {  
    float my_area = lights[i]->getArea();
    int num = args->mesh_data->num_photons_to_shoot * my_area / total_lights_area;
    // the initial energy for this photon
    Vec3f energy = my_area/float(num) * lights[i]->getMaterial()->getEmittedColor();
    Vec3f normal = lights[i]->computeNormal();
    for (int j = 0; j < num; j++) {
      Vec3f start = lights[i]->RandomPoint();
      // the initial direction for this photon (for diffuse light sources)
      Vec3f direction = RandomDiffuseDirection(normal);
      TracePhoton(start,direction,energy,0);
    }
  }
  std::cout << "total photons: " << kdtree->numPhotons() << std::endl;
}


// ======================================================================

// helper function
bool closest_photon(const std::pair<Photon,float> &a, const std::pair<Photon,float> &b) {
  return (a.second < b.second);
}


// ======================================================================
Vec3f PhotonMapping::GatherIndirect(const Vec3f &point, const Vec3f &normal,
                                    const Vec3f &direction_from) const {

    if (kdtree == NULL) { 
        std::cout << "WARNING: Photons have not been traced throughout the scene." << std::endl;
        return Vec3f(0,0,0); 
    }

    // ================================================================
    // ASSIGNMENT: GATHER THE INDIRECT ILLUMINATION FROM THE PHOTON MAP
    // ================================================================

    // collect the closest args->mesh_data->num_photons_to_collect photons
    // determine the radius that was necessary to collect that many photons
    // average the energy of those photons over that radius

    bool found_photons = false;
    double radius = 0.1;
    Vec3f color(0, 0, 0);

    // loop until the photons have been gathered
    while (!found_photons) {
        // Need a smart initial radius guess, currently just using radius value

        // storing photons
        std::vector<Photon> in_box;
        std::unordered_map<double, Photon*> mapping;
        std::set<double> in_radius;

        // create bounding box based on radius
        Vec3f min(point.x() - radius, point.y() - radius, point.z() - radius);
        Vec3f max(point.x() + radius, point.y() + radius, point.z() + radius);
        BoundingBox bb(min, max);


        // get photons in bounding box
        kdtree->CollectPhotonsInBox(bb, in_box);

        // if not enough collected, increase radius and try again
        if (in_box.size() < args->mesh_data->num_photons_to_collect){
            radius *= 2;
            continue;
        }

        // gather all those *actually* in radius
        for (std::vector<Photon>::iterator itr = in_box.begin(); itr != in_box.end(); itr++) {
            Photon p = *itr;
            double distance = (p.getPosition() - point).Length();
            if (distance <= radius && direction_from.Dot3(p.getDirectionFrom()) > 0){ // simple direction check, could be made much more involved
                Photon* ptr= NULL;
                ptr = &p;
                mapping[distance] = ptr;
                in_radius.insert(distance);
            }
        }

        if (in_radius.size() >= args->mesh_data->num_photons_to_collect) {
            // gather num_photons_to_collect closest and determine radius
            std::set<double>::iterator itr = in_radius.begin();
            radius = 0;
            for (int i = 0; i < args->mesh_data->num_photons_to_collect; i++) {
                color += mapping[*itr]->getEnergy();
                radius = std::max(radius, *itr);
                itr++;
            }
            found_photons = true;
        } else {
            // not enough *actually* found, increase radius and try again
            radius *= 2;
        }
    }

    // return the color (scaled by area of circle)
    color *= (1.0 / (M_PI * radius * radius));
    return color;
}

// ======================================================================
// ======================================================================
// Helper functions to render the photons & kdtree

int PhotonMapping::triCount() const {
  int tri_count = 0;
  if (GLOBAL_args->mesh_data->render_kdtree == true && kdtree != NULL) 
    tri_count += kdtree->numBoxes()*12*12;
  if (GLOBAL_args->mesh_data->render_photon_directions == true && kdtree != NULL) 
    tri_count += kdtree->numPhotons()*12;
  return tri_count;
}

int PhotonMapping::pointCount() const {
  if (GLOBAL_args->mesh_data->render_photons == false || kdtree == NULL) return 0;
  return kdtree->numPhotons();
}

// defined in raytree.cpp
void addBox(float* &current, Vec3f start, Vec3f end, Vec3f color, float width);

// ======================================================================

void packKDTree(const KDTree *kdtree, float* &current, int &count) {
  if (!kdtree->isLeaf()) {
    if (kdtree->getChild1() != NULL) { packKDTree(kdtree->getChild1(),current,count); }
    if (kdtree->getChild2() != NULL) { packKDTree(kdtree->getChild2(),current,count); }
  } else {

    Vec3f a = kdtree->getMin();
    Vec3f b = kdtree->getMax();

    Vec3f corners[8] = { Vec3f(a.x(),a.y(),a.z()),
                         Vec3f(a.x(),a.y(),b.z()),
                         Vec3f(a.x(),b.y(),a.z()),
                         Vec3f(a.x(),b.y(),b.z()),
                         Vec3f(b.x(),a.y(),a.z()),
                         Vec3f(b.x(),a.y(),b.z()),
                         Vec3f(b.x(),b.y(),a.z()),
                         Vec3f(b.x(),b.y(),b.z()) };

    float width = 0.01 * (a-b).Length();
    
    addBox(current,corners[0],corners[1],Vec3f(1,1,0),width);
    addBox(current,corners[1],corners[3],Vec3f(1,1,0),width);
    addBox(current,corners[3],corners[2],Vec3f(1,1,0),width);
    addBox(current,corners[2],corners[0],Vec3f(1,1,0),width);

    addBox(current,corners[4],corners[5],Vec3f(1,1,0),width);
    addBox(current,corners[5],corners[7],Vec3f(1,1,0),width);
    addBox(current,corners[7],corners[6],Vec3f(1,1,0),width);
    addBox(current,corners[6],corners[4],Vec3f(1,1,0),width);

    addBox(current,corners[0],corners[4],Vec3f(1,1,0),width);
    addBox(current,corners[1],corners[5],Vec3f(1,1,0),width);
    addBox(current,corners[2],corners[6],Vec3f(1,1,0),width);
    addBox(current,corners[3],corners[7],Vec3f(1,1,0),width);
    
    count++;
  }
}

// ======================================================================

void packPhotons(const KDTree *kdtree, float* &current_points, int &count) {
  if (!kdtree->isLeaf()) {
      if (kdtree->getChild1() != NULL) { packPhotons(kdtree->getChild1(),current_points,count); }
      if (kdtree->getChild2() != NULL) { packPhotons(kdtree->getChild2(),current_points,count); }
  } else {
    for (unsigned int i = 0; i < kdtree->getPhotons().size(); i++) {
      const Photon &p = kdtree->getPhotons()[i];
      Vec3f v = p.getPosition();
      Vec3f color = p.getEnergy()*float(GLOBAL_args->mesh_data->num_photons_to_shoot);
      float12 t = { float(v.x()),float(v.y()),float(v.z()),1,   0,0,0,0,   float(color.r()),float(color.g()),float(color.b()),1 };
      memcpy(current_points, &t, sizeof(float)*12); current_points += 12; 
      count++;
    }
  }
}


void packPhotonDirections(const KDTree *kdtree, float* &current, int &count) {
  if (!kdtree->isLeaf()) {
      if (kdtree->getChild1() != NULL) { packPhotonDirections(kdtree->getChild1(),current,count); }
      if (kdtree->getChild2() != NULL) { packPhotonDirections(kdtree->getChild2(),current,count); }
  } else {
    for (unsigned int i = 0; i < kdtree->getPhotons().size(); i++) {
      const Photon &p = kdtree->getPhotons()[i];
      Vec3f v = p.getPosition();
      Vec3f v2 = p.getPosition() - p.getDirectionFrom() * 0.5;
      Vec3f color = p.getEnergy()*float(GLOBAL_args->mesh_data->num_photons_to_shoot);
      float width = 0.01;
      addBox(current,v,v2,color,width);
      count++;
    }
  }
}
  
// ======================================================================

void PhotonMapping::packMesh(float* &current, float* &current_points) {

  // the photons
  if (GLOBAL_args->mesh_data->render_photons && kdtree != NULL) {
    int count = 0;
    packPhotons(kdtree,current_points,count);
    assert (count == kdtree->numPhotons());
  }
  // photon directions
  if (GLOBAL_args->mesh_data->render_photon_directions && kdtree != NULL) {
    int count = 0;
    packPhotonDirections(kdtree,current,count);
    assert (count == kdtree->numPhotons());
  }

  // the wireframe kdtree
  if (GLOBAL_args->mesh_data->render_kdtree && kdtree != NULL) {
    int count = 0;
    packKDTree(kdtree,current,count);
    assert (count == kdtree->numBoxes());
  }
  
}

// ======================================================================
