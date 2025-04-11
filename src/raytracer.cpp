#include "raytracer.h"
#include "material.h"
#include "argparser.h"
#include "raytree.h"
#include "utils.h"
#include "mesh.h"
#include "face.h"
#include "primitive.h"
#include "photon_mapping.h"
#include "boundingbox.h"
#include "camera.h"

#include <random>

#include "mpm.h"

// ===========================================================================
// casts a single ray through the scene geometry and finds the closest hit
bool RayTracer::CastRay(const Ray &ray, Hit &h, bool use_rasterized_patches) const {
  bool answer = false;

  // intersect each of the quads
  for (int i = 0; i < mesh->numOriginalQuads(); i++) {
    Face *f = mesh->getOriginalQuad(i);
    if (f->intersect(ray,h,args->mesh_data->intersect_backfacing)) answer = true;
  }

  // intersect each of the primitives (either the patches, or the original primitives)
  if (use_rasterized_patches) {
    for (int i = 0; i < mesh->numRasterizedPrimitiveFaces(); i++) {
      Face *f = mesh->getRasterizedPrimitiveFace(i);
      if (f->intersect(ray,h,args->mesh_data->intersect_backfacing)) answer = true;
    }
  } else {
    int num_primitives = mesh->numPrimitives();
    for (int i = 0; i < num_primitives; i++) {
      if (mesh->getPrimitive(i)->intersect(ray,h)) answer = true;
    }
  }

  if (GLOBAL_args->mpm->intersect(ray, h)) answer = true;

  return answer;
}

// ===========================================================================
// does the recursive (shadow rays & recursive rays) work
Vec3f RayTracer::TraceRay(Ray &ray, Hit &hit, int bounce_count) const {

  // First cast a ray and see if we hit anything.
  hit = Hit();
  bool intersect = CastRay(ray,hit,false);
  if (bounce_count < args->mesh_data->num_bounces) {
      RayTree::AddReflectedSegment(ray, 0, hit.getT());
  }
    
  // if there is no intersection, simply return the background color
  if (intersect == false) {
    return Vec3f(srgb_to_linear(mesh->background_color.r()),
                 srgb_to_linear(mesh->background_color.g()),
                 srgb_to_linear(mesh->background_color.b()));
  }

  // otherwise decide what to do based on the material
  Material *m = hit.getMaterial();
  assert (m != NULL);

  // rays coming from the light source are set to white, don't bother to ray trace further.
  if (m->getEmittedColor().Length() > 0.001) {
    return Vec3f(1,1,1);
  } 
 
  
  Vec3f normal = hit.getNormal();
  Vec3f point = ray.pointAtParameter(hit.getT());
  Vec3f answer;

  Vec3f ambient_light = Vec3f(args->mesh_data->ambient_light.data[0],
                              args->mesh_data->ambient_light.data[1],
                              args->mesh_data->ambient_light.data[2]);
  
  // ----------------------------------------------
  //  start with the indirect light (ambient light)
  Vec3f diffuse_color = m->getDiffuseColor(hit.get_s(),hit.get_t());
  if (args->mesh_data->gather_indirect) {
    // photon mapping for more accurate indirect light
    answer = diffuse_color * (photon_mapping->GatherIndirect(point, normal, ray.getDirection()) + ambient_light);
  } else {
    // the usual ray tracing hack for indirect light
    answer = diffuse_color * ambient_light;
  }      

  // ----------------------------------------------
  // add contributions from each light that is not in shadow
  int num_lights = mesh->getLights().size();
  for (int i = 0; i < num_lights; i++) {

    Face *f = mesh->getLights()[i];
    Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
    Vec3f myLightColor;
    Vec3f lightCentroid = f->computeCentroid();
    int ss = args->mesh_data->num_shadow_samples;
    Vec3f modifier;
    if (ss == 0) {
        modifier = Vec3f(1.0, 1.0, 1.0);
    } else {
        modifier = Vec3f(1.0/ss, 1.0/ss, 1.0/ss);
    }

    if (ss == 0) { 
        ss++;
        Vec3f dirToLightCentroid = lightCentroid - point;
        dirToLightCentroid.Normalize();

        float distToLightCentroid = (lightCentroid - point).Length();
        myLightColor = 1 / float(M_PI * distToLightCentroid * distToLightCentroid) * lightColor;
        std::cout << "color: " << myLightColor << std::endl;
        answer += m->Shade(ray, hit, dirToLightCentroid, myLightColor) * modifier;
        continue;
    }

    for (int s = 0; s < ss; s++) {
        if (s > 0) {
            lightCentroid = f->RandomPoint();
        }
        Vec3f dirToLightCentroid = lightCentroid - point;
        dirToLightCentroid.Normalize();



        // ===========================================
        // ASSIGNMENT:  ADD SHADOW & SOFT SHADOW LOGIC
        // ===========================================


        float distToLightCentroid = (lightCentroid - point).Length();
        myLightColor = 1 / float(M_PI * distToLightCentroid * distToLightCentroid) * lightColor;



        // add the lighting contribution from this particular light at this point

        // cast ray from point to light
        Ray to_source(point, dirToLightCentroid);
        Hit source_hit = Hit();
        bool blocked = CastRay(to_source, source_hit, false);
        RayTree::AddShadowSegment(to_source, 0, source_hit.getT());

        // if nothing blocking light, add contribution from it
        if (blocked) {
            Material* source_m = source_hit.getMaterial();
            assert(source_m != NULL);
            Vec3f blocking_point = to_source.pointAtParameter(source_hit.getT());
            if (source_m->getEmittedColor().Length() > 0.001 && (blocking_point - lightCentroid).Length() < 0.001) {
                answer += m->Shade(ray, hit, dirToLightCentroid, myLightColor) * modifier;
            }
        }
        else {
            answer += m->Shade(ray, hit, dirToLightCentroid, myLightColor) * modifier;
        }
    }
  }

      
  // ----------------------------------------------
  // add contribution from reflection, if the surface is shiny

  // return answer if surface not reflective
  Vec3f reflectiveColor = m->getReflectiveColor();
  if (reflectiveColor.Length() < 0.001) { return answer; }

  
  // =================================
  // ASSIGNMENT:  ADD REFLECTIVE LOGIC
  // =================================

  // find next direction
  Vec3f last_dir = ray.getDirection();
  last_dir.Normalize();
  Vec3f last_normal = hit.getNormal();
  last_normal.Normalize();
  Vec3f new_dir = last_dir - (2.0 * (last_dir.Dot3(last_normal) * last_normal));
  new_dir.Normalize();
  Ray reflection(ray.pointAtParameter(hit.getT()), new_dir);
  Hit reflection_hit = Hit();

  // recursively trace if bounces are left
  if (bounce_count > 0) {
      answer += (reflectiveColor * TraceRay(reflection, reflection_hit, bounce_count - 1));
  }
  
  // return final value
  return answer; 

}


// bidirectional path tracing - 1 of each type, match each light endposition to each eye endposition
Vec3f RayTracer::BiTraceRay(Ray& ray, Hit& hit, std::vector<LightEndpoint> lights, int bounce_count) const {
    // First cast a ray and see if we hit anything.
    hit = Hit();
    bool intersect = CastRay(ray, hit, false);
    if (bounce_count < args->mesh_data->num_bounces) {
        RayTree::AddReflectedSegment(ray, 0, hit.getT());
    }

    // if there is no intersection, simply return the background color
    if (intersect == false) {
        return Vec3f(srgb_to_linear(mesh->background_color.r()),
            srgb_to_linear(mesh->background_color.g()),
            srgb_to_linear(mesh->background_color.b()));
    }

    // otherwise decide what to do based on the material
    Material* m = hit.getMaterial();
    assert(m != NULL);

    // rays coming from the light source are set to white, don't bother to ray trace further.
    if (m->getEmittedColor().Length() > 0.001) {
        return Vec3f(1, 1, 1);
    }


    Vec3f normal = hit.getNormal();
    Vec3f point = ray.pointAtParameter(hit.getT());
    Vec3f answer;

    Vec3f ambient_light = Vec3f(args->mesh_data->ambient_light.data[0],
        args->mesh_data->ambient_light.data[1],
        args->mesh_data->ambient_light.data[2]);

    // ----------------------------------------------
    //  start with the indirect light (ambient light)
    Vec3f diffuse_color = m->getDiffuseColor(hit.get_s(), hit.get_t());
    if (args->mesh_data->gather_indirect) {
        // photon mapping for more accurate indirect light
        answer = diffuse_color * (photon_mapping->GatherIndirect(point, normal, ray.getDirection()) + ambient_light);
    }
    else {
        // the usual ray tracing hack for indirect light
        answer = diffuse_color * ambient_light;
    }

    // ----------------------------------------------
    // add contributions from each light that is not in shadow
    for (std::vector<LightEndpoint>::iterator itr = lights.begin(); itr != lights.end(); itr++) {

        Vec3f lightColor = itr->color;
        Vec3f myLightColor;
        Vec3f lightCentroid = itr->position;
        int ss = args->mesh_data->num_shadow_samples;


        Vec3f dirToLightCentroid = lightCentroid - point;
        dirToLightCentroid.Normalize();



        // ===========================================
        // ASSIGNMENT:  ADD SHADOW & SOFT SHADOW LOGIC
        // ===========================================


        float distToLightCentroid = (lightCentroid - point).Length();
        myLightColor = 1 / float(M_PI * distToLightCentroid * distToLightCentroid) * lightColor;

        //std::cout << "color: " << myLightColor << std::endl;

        // add the lighting contribution from this particular light at this point

        // cast ray from point to light
        Ray to_source(point, dirToLightCentroid);
        Hit source_hit = Hit();
        bool blocked = CastRay(to_source, source_hit, false);
        RayTree::AddShadowSegment(to_source, 0, source_hit.getT());

        // if nothing blocking light, add contribution from it
        if (blocked) {
            Vec3f blocking_point = to_source.pointAtParameter(source_hit.getT());
            if ((blocking_point - lightCentroid).Length() < 0.001) {
                answer += m->Shade(ray, hit, dirToLightCentroid, myLightColor);
            }
        }
        else {
            answer += m->Shade(ray, hit, dirToLightCentroid, myLightColor);
        }

    }


    // ----------------------------------------------
    // add contribution from reflection, if the surface is shiny

    // return answer if surface not reflective
    Vec3f reflectiveColor = m->getReflectiveColor();
    if (reflectiveColor.Length() < 0.001) { return answer; }


    // =================================
    // ASSIGNMENT:  ADD REFLECTIVE LOGIC
    // =================================

    // find next direction
    Vec3f last_dir = ray.getDirection();
    last_dir.Normalize();
    Vec3f last_normal = hit.getNormal();
    last_normal.Normalize();
    Vec3f new_dir = last_dir - (2.0 * (last_dir.Dot3(last_normal) * last_normal));
    new_dir.Normalize();
    Ray reflection(ray.pointAtParameter(hit.getT()), new_dir);
    Hit reflection_hit = Hit();

    // recursively trace if bounces are left
    if (bounce_count > 0) {
        answer += (reflectiveColor * BiTraceRay(reflection, reflection_hit, lights, bounce_count - 1));
    }

    // return final value
    return answer;
}




// trace a ray through pixel (i,j) of the image an return the color
Vec3f VisualizeTraceRay(double i, double j) {

  // compute and set the pixel color
  int max_d = mymax(GLOBAL_args->mesh_data->width,GLOBAL_args->mesh_data->height);
  Vec3f color;
  

  double max_rand = 0.5;

  // ==================================
  // ASSIGNMENT: IMPLEMENT ANTIALIASING
  // ==================================

  int as = GLOBAL_args->mesh_data->num_antialias_samples;

  for (int a = 0; a < as; a++) {

      std::vector<LightEndpoint> light_spots;

      int num_lights = GLOBAL_args->mesh->getLights().size();
      //std::cout << "generating lights" << std::endl;
      for (int i = 0; i < num_lights; i++) {

          Face* f = GLOBAL_args->mesh->getLights()[i];
          LightEndpoint light = LightEndpoint();
          Vec3f cur_color = f->getMaterial()->getEmittedColor() * f->getArea();
          light.color = cur_color;
          Vec3f cur_position = f->computeCentroid();
          light.position = cur_position;
          light_spots.push_back(light);
          Vec3f cur_dir = f->computeNormal() + Vec3f(max_rand * ((GLOBAL_args->rand() - 0.5) * 2), max_rand * ((GLOBAL_args->rand() - 0.5) * 2), max_rand * ((GLOBAL_args->rand() - 0.5) * 2));


          int count = 0;
          while (count < 10) {  //GLOBAL_args->mesh_data->num_bounces

              // Cast next light spot
              Hit hit = Hit();
              Ray ray = Ray(cur_position, cur_dir);
              bool intersect = GLOBAL_args->raytracer->CastRay(ray, hit, false);
              RayTree::AddReflectedSegment(ray, 0, hit.getT());

              // if there is no intersection, break
              if (intersect == false) {
                  //std::cout << "off to background, t = " << hit.getT() << std::endl;
                  break;
              }
              LightEndpoint cur_light;


              Material* m = hit.getMaterial();
              if (m->getReflectiveColor().Length() > 0.001) {
                  cur_color.setx(cur_color.x() * m->getReflectiveColor().x());
                  cur_color.sety(cur_color.y() * m->getReflectiveColor().y());
                  cur_color.setz(cur_color.z() * m->getReflectiveColor().z());
              }
              else {
                  cur_color.setx(cur_color.x() * m->getDiffuseColor().x() * m->getRoughness());
                  cur_color.sety(cur_color.y() * m->getDiffuseColor().y() * m->getRoughness());
                  cur_color.setz(cur_color.z() * m->getDiffuseColor().z() * m->getRoughness());
              }
              cur_light.color = cur_color;

              cur_position = ray.pointAtParameter(hit.getT());
              cur_light.position = cur_position;

              light_spots.push_back(cur_light);

              cur_dir = hit.getNormal();

              cur_dir += Vec3f(max_rand * ((GLOBAL_args->rand() - 0.5) * 2), max_rand * ((GLOBAL_args->rand() - 0.5) * 2), max_rand * ((GLOBAL_args->rand() - 0.5) * 2));

              count++;
          }
      }
      //std::cout << light_spots.size() << " lights" << std::endl;

  
      // Here's what we do with a single sample per pixel:
      // construct & trace a ray through the center of the pixel
      double x_offset = 0.5;
      double y_offset = 0.5;
      if (a != 0) {
          x_offset = 0.5 + (0.01 * (0.5 - GLOBAL_args->rand()));
          y_offset = 0.5 + (0.01 * (0.5 - GLOBAL_args->rand()));
      }
      //std::cout << x_offset << ", " << y_offset << std::endl;
      double x = (i - GLOBAL_args->mesh_data->width / 2.0) / double(max_d) + x_offset;
      double y = (j - GLOBAL_args->mesh_data->height / 2.0) / double(max_d) + y_offset;
      Ray r = GLOBAL_args->mesh->camera->generateRay(x, y);
      Hit hit;
      color += GLOBAL_args->raytracer->BiTraceRay(r, hit, light_spots, GLOBAL_args->mesh_data->num_bounces) * Vec3f(1.0/as, 1.0/as, 1.0/as);
      //color += GLOBAL_args->raytracer->TraceRay(r, hit, GLOBAL_args->mesh_data->num_bounces) * Vec3f(1.0/as, 1.0/as, 1.0/as);
      // add that ray for visualization
      RayTree::AddMainSegment(r, 0, hit.getT());
  }


  // return the color
  return color;
}




// for visualization: find the "corners" of a pixel on an image plane
// 1/2 way between the camera & point of interest
Vec3f PixelGetPos(double i, double j) {
  int max_d = mymax(GLOBAL_args->mesh_data->width,GLOBAL_args->mesh_data->height);
  double x = (i-GLOBAL_args->mesh_data->width/2.0)/double(max_d)+0.5;
  double y = (j-GLOBAL_args->mesh_data->height/2.0)/double(max_d)+0.5;
  Camera *camera = GLOBAL_args->mesh->camera;
  Ray r = camera->generateRay(x,y); 
  Vec3f cp = camera->camera_position;
  Vec3f poi = camera->point_of_interest;
  float distance = (cp-poi).Length()/2.0f;
  return r.getOrigin()+distance*r.getDirection();
}





// Scan through the image from the lower left corner across each row
// and then up to the top right.  Initially the image is sampled very
// coarsely.  Increment the static variables that track the progress
// through the scans
int RayTraceDrawPixel() {
  if (GLOBAL_args->mesh_data->raytracing_x >= GLOBAL_args->mesh_data->raytracing_divs_x) {
    // end of row
    GLOBAL_args->mesh_data->raytracing_x = 0; 
    GLOBAL_args->mesh_data->raytracing_y += 1;
  }
  if (GLOBAL_args->mesh_data->raytracing_y >= GLOBAL_args->mesh_data->raytracing_divs_y) {
    // last row
    if (GLOBAL_args->mesh_data->raytracing_divs_x >= GLOBAL_args->mesh_data->width ||
        GLOBAL_args->mesh_data->raytracing_divs_y >= GLOBAL_args->mesh_data->height) {
      // stop rendering, matches resolution of current camera
      return 0; 
    }
    // else decrease pixel size & start over again in the bottom left corner
    GLOBAL_args->mesh_data->raytracing_divs_x *= 3;
    GLOBAL_args->mesh_data->raytracing_divs_y *= 3;
    if (GLOBAL_args->mesh_data->raytracing_divs_x > GLOBAL_args->mesh_data->width * 0.51 ||
        GLOBAL_args->mesh_data->raytracing_divs_x > GLOBAL_args->mesh_data->height * 0.51) {
      GLOBAL_args->mesh_data->raytracing_divs_x = GLOBAL_args->mesh_data->width;
      GLOBAL_args->mesh_data->raytracing_divs_y = GLOBAL_args->mesh_data->height;
    }
    GLOBAL_args->mesh_data->raytracing_x = 0;
    GLOBAL_args->mesh_data->raytracing_y = 0;

    if (GLOBAL_args->raytracer->render_to_a) {
      GLOBAL_args->raytracer->pixels_b.clear();
      GLOBAL_args->raytracer->render_to_a = false;
    } else {
      GLOBAL_args->raytracer->pixels_a.clear();
      GLOBAL_args->raytracer->render_to_a = true;
    }
  }

  double x_spacing = GLOBAL_args->mesh_data->width / double (GLOBAL_args->mesh_data->raytracing_divs_x);
  double y_spacing = GLOBAL_args->mesh_data->height / double (GLOBAL_args->mesh_data->raytracing_divs_y);

  // compute the color and position of intersection
  Vec3f pos1 =  PixelGetPos((GLOBAL_args->mesh_data->raytracing_x  )*x_spacing, (GLOBAL_args->mesh_data->raytracing_y  )*y_spacing);
  Vec3f pos2 =  PixelGetPos((GLOBAL_args->mesh_data->raytracing_x+1)*x_spacing, (GLOBAL_args->mesh_data->raytracing_y  )*y_spacing);
  Vec3f pos3 =  PixelGetPos((GLOBAL_args->mesh_data->raytracing_x+1)*x_spacing, (GLOBAL_args->mesh_data->raytracing_y+1)*y_spacing);
  Vec3f pos4 =  PixelGetPos((GLOBAL_args->mesh_data->raytracing_x  )*x_spacing, (GLOBAL_args->mesh_data->raytracing_y+1)*y_spacing);

  Vec3f color = VisualizeTraceRay((GLOBAL_args->mesh_data->raytracing_x+0.5)*x_spacing, (GLOBAL_args->mesh_data->raytracing_y+0.5)*y_spacing);

  double r = linear_to_srgb(color.r());
  double g = linear_to_srgb(color.g());
  double b = linear_to_srgb(color.b());

  Pixel p;
  p.v1 = pos1;
  p.v2 = pos2;
  p.v3 = pos3;
  p.v4 = pos4;
  p.color = Vec3f(r,g,b);
    
  if (GLOBAL_args->raytracer->render_to_a) {
    GLOBAL_args->raytracer->pixels_a.push_back(p);
  } else {
    GLOBAL_args->raytracer->pixels_b.push_back(p);
  }  

  GLOBAL_args->mesh_data->raytracing_x += 1;
  return 1;
}

// ===========================================================================

int RayTracer::triCount() {
  int count = (pixels_a.size() + pixels_b.size()) * 2;
  return count;
}

void RayTracer::packMesh(float* &current) {
  for (unsigned int i = 0; i < pixels_a.size(); i++) {
    Pixel &p = pixels_a[i];
    Vec3f v1 = p.v1;
    Vec3f v2 = p.v2;
    Vec3f v3 = p.v3;
    Vec3f v4 = p.v4;
    Vec3f normal = ComputeNormal(v1,v2,v3) + ComputeNormal(v1,v3,v4);
    normal.Normalize();
    if (render_to_a) {
      v1 += 0.02*normal;
      v2 += 0.02*normal;
      v3 += 0.02*normal;
      v4 += 0.02*normal;
    }
    normal = Vec3f(0,0,0);
    AddQuad(current,v1,v2,v3,v4,normal,p.color);
  }

  for (unsigned int i = 0; i < pixels_b.size(); i++) {
    Pixel &p = pixels_b[i];
    Vec3f v1 = p.v1;
    Vec3f v2 = p.v2;
    Vec3f v3 = p.v3;
    Vec3f v4 = p.v4;
    Vec3f normal = ComputeNormal(v1,v2,v3) + ComputeNormal(v1,v3,v4);
    normal.Normalize();
    if (!render_to_a) {
      v1 += 0.02*normal;
      v2 += 0.02*normal;
      v3 += 0.02*normal;
      v4 += 0.02*normal;
    }
    normal = Vec3f(0,0,0);
    AddQuad(current,v1,v2,v3,v4,normal,p.color);
  }
}

// ===========================================================================
