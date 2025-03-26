#include "raytree.h"
#include "utils.h"

// ====================================================================
// Initialize the static variables
int RayTree::activated = 0;  
std::vector<Segment> RayTree::main_segments;
std::vector<Segment> RayTree::shadow_segments;
std::vector<Segment> RayTree::reflected_segments;
std::vector<Segment> RayTree::transmitted_segments;

int RayTree::triCount() {
  return numSegments()*12;
}

void addBox(float* &current, Vec3f start, Vec3f end, Vec3f color, float width) {

  // find perpendicular axes
  Vec3f dir = (end-start);
  Vec3f one;
  Vec3f two;
  if (dir.Length() < 0.01*width) {
    dir = one = two = Vec3f(0,0,0);
  } else {
    dir.Normalize(); 
    Vec3f tmp; Vec3f::Cross3(tmp,dir,Vec3f(1,0,0));
    if (tmp.Length() < 0.1) {
      Vec3f::Cross3(tmp,dir,Vec3f(0,0,1));
    }
    tmp.Normalize();
    Vec3f::Cross3(one,dir,tmp);
    assert (fabs(one.Length()-1.0) < 0.001);
    Vec3f::Cross3(two,dir,one);
    assert (fabs(two.Length()-1.0) < 0.001);
  }

  Vec3f pos[8] = { Vec3f(start + width*one + width*two),
                   Vec3f(start + width*one - width*two),
                   Vec3f(start - width*one + width*two),
                   Vec3f(start - width*one - width*two),
                   Vec3f(end   + width*one + width*two),
                   Vec3f(end   + width*one - width*two),
                   Vec3f(end   - width*one + width*two),
                   Vec3f(end   - width*one - width*two) };

  AddBox(current,pos,color);
}

void RayTree::packMesh(float* &current) {

  Vec3f main_color(0.7,0.7,0.7);
  Vec3f shadow_color(0.1,0.9,0.1);
  Vec3f reflected_color(0.9,0.1,0.1);
  Vec3f transmitted_color(0.1,0.1,0.9);

  float width = 0.01;
  
  unsigned int i;
  for (i = 0; i < main_segments.size(); i++) {
    addBox(current,
           main_segments[i].getStart(),
           main_segments[i].getEnd(),
           main_color,width);
  }
  for (i = 0; i < shadow_segments.size(); i++) {
    addBox(current,
           shadow_segments[i].getStart(),
           shadow_segments[i].getEnd(),
           shadow_color,width);
  }
  for (i = 0; i < reflected_segments.size(); i++) {
    addBox(current,
           reflected_segments[i].getStart(),
           reflected_segments[i].getEnd(),
           reflected_color,width);
  }
  for (i = 0; i < transmitted_segments.size(); i++) {
    addBox(current,
           transmitted_segments[i].getStart(),
           transmitted_segments[i].getEnd(),
           transmitted_color,width);
  }
}

// ===========================================================================
