#include <algorithm>

#include "camera.h"
#include "ray.h"
#include "utils.h"

// ====================================================================
// CONSTRUCTORS
// ====================================================================

Camera::Camera(const Vec3f &c, const Vec3f &poi, const Vec3f &u) {
  camera_position = c;
  point_of_interest = poi;
  up = u;
  up.Normalize();
}

OrthographicCamera::OrthographicCamera
(const Vec3f &c, const Vec3f &poi, const Vec3f &u, float s) 
  : Camera(c,poi,u) {
  size = s;
}

PerspectiveCamera::PerspectiveCamera
(const Vec3f &c, const Vec3f &poi, const Vec3f &u, float a) 
  : Camera(c,poi,u) {
  angle = a;
}

// ====================================================================
// dollyCamera: Move camera along the direction vector
// ====================================================================

void Camera::dollyCamera(float dist) {
  Vec3f diff = camera_position - point_of_interest;
  float d = diff.Length();
  Vec3f translate = float(0.004*d*dist)*getDirection();
  camera_position += translate;
}

// ====================================================================
// zoomCamera: Change the field of view/angle
// ====================================================================

void OrthographicCamera::zoomCamera(float factor) {
  size *= pow(1.003,factor);
}

void PerspectiveCamera::zoomCamera(float dist) {
  angle *= pow(1.002,dist);
  // put some reasonable limits on the camera angle (in degrees)
  if (angle < 5) angle = 5;
  if (angle > 175) angle = 175;
}

// ====================================================================
// truckCamera: Translate camera perpendicular to the direction vector
// ====================================================================

void Camera::truckCamera(float dx, float dy) {
  Vec3f diff = camera_position - point_of_interest;
  float d = diff.Length();
  Vec3f translate = (d*0.0007f)*(getHorizontal()*float(dx) + getScreenUp()*float(dy));
  camera_position += translate;
  point_of_interest += translate;
}

// ====================================================================
// rotateCamera: Rotate around the up and horizontal vectors
// ====================================================================

// adjust this number if desired
#define ROTATE_SPEED 0.2


float convertToRadians(float d) { return d * M_PI / 180.0; }


void Camera::rotateCamera(float rx, float ry) {

  // this version of rotate doesn't let the model flip "upside-down"

  // slow the mouse down a little
  rx *= ROTATE_SPEED;
  ry *= ROTATE_SPEED;

  // Note: There is a singularity at the poles (0 & 180 degrees) when
  // 'up' and 'direction' are aligned
  float tiltAngle = acos(up.Dot3(getDirection()) * 180 / M_PI); 
  if (tiltAngle-ry > 178.0) 
    ry = tiltAngle - 178.0; 
  else if (tiltAngle-ry < 2.0) 
    ry = tiltAngle - 2.0; 

  Vec3f h = getHorizontal();
  Matrix m;
  m = Matrix::MakeTranslation(point_of_interest);
  m *= Matrix::MakeAxisRotation(up,convertToRadians(rx));
  m *= Matrix::MakeAxisRotation(h,convertToRadians(ry));
  m *= Matrix::MakeTranslation(-point_of_interest); 
  Vec4f tmp(camera_position.x(),camera_position.y(),camera_position.z(),1);
  tmp = m * tmp;
  camera_position = Vec3f(tmp.x(),tmp.y(),tmp.z());
}

// ====================================================================
// ====================================================================
// GENERATE RAY

Ray OrthographicCamera::generateRay(double x, double y) {
  std::cout << "gr o" << std::endl;
  Vec3f screenCenter = camera_position;
  Vec3f xAxis = getHorizontal() * size; 
  Vec3f yAxis = getScreenUp() * size; 
  Vec3f lowerLeft = screenCenter - 0.5f*xAxis - 0.5f*yAxis;
  Vec3f screenPoint = lowerLeft + float(x)*xAxis + float(y)*yAxis;
  return Ray(screenPoint,getDirection());
}

Ray PerspectiveCamera::generateRay(double x, double y) {
  int width = GLOBAL_args->mesh_data->width;
  int height = GLOBAL_args->mesh_data->height;
  Vec3f screenCenter = camera_position + getDirection();
  float radians_angle = angle * M_PI / 180.0f;
  float screenHeight = 2 * tan(radians_angle/2.0);
  float aspect = std::max(height/float(width),width/float(height));
  screenHeight *= aspect;
  Vec3f xAxis = getHorizontal() * screenHeight;
  Vec3f yAxis = getScreenUp() * screenHeight;
  Vec3f lowerLeft = screenCenter - 0.5f*xAxis - 0.5f*yAxis;
  Vec3f screenPoint = lowerLeft + float(x)*xAxis + float(y)*yAxis;
  Vec3f dir = screenPoint - camera_position;
  dir.Normalize();
  return Ray(camera_position,dir); 
} 

// ====================================================================
// ====================================================================

std::ostream& operator<<(std::ostream &ostr, const Camera &c) {
  const Camera* cp = &c;
  if (dynamic_cast<const OrthographicCamera*>(cp)) {
    const OrthographicCamera* ocp = (const OrthographicCamera*)cp;
    ostr << *ocp << std::endl;
  } else if (dynamic_cast<const PerspectiveCamera*>(cp)) {
    const PerspectiveCamera* pcp = (const PerspectiveCamera*)cp;
    ostr << *pcp << std::endl;
  }
  return ostr;
}

std::ostream& operator<<(std::ostream &ostr, const OrthographicCamera &c) {
  ostr << "OrthographicCamera {" << std::endl;
  ostr << "    camera_position   " << c.camera_position << std::endl;
  ostr << "    point_of_interest " << c.point_of_interest << std::endl;
  ostr << "    up                " << c.up << std::endl; 
  ostr << "    size              " << c.size << std::endl;
  ostr << "}" << std::endl;
  return ostr;
}    

std::ostream& operator<<(std::ostream &ostr, const PerspectiveCamera &c) {
  ostr << "PerspectiveCamera {" << std::endl;
  ostr << "  camera_position    " << c.camera_position << std::endl;
  ostr << "  point_of_interest  " << c.point_of_interest << std::endl;
  ostr << "  up                 " << c.up << std::endl;
  ostr << "  angle              " << c.angle << std::endl;
  ostr << "}" << std::endl;
  return ostr;
}


std::istream& operator>>(std::istream &istr, OrthographicCamera &c) {
  std::string token;
  istr >> token; assert (token == "{");
  istr >> token; assert (token == "camera_position");
  istr >> c.camera_position;
  istr >> token; assert (token == "point_of_interest");
  istr >> c.point_of_interest;
  istr >> token; assert (token == "up");
  istr >> c.up; 
  istr >> token; assert (token == "size");
  istr >> c.size; 
  istr >> token; assert (token == "}");
  return istr;
}    

std::istream& operator>>(std::istream &istr, PerspectiveCamera &c) {
  std::string token;
  istr >> token; assert (token == "{");
  istr >> token; assert (token == "camera_position");
  istr >> c.camera_position;
  istr >> token; assert (token == "point_of_interest");
  istr >> c.point_of_interest;
  istr >> token; assert (token == "up");
  istr >> c.up; 
  istr >> token; assert (token == "angle");
  istr >> c.angle;
  istr >> token; assert (token == "}");
  return istr;
}

// ====================================================================
