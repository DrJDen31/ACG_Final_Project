#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <cassert>
#include <iostream>
#include <fstream>

#include "vectors.h"
#include "matrix.h"

class Ray;

// ====================================================================
// ====================================================================

class Camera {
public:
  // CONSTRUCTOR & DESTRUCTOR
  Camera(const Vec3f &c, const Vec3f &poi, const Vec3f &u);
  virtual ~Camera() {}

  // RENDERING
  virtual Ray generateRay(double x, double y) = 0;

  // GL NAVIGATION
  virtual void glPlaceCamera() = 0;
  void dollyCamera(float dist);
  virtual void zoomCamera(float dist) = 0;
  void truckCamera(float dx, float dy);
  void rotateCamera(float rx, float ry);

  const Matrix& getViewMatrix() const { return ViewMatrix; }
  const Matrix& getProjectionMatrix() const { return ProjectionMatrix; }

  friend std::ostream& operator<< (std::ostream& ostr, const Camera &c);
  friend std::istream& operator>> (std::istream& istr, Camera &c);

public:
  //protected:
  Camera() { assert(0); } // don't use

  // HELPER FUNCTIONS
  Vec3f getHorizontal() const {
    Vec3f answer;
    Vec3f::Cross3(answer,getDirection(),up);
    answer.Normalize();
    return answer;
  }
  Vec3f getScreenUp() const {
    Vec3f answer;
    Vec3f::Cross3(answer,getHorizontal(),getDirection());
    answer.Normalize();
    return answer;
  }
  Vec3f getDirection() const {
    Vec3f answer = point_of_interest - camera_position;
    answer.Normalize();
    return answer;
  }

  // REPRESENTATION
  Vec3f point_of_interest;
  Vec3f camera_position;
  Vec3f up;
  Matrix ViewMatrix;
  Matrix ProjectionMatrix;
};

// ====================================================================

class OrthographicCamera : public Camera {
public:
  // CONSTRUCTOR & DESTRUCTOR
  OrthographicCamera(const Vec3f &c = Vec3f(0,0,1), 
		     const Vec3f &poi = Vec3f(0,0,0), 
		     const Vec3f &u = Vec3f(0,1,0),
		     float s=100);  

  // RENDERING
  Ray generateRay(double x, double y);

  // GL NAVIGATION
  void glPlaceCamera();
  void zoomCamera(float factor);

  friend std::ostream& operator<< (std::ostream& ostr, const OrthographicCamera &c);
  friend std::istream& operator>> (std::istream& istr, OrthographicCamera &c);

private:
  // REPRESENTATION
  float size;
};

// ====================================================================

class PerspectiveCamera : public Camera {
public:
  // CONSTRUCTOR & DESTRUCTOR
  PerspectiveCamera(const Vec3f &c = Vec3f(0,0,1), 
		    const Vec3f &poi = Vec3f(0,0,0), 
		    const Vec3f &u = Vec3f(0,1,0),
		    float a = 45);

  // RENDERING
  Ray generateRay(double x, double y);

  // GL NAVIGATION
  void glPlaceCamera();
  void zoomCamera(float dist);

  friend std::ostream& operator<< (std::ostream& ostr, const PerspectiveCamera &c);
  friend std::istream& operator>> (std::istream& istr, PerspectiveCamera &c);

private:
  // REPRESENTATION
  float angle;
};

// ====================================================================

#endif

