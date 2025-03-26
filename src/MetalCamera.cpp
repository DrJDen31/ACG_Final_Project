// ====================================================================
// Construct the ViewMatrix & ProjectionMatrix for GL Rendering
// ====================================================================

#include <cstring>

#include "argparser.h"
#include "camera.h"
#include "mesh.h"
#include "meshdata.h"

// ====================================================================
// ====================================================================
// Prepare the view matrix

void LookAt(Matrix &view_matrix,
            const Vec3f &camera_position, const Vec3f &point_of_interest, const Vec3f &screen_up) {
  view_matrix.setToIdentity();

  Vec3f Z = camera_position - point_of_interest;
  Z.Normalize();
  Vec3f Y = screen_up;
  Vec3f X;
  Vec3f::Cross3(X,Y,Z);
  Vec3f::Cross3(Y,Z,X);

  X.Normalize();
  Y.Normalize();
  
  view_matrix.set(0,0,  X.x()  );
  view_matrix.set(0,1,  X.y()  );
  view_matrix.set(0,2,  X.z()  );
  view_matrix.set(0,3,  -X.Dot3( camera_position )  );
  view_matrix.set(1,0,  Y.x()  );
  view_matrix.set(1,1,  Y.y()  );
  view_matrix.set(1,2,  Y.z()  );
  view_matrix.set(1,3,  -Y.Dot3( camera_position )  );
  view_matrix.set(2,0,  Z.x()  );
  view_matrix.set(2,1,  Z.y()  );
  view_matrix.set(2,2,  Z.z()  );
  view_matrix.set(2,3,  -Z.Dot3( camera_position )  );
  view_matrix.set(3,0,  0  );
  view_matrix.set(3,1,  0  );
  view_matrix.set(3,2,  0  );
  view_matrix.set(3,3,  1.0f  );
}

// ====================================================================
// ====================================================================
// convert to format for MeshData 

void CopyMat(const Matrix &a,float16 &b) {
  b.data[0]  = a.get(0,0);
  b.data[1]  = a.get(0,1);
  b.data[2]  = a.get(0,2);
  b.data[3]  = a.get(0,3);
  b.data[4]  = a.get(1,0);
  b.data[5]  = a.get(1,1);
  b.data[6]  = a.get(1,2);
  b.data[7]  = a.get(1,3);
  b.data[8]  = a.get(2,0);
  b.data[9]  = a.get(2,1);
  b.data[10] = a.get(2,2);
  b.data[11] = a.get(2,3);
  b.data[12] = a.get(3,0);
  b.data[13] = a.get(3,1);
  b.data[14] = a.get(3,2);
  b.data[15] = a.get(3,3);
}

// ====================================================================
// ====================================================================
// Prepare an orthographic camera matrix

void OrthographicCamera::glPlaceCamera() {

  float aspect = GLOBAL_args->mesh_data->width / (float)GLOBAL_args->mesh_data->height;
  float w;
  float h;
  // handle non square windows
  if (aspect < 1.0) {
    w = size / 2.0;
    h = w / aspect;
  } else {
    h = size / 2.0;
    w = h * aspect;
  }

  LookAt(ViewMatrix,camera_position,point_of_interest,getScreenUp()) ;

  // FIXME: still need to implement & test this (not needed for HW3)
  //ProjectionMatrix = glm::ortho<float>(-w,w,-h,h, 0.1f, 100.0f) ;
  
  //CopyMat(ProjectionMatrix,GLOBAL_args->mesh_data->proj_mat);
  //CopyMat(ViewMatrix,GLOBAL_args->mesh_data->view_mat);
}

// ====================================================================
// ====================================================================
// Prepare a perspective camera matrix

void PerspectiveCamera::glPlaceCamera() {

  // TODO: Fix aspect ratio
  //float aspect = GLOBAL_args->mesh_data->width / (float)GLOBAL_args->mesh_data->height;
  //float w;
  //float h;

  ProjectionMatrix.setToIdentity();
  
  float S = 1 / tan( (angle/2.0) * (M_PI/180.0) );
  float far = 1000.0;
  float near = 0.1;
  
  ProjectionMatrix.set(0,0,S);
  ProjectionMatrix.set(1,1,S);
  ProjectionMatrix.set(2,2, -(far) / (far-near) );
  ProjectionMatrix.set(3,2, -1);
  ProjectionMatrix.set(2,3, -2 * (far*near) / (far-near) );
  ProjectionMatrix.set(3,3, 0); 
  
  LookAt(ViewMatrix,camera_position,point_of_interest,getScreenUp()) ;

  CopyMat(ProjectionMatrix,GLOBAL_args->mesh_data->proj_mat);
  CopyMat(ViewMatrix,GLOBAL_args->mesh_data->view_mat);
}

// ====================================================================
// ====================================================================
