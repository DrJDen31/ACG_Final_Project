#ifndef _MESH_DATA_H_
#define _MESH_DATA_H_

// ====================================================================
// ====================================================================

// a homogeneous 3D point or a color with alpha
typedef struct float3 {
  float data[3];
} float3;

// a homogenous 3D point or a color with alpha
typedef struct float4 {
  float data[4];
} float4;

// a vertex with position, normal, and color
typedef struct float12 {
  float data[12];
} float12;

// a 4x4 matrix
typedef struct float16 {
  float data[16];
} float16;


// VISUALIZATION MODES FOR RADIOSITY
#define NUM_RENDER_MODES 6
enum RENDER_MODE { RENDER_MATERIALS, RENDER_RADIANCE, RENDER_FORM_FACTORS, 
		   RENDER_LIGHTS, RENDER_UNDISTRIBUTED, RENDER_ABSORBED };


typedef struct MeshData {
  
  // REPRESENTATION
  int width;
  int height;

  // animation control
  bool raytracing_animation;
  bool radiosity_animation;

  // RADIOSITY PARAMETERS
  enum RENDER_MODE render_mode;
  bool interpolate;
  bool wireframe;
  int num_form_factor_samples;
  int sphere_horiz;
  int sphere_vert;
  int cylinder_ring_rasterization;

  // RAYTRACING PARAMETERS
  int num_bounces;
  int num_shadow_samples;
  int num_antialias_samples;
  int num_glossy_samples;
  float3 ambient_light;
  bool intersect_backfacing;
  int raytracing_divs_x;
  int raytracing_divs_y;
  int raytracing_x;
  int raytracing_y;
  
  // PHOTON MAPPING PARAMETERS
  int num_photons_to_shoot;
  int num_photons_to_collect;
  bool render_photons;
  bool render_photon_directions;
  bool render_kdtree;
  bool gather_indirect;

  bool bounding_box_frame;
  
  int meshTriCount;
  float* meshTriData;
  int meshPointCount;
  float* meshPointData;

  int meshTriCount_allocated;
  int meshPointCount_allocated;
  
  float16 proj_mat;
  float16 view_mat;
  
} MeshData;


void INIT_MeshData(MeshData *mesh_data);
void loadOBJ(MeshData *mesh_data);

// ====================================================================
// ====================================================================

#endif
