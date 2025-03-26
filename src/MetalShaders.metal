// ==================================================================
// Apple Metal Shaders
// ==================================================================

#include <simd/simd.h>
#include <metal_common>

using namespace metal;

// Include header shared between this Metal shader code and C code executing Metal API commands
#include "MetalShaderTypes.h"


// Vertex shader outputs and fragment shader inputs
typedef struct
{
  float4 clipSpacePosition [[position]];
  float pointSize [[point_size]];
  float3 position_worldspace;
  float3 normal_cameraspace;
  float3 eyedirection_cameraspace;
  float3 lightdirection_cameraspace;
  float4 mycolor;
  int renderMode;
} RasterizerData;


// ======================================================================
// Vertex function
vertex RasterizerData
vertexShader(uint vertexID [[vertex_id]],
             constant MetalVertex *vertices [[buffer(MetalVertexInputIndexVertices)]],
             constant vector_uint2 *viewportSizePointer [[buffer(MetalVertexInputIndexViewportSize)]],
             constant float4x4 &mvp [[buffer(2)]],
             constant float4x4 &m [[buffer(3)]],
             constant float4x4 &v [[buffer(4)]],
             constant float3 &lightposition [[buffer(5)]],    
             constant int &wireframe [[buffer(6)]],
             constant int &renderMode [[buffer(7)]]         
             )

{
  RasterizerData out;
  out.pointSize = 10;

  out.clipSpacePosition = mvp * vertices[vertexID].position;
  out.position_worldspace = (m * vertices[vertexID].position).xyz;
  
  vector_float3 vertexposition_cameraspace = (v * m * vertices[vertexID].position).xyz;
  out.eyedirection_cameraspace = vector_float3(0,0,0) - vertexposition_cameraspace;

  out.normal_cameraspace = (v * m * vertices[vertexID].normal).xyz;

  out.mycolor = vertices[vertexID].color;
  out.renderMode = renderMode;

  return out;
}


// ======================================================================
// Fragment function
fragment float4 fragmentShader(RasterizerData in [[stage_in]],
                               float2 pointCoord [[point_coord]],
                               bool is_front_face [[front_facing]],
                               constant float3 &lightposition2 [[buffer(5)]],
                               constant int &wireframe2 [[buffer(6)]],
                               constant int &renderMode2 [[buffer(7)]]) {

  // NOTE: This is a work in progress, somethig is fishy about the uniforms -- needs debugging

  // Light emission properties
  // You probably want to put them as uniforms
  vector_float3 LightColor = vector_float3(1,1,1);
  float LightPower = 50.0;

  int rm = in.renderMode;

  // Material properties
  vector_float3 MaterialDiffuseColor;
  MaterialDiffuseColor = in.mycolor.xyz;
    
  vector_float3 MaterialAmbientColor = vector_float3(0.2,0.2,0.2) * MaterialDiffuseColor;
  vector_float3 MaterialSpecularColor = vector_float3(0.1,0.1,0.1);

  vector_float3 lightposition(4,4,4);

  if (length(in.normal_cameraspace) < 0.1 ||
      rm == 1  /* radiance */ ||
      rm == 2  /* form factors */ ||
      rm == 4  /* undistributed */ ||
      rm == 5  /* absorbed */) {
    // don't compute local shading on this patch 
    return vector_float4(MaterialDiffuseColor,1);
  } 

  // Normal of the computed fragment, in camera space
  vector_float3 n = normalize( in.normal_cameraspace );

  if(! is_front_face ) {
    MaterialDiffuseColor = vector_float3(0.0,0.0,0.6);
    MaterialAmbientColor = vector_float3(0.3,0.3,0.3) * MaterialDiffuseColor;
    MaterialSpecularColor = vector_float3(0.1,0.1,0.3);
    n = -n;
  }

  // Distance to the light
  // NOTE: NOT RIGHT FOR A HEADLAMP...
  float distance = length( lightposition - in.position_worldspace );

  // Direction of the light (from the fragment to the light)
  // NOTE: INSTEAD MAKING A HEADLAMP
  vector_float3 l = normalize(vector_float3(1.0,0.8,-2.0));

  // Cosine of the angle between the normal and the light direction,
  // clamped above 0
  //  - light is at the vertical of the triangle -> 1
  //  - light is perpendicular to the triangle -> 0
  //  - light is behind the triangle -> 0
  float cosTheta = clamp( dot( n,l ), 0,1 );

  // Eye vector (towards the camera)
  vector_float3 E = normalize(in.eyedirection_cameraspace);
  // Direction in which the triangle reflects the light
  vector_float3 R = reflect(-l,n);

  // Cosine of the angle between the Eye vector and the Reflect vector,
  // clamped to 0
  //  - Looking into the reflection -> 1
  //  - Looking elsewhere -> < 1
  float cosAlpha = clamp( dot( E,R ), 0,1 );

  vector_float3 mycolor2 =
    // Ambient : simulates indirect lighting
    MaterialAmbientColor +
    // Diffuse : "color" of the object
    MaterialDiffuseColor * LightColor * LightPower * cosTheta / (distance*distance) +
    // Specular : reflective highlight, like a mirror
    MaterialSpecularColor * LightColor * LightPower * pow(cosAlpha,5) / (distance*distance);

  return vector_float4(mycolor2,1);
}

// ======================================================================
