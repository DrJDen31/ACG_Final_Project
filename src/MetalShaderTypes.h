// ==================================================================
// Shared data types between Metal Shaders and Objective C source code
// ==================================================================

#ifndef MetalShaderTypes_h
#define MetalShaderTypes_h

#include <simd/simd.h>

typedef enum MetalVertexInputIndex
{
    MetalVertexInputIndexVertices     = 0,
    MetalVertexInputIndexViewportSize = 1,
} MetalVertexInputIndex;

// position & color
typedef struct
{
  vector_float4 position;
  vector_float4 normal;
  vector_float4 color;
} MetalVertex;

#endif
