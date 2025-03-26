// ==================================================================
// Implementation of Apple Metal Rendering of the Mesh Data
// ==================================================================

@import simd;
@import MetalKit;

#import "MetalRenderer.h"
#import "MetalMTKView.h"
#import "MetalShaderTypes.h"

#import "meshdata.h"


// The one significant global variable
extern MeshData *mesh_data;


extern void placeCamera();

@implementation MetalRenderer
{
  id<MTLDevice> _device;
  id<MTLRenderPipelineState> _pipelineState;
  id<MTLDepthStencilState> _depthStencilState;
  id<MTLCommandQueue> _commandQueue;
  vector_uint2 _viewportSize;
  id<MTLBuffer> _meshVertexBuffer;
  id<MTLBuffer> _meshPointVertexBuffer;

  //Uniform
  id<MTLBuffer> mvpUniform;
  id<MTLBuffer> mUniform;
  id<MTLBuffer> vUniform;
  id<MTLBuffer> lightposition_Uniform;
  id<MTLBuffer> wireframeUniform;
  id<MTLBuffer> renderModeUniform;
    
  // The number of vertices in our vertex buffer;
  NSUInteger _numMeshVertices;
  NSUInteger _numMeshPointVertices;
}


- (nonnull instancetype)initWithMetalKitView:(nonnull MetalMTKView *)mtkView
{  
    _meshVertexBuffer=0;
    _meshPointVertexBuffer=0;
    mvpUniform=0;
    mUniform=0;
    vUniform=0;
    lightposition_Uniform=0;
    wireframeUniform=0;
    renderModeUniform=0;

    self = [super init];
    if(self)
    {
      _device = mtkView.device;
      [self loadMetal:mtkView];
      [mtkView setRenderer:self];
    }
    
    return self;
}


NSMutableData *meshVertexData = NULL;
NSMutableData *meshPointVertexData = NULL;

+ (nonnull NSData *)generateMeshVertexData
{
  NSUInteger dataSize = sizeof(float)*12*3*mesh_data->meshTriCount;
  [meshVertexData release];
  meshVertexData = [[NSMutableData alloc] initWithLength:dataSize];
  memcpy(meshVertexData.mutableBytes, mesh_data->meshTriData, sizeof(float)*12*3*mesh_data->meshTriCount);
  return meshVertexData;
}

+ (nonnull NSData *)generateMeshPointVertexData
{
  NSUInteger dataSize = sizeof(float)*12*3*mesh_data->meshPointCount;
  [meshPointVertexData release];
  meshPointVertexData = [[NSMutableData alloc] initWithLength:dataSize];
  memcpy(meshPointVertexData.mutableBytes, mesh_data->meshPointData, sizeof(float)*12*mesh_data->meshPointCount);
  return meshPointVertexData;
}


- (void)loadMetal:(nonnull MTKView *)mtkView
{
    mtkView.colorPixelFormat = MTLPixelFormatBGRA8Unorm_sRGB;

    id<MTLLibrary> defaultLibrary = [_device newDefaultLibrary];

    id<MTLFunction> vertexFunction = [defaultLibrary newFunctionWithName:@"vertexShader"];
    id<MTLFunction> fragmentFunction = [defaultLibrary newFunctionWithName:@"fragmentShader"];

    MTLRenderPipelineDescriptor *pipelineStateDescriptor = [[MTLRenderPipelineDescriptor alloc] init];
    pipelineStateDescriptor.label = @"Simple Pipeline";
    pipelineStateDescriptor.vertexFunction = vertexFunction;
    pipelineStateDescriptor.fragmentFunction = fragmentFunction;
    pipelineStateDescriptor.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
    pipelineStateDescriptor.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;
    pipelineStateDescriptor.colorAttachments[0].pixelFormat = mtkView.colorPixelFormat;

    MTLDepthStencilDescriptor *depthStencilDescriptor = [MTLDepthStencilDescriptor new];
    depthStencilDescriptor.depthCompareFunction = MTLCompareFunctionLess;
    depthStencilDescriptor.depthWriteEnabled = YES;
    mtkView.depthStencilPixelFormat = MTLPixelFormatDepth32Float;
    _depthStencilState = [_device newDepthStencilStateWithDescriptor:depthStencilDescriptor];

    NSError *error = NULL;
    _pipelineState = [_device newRenderPipelineStateWithDescriptor:pipelineStateDescriptor error:&error];
    if (!_pipelineState) { NSLog(@"Failed to created pipeline state, error %@", error); }

    [self reGenerate];
  
    _commandQueue = [_device newCommandQueue];
}


- (void)reGenerate
{
  NSData *meshVertexData = [MetalRenderer generateMeshVertexData];
  [_meshVertexBuffer release];
  _meshVertexBuffer = [_device newBufferWithLength:meshVertexData.length
                                            options:MTLResourceStorageModeShared];
  memcpy(_meshVertexBuffer.contents, meshVertexData.bytes, meshVertexData.length);
  _numMeshVertices = meshVertexData.length / sizeof(MetalVertex);

  NSData *meshPointVertexData = [MetalRenderer generateMeshPointVertexData];
  [_meshPointVertexBuffer release];
  _meshPointVertexBuffer = [_device newBufferWithLength:meshPointVertexData.length
                                            options:MTLResourceStorageModeShared];
  memcpy(_meshPointVertexBuffer.contents, meshPointVertexData.bytes, meshPointVertexData.length);
  _numMeshPointVertices = meshPointVertexData.length / sizeof(MetalVertex);
}


- (void)mtkView:(nonnull MTKView *)view drawableSizeWillChange:(CGSize)size
{
  _viewportSize.x = size.width;
  _viewportSize.y = size.height;

  mesh_data->width = size.width / 2.0;
  mesh_data->height = size.height / 2.0;
}



- (matrix_float4x4) copyMatrix:(float16) mat {
  matrix_float4x4 m=matrix_identity_float4x4;

  m.columns[0][0] = mat.data[0];
  m.columns[1][0] = mat.data[1];
  m.columns[2][0] = mat.data[2];
  m.columns[3][0] = mat.data[3];

  m.columns[0][1] = mat.data[4];
  m.columns[1][1] = mat.data[5];
  m.columns[2][1] = mat.data[6];
  m.columns[3][1] = mat.data[7];

  m.columns[0][2] = mat.data[8];
  m.columns[1][2] = mat.data[9];
  m.columns[2][2] = mat.data[10];
  m.columns[3][2] = mat.data[11];

  m.columns[0][3] = mat.data[12];
  m.columns[1][3] = mat.data[13];
  m.columns[2][3] = mat.data[14];
  m.columns[3][3] = mat.data[15];
  
  return m;
}



/// Called whenever the view needs to render a frame
- (void)drawInMTKView:(nonnull MTKView *)view
{
  // Create a new command buffer for each render pass to the current drawable
  id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
  commandBuffer.label = @"MyCommand";

  // Obtain a renderPassDescriptor generated from the view's drawable textures
  MTLRenderPassDescriptor *renderPassDescriptor = view.currentRenderPassDescriptor;


  if(renderPassDescriptor != nil)  {

      // set background color to white
      renderPassDescriptor.colorAttachments[0].clearColor = MTLClearColorMake(1.0,1.0,1.0,1.0);
      
      // Create a render command encoder so we can render into something
      id<MTLRenderCommandEncoder> renderEncoder =
        [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];
      renderEncoder.label = @"MyRenderEncoder";

      [renderEncoder setViewport:(MTLViewport){0.0, 0.0, _viewportSize.x, _viewportSize.y, -1.0, 1.0 }];
      [renderEncoder setRenderPipelineState:_pipelineState];

      [renderEncoder setDepthStencilState:_depthStencilState];
      [renderEncoder setFrontFacingWinding:MTLWindingCounterClockwise];

      // cull neither (we want to see the blue "inside" / backside)
      if (!mesh_data->intersect_backfacing) {
        [renderEncoder setCullMode:MTLCullModeBack];
      }
      //[renderEncoder setCullMode:MTLCullModeFront];

      
      placeCamera();

      matrix_float4x4 modelMatrix=matrix_identity_float4x4;
      matrix_float4x4 worldMatrix=matrix_identity_float4x4;
      matrix_float4x4 viewMatrix=[self copyMatrix :mesh_data->view_mat];
      matrix_float4x4 projectiveMatrix=[self copyMatrix :mesh_data->proj_mat];

      matrix_float4x4 modelViewProjectionTransformation
        =matrix_multiply(projectiveMatrix,
                         matrix_multiply(viewMatrix,
                                         matrix_multiply(worldMatrix, modelMatrix)));
          
      vector_float3 lightPosition= {4,4,4};
      
      //Load the MVP transformation into the MTLBuffer
      if (mvpUniform==0) {
        mvpUniform=[_device newBufferWithBytes:(void*)&modelViewProjectionTransformation
                                        length:sizeof(modelViewProjectionTransformation)
                                       options:MTLResourceOptionCPUCacheModeDefault];
        mUniform=  [_device newBufferWithBytes:(void*)&modelMatrix
                                        length:sizeof(modelMatrix)
                                       options:MTLResourceOptionCPUCacheModeDefault];
        vUniform=  [_device newBufferWithBytes:(void*)&viewMatrix
                                        length:sizeof(viewMatrix)
                                       options:MTLResourceOptionCPUCacheModeDefault];
        lightposition_Uniform=  [_device newBufferWithBytes:(void*)&lightPosition
                                                     length:sizeof(lightPosition)
                                                    options:MTLResourceOptionCPUCacheModeDefault];
        wireframeUniform=  [_device newBufferWithBytes:(void*)&mesh_data->wireframe
                                                length:sizeof(int)
                                               options:MTLResourceOptionCPUCacheModeDefault];
        renderModeUniform=  [_device newBufferWithBytes:(void*)&mesh_data->render_mode
                                                 length:sizeof(int)
                                                options:MTLResourceOptionCPUCacheModeDefault];
      } else {
        memcpy(mvpUniform.contents,&modelViewProjectionTransformation,sizeof(modelViewProjectionTransformation));
        memcpy(mUniform.contents,&modelViewProjectionTransformation,sizeof(modelMatrix));
        memcpy(vUniform.contents,&modelViewProjectionTransformation,sizeof(viewMatrix));
        memcpy(lightposition_Uniform.contents,&lightPosition,sizeof(lightPosition));
        memcpy(wireframeUniform.contents,&mesh_data->wireframe,sizeof(int));
        memcpy(renderModeUniform.contents,&mesh_data->render_mode,sizeof(int));
      }
      
      [renderEncoder setVertexBytes:&_viewportSize
                             length:sizeof(_viewportSize)
                            atIndex:MetalVertexInputIndexViewportSize];
      [renderEncoder setVertexBuffer:mvpUniform offset:0 atIndex:2];
      [renderEncoder setVertexBuffer:mUniform offset:0 atIndex:3];
      [renderEncoder setVertexBuffer:vUniform offset:0 atIndex:4];
      [renderEncoder setVertexBuffer:lightposition_Uniform offset:0 atIndex:5];
      [renderEncoder setVertexBuffer:wireframeUniform offset:0 atIndex:6];
      [renderEncoder setVertexBuffer:renderModeUniform offset:0 atIndex:7];


      // Send our data to the Metal vertex shader
      [renderEncoder setVertexBuffer:_meshVertexBuffer
                              offset:0
                             atIndex:MetalVertexInputIndexVertices];
      int numMeshVertices = mesh_data->meshTriCount * 3;
      [renderEncoder drawPrimitives:MTLPrimitiveTypeTriangle
                        vertexStart:0
                        vertexCount:numMeshVertices];

      [renderEncoder setVertexBuffer:_meshPointVertexBuffer
                              offset:0
                             atIndex:MetalVertexInputIndexVertices];
      int numMeshPointVertices = mesh_data->meshPointCount;
      [renderEncoder drawPrimitives:MTLPrimitiveTypePoint
                        vertexStart:0
                        vertexCount:numMeshPointVertices];


      
      [renderEncoder endEncoding];
      [commandBuffer presentDrawable:view.currentDrawable];
    }

  // Finalize rendering here & push the command buffer to the GPU
  [commandBuffer commit];
}

@end
