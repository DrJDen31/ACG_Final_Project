#include <iostream>
#include <cstring>
#include "utils.h"

// ============================================================================================
// ============================================================================================
// Helper function that adds 3 triangles to render wireframe

void AddWireFrameTriangle(float* &current,
                          const Vec3f &apos, const Vec3f &bpos, const Vec3f &cpos,
                          const Vec3f &anormal, const Vec3f &bnormal, const Vec3f &cnormal,
                          const Vec3f &edge_color,
                          const Vec3f &acolor_, const Vec3f &bcolor_, const Vec3f &ccolor) {

  // will be edited if wireframe is visible
  Vec3f acolor=acolor_;
  Vec3f bcolor=bcolor_;
  
  /*     a-------------b
   *      \           /
   *       x---------y
   *        \       /
   *         \     /
   *          \   /
   *           \ /
   *            c     
   */

  // interpolate the positions, colors, and normals
  float frac = 0.05;
  Vec3f xpos = (1-frac)*apos + frac*cpos;
  Vec3f ypos = (1-frac)*bpos + frac*cpos;
  Vec3f xcolor = (1-frac)*acolor + frac*ccolor;
  Vec3f ycolor = (1-frac)*bcolor + frac*ccolor;
  Vec3f xnormal = (1-frac)*anormal + frac*cnormal; xnormal.Normalize();
  Vec3f ynormal = (1-frac)*bnormal + frac*cnormal; ynormal.Normalize();

  float12 ta;
  float12 tb;
  float12 tc;

  // the main triangle
  ta = { float(xpos.x()),float(xpos.y()),float(xpos.z()),1, float(xnormal.x()),float(xnormal.y()),float(xnormal.z()),0, float(xcolor.r()),float(xcolor.g()),float(xcolor.b()),1 };
  tb = { float(ypos.x()),float(ypos.y()),float(ypos.z()),1, float(ynormal.x()),float(ynormal.y()),float(ynormal.z()),0, float(ycolor.r()),float(ycolor.g()),float(ycolor.b()),1 };
  tc = { float(cpos.x()),float(cpos.y()),float(cpos.z()),1, float(cnormal.x()),float(cnormal.y()),float(cnormal.z()),0, float(ccolor.r()),float(ccolor.g()),float(ccolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;

  // the two triangles that make the wireframe
  if (!GLOBAL_args->mesh_data->wireframe) {
    acolor = edge_color;
    bcolor = edge_color;
    xcolor = edge_color;
    ycolor = edge_color;    
  }
  ta = { float(apos.x()),float(apos.y()),float(apos.z()),1, float(anormal.x()),float(anormal.y()),float(anormal.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  tb = { float(bpos.x()),float(bpos.y()),float(bpos.z()),1, float(bnormal.x()),float(bnormal.y()),float(bnormal.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tc = { float(xpos.x()),float(xpos.y()),float(xpos.z()),1, float(xnormal.x()),float(xnormal.y()),float(xnormal.z()),0, float(xcolor.r()),float(xcolor.g()),float(xcolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  ta = { float(xpos.x()),float(xpos.y()),float(xpos.z()),1, float(xnormal.x()),float(xnormal.y()),float(xnormal.z()),0, float(xcolor.r()),float(xcolor.g()),float(xcolor.b()),1 };
  tb = { float(bpos.x()),float(bpos.y()),float(bpos.z()),1, float(bnormal.x()),float(bnormal.y()),float(bnormal.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tc = { float(ypos.x()),float(ypos.y()),float(ypos.z()),1, float(ynormal.x()),float(ynormal.y()),float(ynormal.z()),0, float(ycolor.r()),float(ycolor.g()),float(ycolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
}

// ============================================================================================
// ============================================================================================
// Adds 2 triangles to make a simple quad (no wireframe)

void AddQuad(float* &current,
             const Vec3f &apos, const Vec3f &bpos, const Vec3f &cpos, const Vec3f &dpos,
             const Vec3f &normal,
             const Vec3f &color) {

  float12 ta;
  float12 tb;
  float12 tc;

  ta = { float(apos.x()),float(apos.y()),float(apos.z()),1, float(normal.x()),float(normal.y()),float(normal.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(bpos.x()),float(bpos.y()),float(bpos.z()),1, float(normal.x()),float(normal.y()),float(normal.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(cpos.x()),float(cpos.y()),float(cpos.z()),1, float(normal.x()),float(normal.y()),float(normal.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;

  ta = { float(apos.x()),float(apos.y()),float(apos.z()),1, float(normal.x()),float(normal.y()),float(normal.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(cpos.x()),float(cpos.y()),float(cpos.z()),1, float(normal.x()),float(normal.y()),float(normal.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(dpos.x()),float(dpos.y()),float(dpos.z()),1, float(normal.x()),float(normal.y()),float(normal.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
}

// ============================================================================================
// ============================================================================================
// Adds 12 triangles to make a simple rectangular box

void AddBox(float* &current,
            const Vec3f pos[8],
            const Vec3f &color) {

  Vec3f normal1 = ComputeNormal(pos[0],pos[1],pos[2]);
  normal1 = Vec3f(0,0,0);
  AddQuad(current,pos[0],pos[1],pos[3],pos[2],normal1,color);
  AddQuad(current,pos[4],pos[6],pos[7],pos[5],-normal1,color);

  Vec3f normal2 = ComputeNormal(pos[0],pos[4],pos[5]);
  normal2 = Vec3f(0,0,0);
  AddQuad(current,pos[0],pos[4],pos[5],pos[1],normal2,color);
  AddQuad(current,pos[2],pos[3],pos[7],pos[6],-normal2,color);

  Vec3f normal3 = ComputeNormal(pos[0],pos[2],pos[6]);
  normal3 = Vec3f(0,0,0);
  AddQuad(current,pos[0],pos[2],pos[6],pos[4],normal3,color);
  AddQuad(current,pos[1],pos[5],pos[7],pos[3],normal3,color);
}

// ============================================================================================
// ============================================================================================
