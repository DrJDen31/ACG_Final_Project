// ================================================================
// Parse the command line arguments and the input file
// ================================================================

#ifndef __ARG_PARSER_H__
#define __ARG_PARSER_H__

#include <string>
#include <random>

struct MeshData;
class Mesh;
class RayTracer;
class Radiosity;
class PhotonMapping;
class BoundingBox;

// ======================================================================
// Class to collect all the high-level rendering parameters controlled
// by the command line or the keyboard input
// ======================================================================

class ArgParser {

public:

  ArgParser(int argc, const char *argv[], MeshData *_mesh_data);

  
  double rand() {
#if 1
    // random seed
    static std::random_device rd;    
    static std::mt19937 engine(rd());
#else
    // deterministic randomness
    static std::mt19937 engine(37);
#endif
    static std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(engine);
  }

  // helper functions
  void separatePathAndFile(const std::string &input, std::string &path, std::string &file);

  void Load();
  void DefaultValues();

  // ==============
  // REPRESENTATION
  // all public! (no accessors)

  std::string input_file;
  std::string path;

  Mesh *mesh;
  MeshData *mesh_data;
  RayTracer *raytracer;
  Radiosity *radiosity;
  PhotonMapping *photon_mapping;
  BoundingBox *bbox;


};

extern ArgParser *GLOBAL_args;
void packMesh(MeshData *mesh_data, RayTracer *raytracer, Radiosity *radiosity, PhotonMapping *photonmapping);

#endif
