#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "grid.h"

struct Particle {
  glm::vec3 x; // position
  glm::vec3 v; // velocity
  glm::mat3 F; // Deformation gradient
  glm::mat3 C; // Affine velocity matrix
  float volume_0;
  float mass;
  glm::vec3 normal; // normal (for raytracing)
};

class ParticleSystem {
public:
  ParticleSystem(int count, int gridSize);
  const std::vector<Particle>& getParticles() const { return particles; }
  std::vector<Particle>& getParticles() { return particles; }
  void update(Particle& p);

private:
  std::vector<Particle> particles;
};
