#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "grid.h"

// FIXME: use param
static constexpr float dt = 1.0f;

struct Particle {
  glm::vec2 x;
  glm::vec2 v;
  glm::mat2 F = glm::mat2(1.0f); // Deformation gradient
  glm::mat2 C = glm::mat2(0.0f); // APIC affine matrix
  float Jp = 1.0f;               // Plastic compression
  float mass = 1.0f;
};

class ParticleSystem {
public:
  std::vector<Particle> particles;

  void init(int count, int gridSize);
  const std::vector<Particle>& getParticles() const { return particles; }
  void draw() const;
  void p2g(Grid &grid);   // Particle-to-grid
  void g2p(Grid &grid);   // Grid-to-particle
  void update();          // Advance particle positions
};
