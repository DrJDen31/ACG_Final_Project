#pragma once
#include <vector>
#include <glm/glm.hpp>

struct Particle {
  glm::vec2 x;
  glm::vec2 v;
  float mass;
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
