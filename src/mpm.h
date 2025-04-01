#pragma once
#include "grid.h"
#include "particleSystem.h"

struct Weights {
    glm::ivec3 base;
    glm::vec3 wx, wy, wz;
    glm::vec3 fx;
};


class MPM {
public:
    MPM(int gridSize, int particleCount);
    std::vector<Particle>& getParticles() { return particles.getParticles(); }
    void step();
    void draw() const;

private:
    void p2g(); // Particle-to-grid
    void g2p(); // Grid-to-particle

    Grid grid;
    ParticleSystem particles;
};
