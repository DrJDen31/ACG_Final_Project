#pragma once
#include "grid.h"
#include "particleSystem.h"

class MPM {
public:
    MPM(int gridSize, int particleCount);
    const std::vector<Particle>& getParticles() const { return particles.getParticles(); }
    void step();
    void draw() const;

private:
    Grid grid;
    ParticleSystem particles;
};
