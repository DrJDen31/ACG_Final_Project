#pragma once
#include "grid.h"
#include "particleSystem.h"
#include "mesh.h"

struct Weights {
    glm::ivec3 base;
    glm::vec3 wx, wy, wz;
    glm::vec3 fx;
};

class Ray;
class Hit;

class MPM {
public:
    MPM(int gridSize, const std::vector<MPMObject>& objects);
    std::vector<Particle>& getParticles() { return particles.getParticles(); }
    void step();
    void draw() const;

    // for raytracing
    bool intersect(const Ray& r, Hit& h, Vec3f& particle_pos, double threshold = 0.15) const;

private:
    void p2g(); // Particle-to-grid
    void g2p(); // Grid-to-particle

    Grid grid;
    ParticleSystem particles;

    // Paramters for simulation
    float mu; // shear modulus
    float lambda; // bulk modulus
};
