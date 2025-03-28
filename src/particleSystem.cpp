#include "particleSystem.h"

// Square
// ParticleSystem::ParticleSystem(int count, int gridSize) {
//     particles.clear();
//     for (int i = 0; i < count; ++i) {
//         Particle p;
//         p.x = glm::vec2(rand() % gridSize, rand() % gridSize);
//         p.v = glm::vec2(0.0f);
//         particles.push_back(p);
//     }
// }

// Snowball
ParticleSystem::ParticleSystem(int count, int gridSize) {
    particles.clear();

    glm::vec2 center = glm::vec2(gridSize * 0.5f); // center of the grid
    float radius = gridSize * 0.25f; // tweak as needed

    for (int i = 0; i < count; ++i) {
        Particle p;

        // Small random offset around center (circular scatter)
        float angle = static_cast<float>(rand()) / RAND_MAX * 2.0f * 3.1415926f;
        float r = static_cast<float>(rand()) / RAND_MAX * radius;
        glm::vec2 offset = glm::vec2(cos(angle), sin(angle)) * r;

        p.x = center + offset;
        p.v = glm::vec2(4.5f, 20.0f); // upward & right velocity
        p.mass = 1.0f;

        particles.push_back(p);
    }
}




// 4.4: advect particle positions by their velocity
void ParticleSystem::update(Particle& p) {
    float dt = 0.05f * (1.0f / 60.0f); // global timestepTODO: put somewhere else
    p.x += dt * p.v;

    if (p.x.y < 1.0f) {
        p.x.y = 1.0f;
        p.v.y = 0.0f;
    }
}