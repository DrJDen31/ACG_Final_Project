#include "particleSystem.h"
#include <GL/gl.h>
#ifdef _WIN32
    #include <Windows.h>
#endif

#define M_PI 3.14159265358979323846


// 3D Snowball
ParticleSystem::ParticleSystem(int count, int gridSize) {
    particles.clear();

    glm::vec3 center = glm::vec3(gridSize * 0.5f); // center of the grid
    float radius = gridSize * 0.25f; // tweak as needed

    for (int i = 0; i < count; ++i) {
        Particle p;

        // Small random offset around center (circular scatter)
        float theta = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
        float phi = static_cast<float>(rand()) / RAND_MAX * M_PI;
        float r = static_cast<float>(rand()) / RAND_MAX * radius;
        glm::vec3 offset = glm::vec3(
            r * sin(phi) * cos(theta),
            r * sin(phi) * sin(theta),
            r * cos(phi)
        );

        p.x = center + offset;
        p.v = glm::vec3(4.5f, 20.0f, 4.5f); // upward & right velocity
        p.mass = 10.0f;
        p.volume_0 = 10.0f;
        p.F = glm::mat3(1.0f);
        p.C = glm::mat3(0.0f);

        particles.push_back(p);
    }
}

// 2D Snowball
// ParticleSystem::ParticleSystem(int count, int gridSize) {
//     particles.clear();

//     glm::vec3 center = glm::vec3(gridSize * 0.5f); // center in XY plane
//     float radius = gridSize * 0.25f; // tweak as needed

//     for (int i = 0; i < count; ++i) {
//         Particle p;

//         // Circular offset in 2D (XY)
//         float angle = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
//         float r = static_cast<float>(rand()) / RAND_MAX * radius;
//         glm::vec3 offset = glm::vec3(
//             r * cos(angle),
//             r * sin(angle),
//             0.0f // no Z displacement
//         );

//         p.x = center + offset;
//         p.v = glm::vec3(4.5f, 20.0f, 0.0f); // no Z velocity
//         p.mass = 10.0f;
//         p.volume_0 = 10.0f;
//         p.F = glm::mat3(1.0f);
//         p.C = glm::mat3(0.0f);

//         particles.push_back(p);
//     }
// }


// ParticleSystem::ParticleSystem(int count, int gridSize) {
//     particles.clear();

//     float layerHeight = 1.5f; // thickness above ground
//     for (int i = 0; i < count; ++i) {
//         Particle p;

//         // Random X-Z position across the grid
//         float x = static_cast<float>(rand()) / RAND_MAX * gridSize;
//         float z = static_cast<float>(rand()) / RAND_MAX * gridSize;

//         // Small vertical jitter within a thin layer
//         float y = 40.0f + static_cast<float>(rand()) / RAND_MAX * layerHeight;

//         p.x = glm::vec3(x, y, z);
//         p.v = glm::vec3(0.0f); // initially at rest
//         p.mass = 10.0f;
//         p.volume_0 = 10.0f;
//         p.F = glm::mat3(1.0f);
//         p.C = glm::mat3(0.0f);

//         particles.push_back(p);
//     }
// }

// 4.4: advect particle positions by their velocity
void ParticleSystem::update(Particle& p) {
    p.x += dt * p.v;

    if (p.x.y < 1.0f) {
        p.x.y = 1.0f;
        p.v.y = 0.0f;
    }
}