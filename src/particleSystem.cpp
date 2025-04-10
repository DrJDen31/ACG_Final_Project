#include "particleSystem.h"

#ifdef _WIN32
    #include <windows.h>
#endif

#include <GL/gl.h>


#define M_PI 3.14159265358979323846

ParticleSystem::ParticleSystem(int count, glm::vec3 velocity, glm::vec3 center,
                               glm::vec3 size, float mass, float volume, float radius,
                               const std::string& region) {
    particles.clear();

    for (int i = 0; i < count; ++i) {
        Particle p;

        // Pick shape method based on region
        glm::vec3 offset;
        if (region == "spherical") {
            float theta = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
            float phi   = static_cast<float>(rand()) / RAND_MAX * M_PI;
            float r     = static_cast<float>(rand()) / RAND_MAX * radius;

            offset = glm::vec3(
                r * sin(phi) * cos(theta),
                r * sin(phi) * sin(theta),
                r * cos(phi)
            );
        } else { // cartesian (cube)
            offset = glm::vec3(
                (static_cast<float>(rand()) / RAND_MAX - 0.5f) * size.x,
                (static_cast<float>(rand()) / RAND_MAX - 0.5f) * size.y,
                (static_cast<float>(rand()) / RAND_MAX - 0.5f) * size.z
            );
        }

        p.x = center + offset;
        p.v = velocity;
        p.mass = mass;
        p.volume_0 = volume;
        p.F = glm::mat3(1.0f);
        p.C = glm::mat3(0.0f);

        // random normal
        p.normal = glm::vec3(static_cast<float>(rand()) / RAND_MAX, static_cast<float>(rand()) / RAND_MAX, static_cast<float>(rand()) / RAND_MAX);

        particles.push_back(p);
    }
}

// 4.4: advect particle positions by their velocity
void ParticleSystem::update(Particle& p) {
    p.x += dt * p.v;

    if (p.x.y < 1.0f) {
        p.x.y = 1.0f;
        p.v.y = 0.0f;
    }
}
