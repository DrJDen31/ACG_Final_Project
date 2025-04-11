#include "mpm.h"
#include "particleSystem.h"

#ifdef _WIN32
    #include <windows.h>
#endif

#include <GL/gl.h>


#define M_PI 3.14159265358979323846

ParticleSystem::ParticleSystem(const std::vector<MPMObject>& objects) {
    particles.clear();

    for (const auto& obj : objects) {
        for (int i = 0; i < obj.particle_count; ++i) {
            Particle p;
            glm::vec3 offset;

            if (obj.region_type == "spherical") {
                float theta = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
                float phi   = static_cast<float>(rand()) / RAND_MAX * M_PI;
                float r     = static_cast<float>(rand()) / RAND_MAX * obj.radius;

                offset = glm::vec3(
                    r * sin(phi) * cos(theta),
                    r * sin(phi) * sin(theta),
                    r * cos(phi)
                );
            } else {
                offset = glm::vec3(
                    (static_cast<float>(rand()) / RAND_MAX - 0.5f) * obj.size.x,
                    (static_cast<float>(rand()) / RAND_MAX - 0.5f) * obj.size.y,
                    (static_cast<float>(rand()) / RAND_MAX - 0.5f) * obj.size.z
                );
            }

            p.x = obj.center + offset;
            p.v = obj.velocity;
            p.mass = obj.mass;
            p.volume_0 = obj.volume;
            p.F = glm::mat3(1.0f);
            p.C = glm::mat3(0.0f);
            p.normal = glm::normalize(glm::vec3(
                static_cast<float>(rand()) / RAND_MAX,
                static_cast<float>(rand()) / RAND_MAX,
                static_cast<float>(rand()) / RAND_MAX
            ));

            particles.push_back(p);
        }
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
