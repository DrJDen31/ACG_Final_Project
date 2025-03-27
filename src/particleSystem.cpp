#include "particleSystem.h"
#include <GL/gl.h>

// Square
// void ParticleSystem::init(int count, int gridSize) {
//     particles.clear();
//     for (int i = 0; i < count; ++i) {
//         Particle p;
//         p.x = glm::vec2(rand() % gridSize, rand() % gridSize);
//         p.v = glm::vec2(0.0f);
//         particles.push_back(p);
//     }
// }

// Snowball
void ParticleSystem::init(int count, int gridSize) {
    particles.clear();

    glm::vec2 center(gridSize / 2.0f, gridSize / 2.0f);
    float radius = gridSize * 0.3f; // 30% of grid size

    for (int i = 0; i < count; ++i) {
        float angle = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
        float r = sqrtf(static_cast<float>(rand()) / RAND_MAX) * radius; // uniform in circle

        glm::vec2 offset = glm::vec2(cos(angle), sin(angle)) * r;

        Particle p;
        p.x = center + offset;
        p.v = glm::vec2(0.0f);
        particles.push_back(p);
    }
}


void ParticleSystem::draw() const {
    glPointSize(4.0f);
    glBegin(GL_POINTS);
    for (const auto& p : particles) {
        glVertex2f(p.x.x / 64.0f * 2.0f - 1.0f, p.x.y / 64.0f * 2.0f - 1.0f);
    }
    glEnd();
}

float quadraticBSpline(float x) {
    x = fabs(x);
    if (x < 1.0f) return 0.5f * x * x - x + 0.75f;
    else if (x < 2.0f) return 0.25f * (2.0f - x) * (2.0f - x);
    return 0.0f;
}

void polarDecomposition(const glm::mat2& F, glm::mat2& R, glm::mat2& S) {
    // Approximate polar decomposition using Gram-Schmidt orthogonalization
    glm::vec2 col0 = F[0];
    glm::vec2 col1 = F[1] - glm::dot(col0, F[1]) / glm::dot(col0, col0) * col0;

    col0 = glm::normalize(col0);
    col1 = glm::normalize(col1);

    R = glm::mat2(col0, col1);       // Approximate rotation
    S = glm::transpose(R) * F;       // Stretch
}

void ParticleSystem::p2g(Grid &grid) {
    constexpr float E = 200.0f;
    constexpr float nu = 0.2f;
    constexpr float dt = 0.002f;

    constexpr float mu = E / (2.0f * (1.0f + nu));
    constexpr float lambda = E * nu / ((1.0f + nu) * (1.0f - 2.0f * nu));

    grid.clear();

    for (auto& p : particles) {
        glm::ivec2 base = glm::floor(p.x - 0.5f);

        float J = glm::determinant(p.F);
        glm::mat2 R, S;
        polarDecomposition(p.F, R, S);

        glm::mat2 stress = 2.0f * mu * (p.F - R) * glm::transpose(p.F)
                         + lambda * logf(J) * J * glm::mat2(1.0f);
        glm::mat2 affine = -dt * 0.1f * stress + p.mass * p.C;

        for (int gx = 0; gx < 3; gx++) {
            for (int gy = 0; gy < 3; gy++) {
                int ix = base.x + gx;
                int iy = base.y + gy;
                if (ix < 0 || ix >= grid.size || iy < 0 || iy >= grid.size) continue;

                glm::vec2 cellPos(ix, iy);
                glm::vec2 dist = p.x - cellPos;

                float weight = 1.0f; // still fake

                Cell& cell = grid.at(ix, iy);
                cell.v += weight * (p.v * p.mass + affine * dist);
                cell.mass += weight * p.mass;
            }
        }
    }
}

void ParticleSystem::g2p(Grid &grid) {
    for (auto& p : particles) {
        glm::ivec2 base = glm::floor(p.x);
        glm::vec2 newV(0.0f);
        float totalWeight = 0.0f;

        for (int gx = -1; gx <= 1; gx++) {
            for (int gy = -1; gy <= 1; gy++) {
                int ix = base.x + gx;
                int iy = base.y + gy;

                if (ix < 0 || ix >= grid.size || iy < 0 || iy >= grid.size) continue;

                float weight = 1.0f; // TEMPORARY, same as p2g
                const Cell& cell = grid.at(ix, iy);
                if (cell.mass > 0.0f) {
                    newV += weight * (cell.v / cell.mass);
                    totalWeight += weight;
                }
            }
        }

        if (totalWeight > 0.0f)
            p.v = newV / totalWeight;

        // Floor collision (prevent downward motion into floor)
        if (p.x.y <= 1.0f && p.v.y < 0.0f) {
            p.v.y = 0.0f;
            p.C[1][0] = 0.0f; // optional: clear y-related shear
            p.C[1][1] = 0.0f;
        }

        // Deformation update
        p.F = (glm::mat2(1.0f) + dt * p.C) * p.F;
    }
}

void ParticleSystem::update() {
    for (auto& p : particles) {
        p.x += dt * p.v;

        if (p.x.y < 1.0f) {
            p.x.y = 1.0f;
            p.v.y = 0.0f;
        }
    }
}