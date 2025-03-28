#include "mpm.h"
#include <glm/common.hpp>
#include <GL/gl.h>
#include <iostream>

MPM::MPM(int gridSize, int particleCount)
    // 1.  initialise grid - fill your grid array with (grid_res * grid_res) cells.
    : grid(gridSize),

    // 2. create a bunch of particles. set their positions somewhere in your simulation domain.
    particles(particleCount, gridSize) {

    // initialise their deformation gradients to the identity matrix, as they're in their undeformed state.

    // 3. optionally precompute state variables e.g. particle initial volume, if your model calls for it
}

void MPM::step() {
    grid.clear(); // Step 1
    p2g(); // Step 3
    grid.update(); // Step 2
    g2p(); // Step 4
}

void MPM::draw() const {
    glPointSize(4.0f);
    glBegin(GL_POINTS);
    for (const auto& p : particles.getParticles()) {
        glVertex2f(p.x.x / 64.0f * 2.0f - 1.0f, p.x.y / 64.0f * 2.0f - 1.0f);
    }
    glEnd();
}

Weights computeWeights(const glm::vec2& pos, float inv_dx) {
    glm::vec2 xp = pos * inv_dx;
    glm::ivec2 base = glm::ivec2(glm::floor(xp - glm::vec2(0.5f)));
    glm::vec2 fx = xp - glm::vec2(base);

    glm::vec3 wx = {
        0.5f * glm::pow(1.5f - fx.x, 2.0f),
        0.75f - glm::pow(fx.x - 1.0f, 2.0f),
        0.5f * glm::pow(fx.x - 0.5f, 2.0f)
    };

    glm::vec3 wy = {
        0.5f * glm::pow(1.5f - fx.y, 2.0f),
        0.75f - glm::pow(fx.y - 1.0f, 2.0f),
        0.5f * glm::pow(fx.y - 0.5f, 2.0f)
    };

    return Weights{ base, wx, wy, fx };
}



// 2. particle-to-grid (P2G).
void MPM::p2g() {
    // goal: transfers data from particles to our grid
    // TODO: Add grid cell size in later
    float dx = 1.0f; // Grid cell size
    float inv_dx = 1.0f / dx;
    float particleMass = 1.0f; // FIXME: assumeing constant mass for now

    for (auto& p : particles.getParticles()) {
        // 2.1: calculate weights for the 3x3 neighbouring cells surrounding the particle's position
        // on the grid using an interpolation function
        Weights w = computeWeights(p.x, inv_dx);

        // 2.2: calculate quantities like e.g. stress based on constitutive equation

        // 2.3:
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                glm::ivec2 offset(i, j);
                glm::ivec2 node = w.base + offset;

                if (node.x < 0 || node.x >= grid.size || node.y < 0 || node.y >= grid.size)
                    continue;

                float weight = w.wx[i] * w.wy[j];
                // glm::vec2 dpos = ((glm::vec2(offset) - w.fx) * dx);

                Cell& cell = grid.at(node.x, node.y);

                // scatter our particle's momentum to the grid, using the cell's interpolation weight calculated in 2.1
                cell.mass += weight * particleMass;
                cell.velocity += weight * particleMass * p.v;
            }
        }
    }
}

// 4. grid-to-particle (G2P).
void MPM::g2p() {
    // goal: report our grid's findings back to our particles, and integrate their position + velocity forward
    float inv_dx = 1.0f;

    for (auto& p : particles.getParticles()) {
        // 4.1: update particle's deformation gradient using MLS-MPM's velocity gradient estimate
        // Reference: MLS-MPM paper, Eq. 17

        // 4.2: calculate neighbouring cell weights as in step 2.1.
        // note: our particle's haven't moved on the grid at all by this point, so the weights will be identical
        Weights w = computeWeights(p.x, inv_dx);

        // 4.3: calculate our new particle velocities
        p.v = glm::vec2(0.0f);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                glm::ivec2 offset(i, j);
                glm::ivec2 node = w.base + offset;

                if (node.x < 0 || node.x >= grid.size || node.y < 0 || node.y >= grid.size)
                    continue;

                float weight = w.wx[i] * w.wy[j];
                const Cell& cell = grid.at(node.x, node.y);

                // 4.3.1: get this cell's weighted contribution to our particle's new velocity
                p.v += weight * cell.velocity;
            }
        }

        // 4.4: advect particle positions by their velocity
        particles.update(p);
    }
}