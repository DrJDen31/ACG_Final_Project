#include "mpm.h"
#include <glm/common.hpp>
#include <GL/gl.h>

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

// Quadratic B-spline interpolation
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

    for (auto& p : particles.getParticles()) {
        // 2.1: calculate weights for the 3x3 neighbouring cells surrounding the particle's position
        // on the grid using an interpolation function
        Weights w = computeWeights(p.x, inv_dx);

        // 2.2: calculate quantities like e.g. stress based on constitutive equation
        glm::mat2 F = p.F; // deformation gradient

        // MPM course page 13, "Kinematics Theory"
        float J = glm::determinant(F);
        float volume = p.volume_0 * J;

        // required terms for Neo-Hookean model (eq. 48, MPM course)
        glm::mat2 F_T = glm::transpose(F);
        glm::mat2 F_inv_T = glm::inverse(F_T);
        glm::mat2 F_minus_F_inv_T = F - F_inv_T;

        // Tunable parameters // TODO: put in initalizer
        float mu = 1400.0f; // shear modulus
        float lambda = 1400.0f; // bulk modulus
        // Cauvhy stress = (1 / det(F)) * P *F_T
        glm::mat2 P = mu * F_minus_F_inv_T + lambda * std::log(J) * F_inv_T;
        glm::mat2 stress = (1.0f / J) * P * F_T;

        // (M_p)^-1 = 4, see APIC paper and MPM course page 42
        // this term is used in MLS-MPM paper eq. 16. with quadratic weights, Mp = (1/4) * (delta_x)^2.
        // in this simulation, delta_x = 1, because i scale the rendering of the domain rather than the domain itself.
        // we multiply by dt as part of the process of fusing the momentum and force update for MLS-MPM
        glm::mat2 eq16Term0 = -volume * 4.0f * dt * stress;

        // 2.3:
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                glm::ivec2 offset(i, j);
                glm::ivec2 node = w.base + offset;

                if (node.x < 0 || node.x >= grid.size || node.y < 0 || node.y >= grid.size)
                    continue;

                float weight = w.wx[i] * w.wy[j];
                glm::vec2 dpos = ((glm::vec2(offset) - w.fx) * dx);

                Cell& cell = grid.at(node.x, node.y);

                // APIC: compute Q = C * dpos
                glm::vec2 Q = p.C * dpos;

                float weightedMass = weight * p.mass;
                cell.mass += weightedMass;

                // scatter our particle's momentum to the grid, using the cell's interpolation weight calculated in old 2.1
                // cell.mass += weight * particleMass;
                // cell.velocity += weight * particleMass * p.v;

                // New 2.1: Fused momentum + stress force contribution
                glm::vec2 momentum = weightedMass * (p.v + Q) + eq16Term0 * dpos * weight;
                cell.velocity += momentum;
            }
        }
    }
}

// 4. grid-to-particle (G2P).
void MPM::g2p() {
    // goal: report our grid's findings back to our particles, and integrate their position + velocity forward
    float dx = 1.0f;
    float inv_dx = 1.0f;

    for (auto& p : particles.getParticles()) {
        // 4.1: update particle's deformation gradient using MLS-MPM's velocity gradient estimate
        // Reference: MLS-MPM paper, Eq. 17
        p.F = (glm::mat2(1.0f) + dt * p.C) * p.F;

        // 4.2: calculate neighbouring cell weights as in step 2.1.
        // note: our particle's haven't moved on the grid at all by this point, so the weights will be identical
        Weights w = computeWeights(p.x, inv_dx);

        // 4.3: calculate our new particle velocities
        p.v = glm::vec2(0.0f);

        p.C = glm::mat2(0.0f); // Reset C before accumulation
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                glm::ivec2 offset(i, j);
                glm::ivec2 node = w.base + offset;

                if (node.x < 0 || node.x >= grid.size || node.y < 0 || node.y >= grid.size)
                    continue;

                float weight = w.wx[i] * w.wy[j];
                glm::vec2 dpos = (glm::vec2(offset) - w.fx) * dx;
                const Cell& cell = grid.at(node.x, node.y);

                // 4.3.1: get this cell's weighted contribution to our particle's new velocity
                p.v += weight * cell.velocity;
                // APIC: C += 4 * inv_dx * outer_product(weight * velocity, dpos)
                p.C += 4.0f * weight * glm::outerProduct(cell.velocity, dpos);
            }
        }

        // 4.4: advect particle positions by their velocity
        particles.update(p);
    }
}