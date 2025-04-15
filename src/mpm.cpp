#include "mpm.h"
#include "mesh.h"
#include "argparser.h"
#include <glm/common.hpp>

#ifdef _WIN32
    #include <windows.h>
#endif

#include <GL/gl.h>
#include <thread>

// TODO: remove
#include <iostream>

// for intersection
#include "ray.h"
#include "hit.h"
#include "material.h"

#include "raytree.h"

MPM::MPM(int gridSize, const std::vector<MPMObject>& objects)
    // 1.  initialise grid - fill your grid array with (grid_res * grid_res) cells.
    : grid(gridSize),

    // 2. create a bunch of particles. set their positions somewhere in your simulation domain.
    particles(objects) {
    // initialise their deformation gradients to the identity matrix, as they're in their undeformed state.

    // 3. optionally precompute state variables e.g. particle initial volume, if your model calls for it
    // mu = 20.0f;
    // lambda = 150.0f;
    mu = 1.0f;
    lambda = 1.0f;
    enablePlasticity = false;
}

void MPM::step() {
    grid.clear(); // Step 1
    p2g(); // Step 2
    grid.update(); // Step 3
    g2p(); // Step 4
}

void MPM::draw() const {
    glPointSize(2.5f);
    glBegin(GL_POINTS);
    for (const auto& p : particles.getParticles()) {
        glVertex3f(p.x.x, p.x.y, p.x.z);
    }
    glEnd();
}

// Quadratic B-spline interpolation
Weights computeWeights(const glm::vec3& pos, float inv_dx) {
    glm::vec3 xp = pos * inv_dx;
    glm::ivec3 base = glm::ivec3(glm::floor(xp - glm::vec3(0.5f)));
    glm::vec3 fx = xp - glm::vec3(base);

    glm::vec3 wx = {
        0.5f * glm::pow(1.5f - fx.x, 2.0f),
        0.75f - glm::pow(fx.x - 1.0f, 2.0f),
        0.5f * glm::pow(fx.x - 0.5f, 2.0f)
    };

    glm::vec3 wy = {
        0.5f * glm::pow(1.5f - fx.y, 2.0f),
        0.75f - glm::pow(fx.y - 1.0f, 2.0f),
        0.5f * glm::pow(fx.y - 0.5f, 2.0f)
    }; glm::vec3 wz = {
        0.5f * glm::pow(1.5f - fx.z, 2.0f),
        0.75f - glm::pow(fx.z - 1.0f, 2.0f),
        0.5f * glm::pow(fx.z - 0.5f, 2.0f)
    };

    return Weights{ base, wx, wy, wz, fx };
}

// Minimal 3x3 SVD using Jacobi iteration
// Computes F = U * S * V^T
void svd(const glm::mat3& F, glm::mat3& U, glm::mat3& S, glm::mat3& V) {
    glm::mat3 ATA = glm::transpose(F) * F;

    // V: eigenvectors of ATA
    V = glm::mat3(1.0f);
    for (int iter = 0; iter < 10; ++iter) {
        for (int p = 0; p < 2; ++p) {
        for (int q = p+1; q < 3; ++q) {
            float app = ATA[p][p], aqq = ATA[q][q];
            float apq = ATA[p][q];

            if (fabs(apq) < 1e-5f) continue;

            float phi = 0.5f * atan2(2 * apq, aqq - app);
            float c = cos(phi), s = sin(phi);

            glm::mat3 J(1.0f);
            J[p][p] =  c; J[p][q] = s;
            J[q][p] = -s; J[q][q] = c;

            ATA = glm::transpose(J) * ATA * J;
            V = V * J;
        }}
    }

    // S: singular values from sqrt of eigenvalues of ATA
    glm::vec3 sigma(
        sqrtf(ATA[0][0]),
        sqrtf(ATA[1][1]),
        sqrtf(ATA[2][2])
    );
    S = glm::mat3(0.0f);
    S[0][0] = sigma[0];
    S[1][1] = sigma[1];
    S[2][2] = sigma[2];

    // U = F * V * S⁻¹
    glm::mat3 Sinv(0.0f);
    for (int i = 0; i < 3; ++i)
        if (sigma[i] > 1e-6f)
            Sinv[i][i] = 1.0f / sigma[i];
    U = F * V * Sinv;
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
        glm::mat3 F = p.F; // deformation gradient

        // MPM course page 13, "Kinematics Theory"
        float J = glm::determinant(F);
        float volume = p.volume_0 * J;

        // required terms for Neo-Hookean model (eq. 48, MPM course)
        glm::mat3 F_T = glm::transpose(F);
        glm::mat3 F_inv_T = glm::inverse(F_T);
        glm::mat3 F_minus_F_inv_T = F - F_inv_T;

        // Cauvhy stress = (1 / det(F)) * P *F_T
        glm::mat3 P = mu * F_minus_F_inv_T + lambda * std::log(J) * F_inv_T;
        glm::mat3 stress = (1.0f / J) * P * F_T;

        // (M_p)^-1 = 4, see APIC paper and MPM course page 42
        // this term is used in MLS-MPM paper eq. 16. with quadratic weights, Mp = (1/4) * (delta_x)^2.
        // in this simulation, delta_x = 1, because i scale the rendering of the domain rather than the domain itself.
        // we multiply by dt as part of the process of fusing the momentum and force update for MLS-MPM
        glm::mat3 eq16Term0 = -volume * 4.0f * dt * stress;

        // 2.3:
        for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
            glm::ivec3 offset(i, j, k);
            glm::ivec3 node = w.base + offset;

            if (node.x < 0 || node.x >= grid.size ||
                node.y < 0 || node.y >= grid.size ||
                node.z < 0 || node.z >= grid.size)
                continue;

            float weight = w.wx[i] * w.wy[j] * w.wz[k];
            glm::vec3 dpos = ((glm::vec3(offset) - w.fx) * dx);

            Cell& cell = grid.at(node.x, node.y, node.z);

            // APIC: compute Q = C * dpos
            glm::vec3 Q = p.C * dpos;

            float weightedMass = weight * p.mass;
            cell.mass += weightedMass;

            // scatter our particle's momentum to the grid, using the cell's interpolation weight calculated in old 2.1
            // cell.mass += weight * particleMass;
            // cell.velocity += weight * particleMass * p.v;

            // New 2.1: Fused momentum + stress force contribution
            glm::vec3 momentum = weightedMass * (p.v + Q) + eq16Term0 * dpos * weight;
            cell.velocity += momentum;
        }
        }
        }
    }
}

// 4. grid-to-particle (G2P).
void MPM::g2p() {
    // goal: report our grid's findings back to our particles, and integrate their position + velocity forward
    float dx = 1.0f;
    float inv_dx = 1.0f;

    auto& particleList = particles.getParticles();
    int numThreads = std::thread::hardware_concurrency();
    int chunkSize = particleList.size() / numThreads;

    auto worker = [&](int start, int end) {
        for (int idx = start; idx < end; ++idx) {
            Particle& p = particleList[idx];
            // 4.1: update particle's deformation gradient using MLS-MPM's velocity gradient estimate
            // Reference: MLS-MPM paper, Eq. 17
            p.F = (glm::mat3(1.0f) + dt * p.C) * p.F;

            if (enablePlasticity) {
                // Simple plasticity: limit singular values (stretch) of F
                float F_max = 1.5f;
                float F_min = 0.6f;

                // Optional: QR or SVD (or just approximate with polar decomposition)
                glm::mat3 U, Sigma, V;
                svd(p.F, U, Sigma, V);

                // Clamp singular values
                for (int i = 0; i < 3; ++i)
                    Sigma[i][i] = glm::clamp(Sigma[i][i], F_min, F_max);

                // Recompose clamped F
                p.F = U * Sigma * glm::transpose(V);
            }

            // 4.2: calculate neighbouring cell weights as in step 2.1.
            // note: our particle's haven't moved on the grid at all by this point, so the weights will be identical
            Weights w = computeWeights(p.x, inv_dx);

            // 4.3: calculate our new particle velocities
            p.v = glm::vec3(0.0f);

            p.C = glm::mat3(0.0f); // Reset C before accumulation
            for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                    glm::ivec3 offset(i, j, k);
                    glm::ivec3 node = w.base + offset;

                    if (node.x < 0 || node.x >= grid.size ||
                        node.y < 0 || node.y >= grid.size ||
                        node.z < 0 || node.z >= grid.size)
                        continue;

                    float weight = w.wx[i] * w.wy[j] * w.wz[k];
                    glm::vec3 dpos = (glm::vec3(offset) - w.fx) * dx;
                    const Cell& cell = grid.at(node.x, node.y, node.z);

                    // 4.3.1: get this cell's weighted contribution to our particle's new velocity
                    p.v += weight * cell.velocity;
                    // APIC: C += 4 * inv_dx * outer_product(weight * velocity, dpos)
                    p.C += 4.0f * weight * glm::outerProduct(cell.velocity, dpos);
            }
            }
            }

            // 4.4: advect particle positions by their velocity
            particles.update(p);
        }
    };

    std::vector<std::thread> threads;
    for (int t = 0; t < numThreads; ++t) {
        int start = t * chunkSize;
        int end = (t == numThreads - 1) ? particleList.size() : (t + 1) * chunkSize;
        threads.emplace_back(worker, start, end);
    }

    for (auto& thread : threads) thread.join();
}

bool MPM::intersect(const Ray& r, Hit& h, Vec3f& particle_pos, double threshold) const {
    // snow material, should move to somewhere global or file configured
    Vec3f diffuse = Vec3f(0.28f, 0.28f, 0.28f);
    Vec3f reflective = Vec3f(0.0f, 0.0f, 0.0f);
    Vec3f emitted = Vec3f(0.0f, 0.0f, 0.0f);
    float roughness = 0.7;
    Material* material = new Material("", diffuse, reflective, emitted, roughness);

    // normal of the hit (will come from the particle)
    Vec3f normal;

    // particles
    std::vector<Particle> particleList = particles.getParticles();

    // trackers for nearest valid collision
    double t = -1;
    double t_2 = -1;
    int index = -1;
    int index_2 = -1;
    Vec3f distance_1;
    Vec3f distance_2;
    bool collision = false;
    bool found_1 = false;
    bool found_2 = false;

    // find center of mass
    bool found_center = false;
    Vec3f center_of_mass = Vec3f(0, 0, 0);


    // for each particle, check if we intersect it
    for (int i = 0; i < particleList.size(); i++) {
        int print_num = 0;
        // get the particle's position
        Particle p = particleList[i];
        Vec3f pos = Vec3f(p.x.x / 64.0f * 2.0f - 1.0f, p.x.y / 64.0f * 2.0f - 1.0f, p.x.z / 64.0f * 2.0f - 1.0f);
        if (i < 0) {
            std::cout << std::endl << "particle at " << pos << std::endl;
            std::cout << "ray origin: " << r.getOrigin() << std::endl;
            std::cout << "ray dir: " << r.getDirection() << std::endl;
        }

        Vec3f to_point = pos - r.getOrigin();
        double to_p = to_point.Length();

        Vec3f projected = r.pointAtParameter(to_p);
        Vec3f distance = projected - pos;

        /*

        // X
        // determine where the point would be along the ray given it's x value
        double delta_x = pos.x() - r.getOrigin().x();
        double ts = delta_x / r.getDirection().x();
        Vec3f projected = r.pointAtParameter(ts);

        Vec3f cast = pos - r.getOrigin();
        cast.Normalize();

        // if we are close enough to expected position, register collision
        Vec3f distance = projected - pos;

        // Y
        // determine where the point would be along the ray given it's y value
        double delta_y = pos.y() - r.getOrigin().y();
        double y_ts = delta_y / r.getDirection().y();
        Vec3f y_projected = r.pointAtParameter(y_ts);

        cast = pos - r.getOrigin();
        cast.Normalize();

        // if we are close enough to expected position, register collision
        Vec3f y_distance = y_projected - pos;
        if (y_distance.Length() < distance.Length()) {
            distance = y_distance;
            projected = y_projected;
            ts = y_ts;
        }

        // Z
        // determine where the point would be along the ray given it's y value
        double delta_z = pos.z() - r.getOrigin().z();
        double z_ts = delta_z / r.getDirection().z();
        Vec3f z_projected = r.pointAtParameter(z_ts);

        cast = pos - r.getOrigin();
        cast.Normalize();

        // if we are close enough to expected position, register collision
        Vec3f z_distance = z_projected - pos;
        if (z_distance.Length() < distance.Length()) {
            distance = z_distance;
            projected = z_projected;
            ts = z_ts;
        }
        
        if (i < print_num) {
            Ray checker(r.getOrigin(), cast);
            RayTree::AddReflectedSegment(checker, 0, 5);
            std::cout << "particle would be at " << projected << std::endl;
            std::cout << "particle " << distance.Length() << " away" << std::endl;
        }
        */

        // update bests if best yet
        if (!found_1 || (distance.Length() < threshold && to_p < t)) {
            if (found_1) {
                distance_2 = distance_1;
                index_2 = index;
                t_2 = t;
                found_2 = true;
            }
            distance_1 = distance;
            index = i;
            t = to_p;
            found_1 = true;
        }
    }


    if (found_1 && distance_1.Length() < threshold && t > 0) {
        //std::cout << "t before: " << t << std::endl;
        // find center of mass if not found yet
        /*
        if (!found_center) {
            for (int j = 0; j < particleList.size(); j++) {
                // get the particle's position
                Particle p_c = particleList[j];
                center_of_mass += Vec3f(p_c.x.x / 64.0f * 2.0f - 1.0f, p_c.x.y / 64.0f * 2.0f - 1.0f, p_c.x.z / 64.0f * 2.0f - 1.0f);
            }
            center_of_mass *= 1 / (particleList.size());
        }
        */
        
        t = max(t - (threshold - distance_1.Length()), 0);

        //std::cout << "t after: " << t << std::endl;

        collision = true;
        Particle p = particleList[index];
        Vec3f dir_from_center = r.pointAtParameter(t) - Vec3f(p.x.x / 64.0f * 2.0f - 1.0f, p.x.y / 64.0f * 2.0f - 1.0f, p.x.z / 64.0f * 2.0f - 1.0f);
        normal = dir_from_center + (0.1 * Vec3f(GLOBAL_args->rand(), GLOBAL_args->rand(), GLOBAL_args->rand()));
        normal.Normalize();
    }

    // update hit if collision found
    if (collision && t < h.getT()) h.set(t, material, normal);

    // return result
    return collision;
}
