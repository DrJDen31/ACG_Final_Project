#include "grid.h"
#include <thread>

glm::mat3 computeRotationMatrix(float angle_x_deg, float angle_y_deg,
                                float angle_z_deg);

Grid::Grid(int s) : size(s), cells(s * s * s) {
    // Compute once before simulation step
    glm::mat3 rotation = computeRotationMatrix(0.0f, 0.0f, 0.0f);
    worldTranslation = glm::vec3(0.0f, 0.0f, 0.0f);
    worldRotation = glm::inverse(rotation);
}

bool isInsideBoundary(const glm::vec3& pos_world, int size,
                      const glm::mat3& inverse_rotation,
                      const glm::vec3& translation);

// 3. calculate grid velocities
void Grid::update() {
const int numThreads = std::thread::hardware_concurrency();
    const int chunkSize = cells.size() / numThreads;

    auto worker = [&](int start, int end) {
        for (int i = start; i < end; ++i) {
            Cell& cell = cells[i];

            if (cell.mass > 0.0f) {
                cell.velocity /= cell.mass;
                cell.velocity += dt * gravity;
            }

            // Get coords from flat index
            int x = i % size;
            int y = (i / size) % size;
            int z = i / (size * size);

            glm::vec3 pos_world(x, y, z);
            if (!isInsideBoundary(pos_world, size, worldRotation, worldTranslation)) {
                cell.velocity = glm::vec3(0.0f);
            }
        }
    };

    std::vector<std::thread> threads;
    for (int t = 0; t < numThreads; ++t) {
        int start = t * chunkSize;
        int end = (t == numThreads - 1) ? cells.size() : (t + 1) * chunkSize;
        threads.emplace_back(worker, start, end);
    }

    for (auto& thread : threads) thread.join();
}

void Grid::clear() {
    const int numThreads = std::thread::hardware_concurrency();
    const int chunkSize = cells.size() / numThreads;

    // Clear function
    auto worker = [&](int start, int end) {
        for (int i = start; i < end; ++i) {
            cells[i].velocity = glm::vec3(0.0f);
            cells[i].mass = 0.0f;
        }
    };

    std::vector<std::thread> threads;
    for (int t = 0; t < numThreads; ++t) {
        int start = t * chunkSize;
        int end = (t == numThreads - 1) ? cells.size() : (t + 1) * chunkSize;
        threads.emplace_back(worker, start, end);
    }

    for (auto& thread : threads) thread.join();
}

Cell& Grid::at(int x, int y, int z) {
    return cells[(z * size * size) + (y * size) + x];
}

// Check boundary
bool isInsideBoundary(const glm::vec3& pos_world, int size,
                      const glm::mat3& inverse_rotation,
                      const glm::vec3& translation) {
    glm::vec3 minBound(0.0);
    glm::vec3 maxBound(size * 3.0f);

    glm::vec3 local_pos = inverse_rotation * (pos_world - translation);
    return glm::all(glm::greaterThanEqual(local_pos, minBound)) &&
           glm::all(glm::lessThanEqual(local_pos, maxBound));
}

glm::mat3 computeRotationMatrix(float angle_x_deg, float angle_y_deg,
                                float angle_z_deg) {
    float x = glm::radians(angle_x_deg);
    float y = glm::radians(angle_y_deg);
    float z = glm::radians(angle_z_deg);

    glm::mat3 rotX(
        glm::vec3(1, 0, 0),
        glm::vec3(0, cos(x), -sin(x)),
        glm::vec3(0, sin(x), cos(x)));

    glm::mat3 rotY(
        glm::vec3(cos(y), 0, sin(y)),
        glm::vec3(0, 1, 0),
        glm::vec3(-sin(y), 0, cos(y)));

    glm::mat3 rotZ(
        glm::vec3(cos(z), -sin(z), 0),
        glm::vec3(sin(z), cos(z), 0),
        glm::vec3(0, 0, 1));

    return rotZ * rotY * rotX;
}
