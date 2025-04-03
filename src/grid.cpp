#include "grid.h"
#include <thread>

Grid::Grid(int s) : size(s), cells(s * s * s) {}

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

            if (x < 2 || x > size - 3 ||
                y < 2 || y > size - 3 ||
                z < 2 || z > size - 3) {
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
