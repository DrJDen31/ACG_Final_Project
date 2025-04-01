#include "grid.h"

Grid::Grid(int s) : size(s), cells(s * s * s) {}

// 3. calculate grid velocities
void Grid::update() {
    int index = 0;

    for (auto& cell : cells) {
        // 3.1: calculate grid velocity based on momentum found in the P2G stage
        if (cell.mass > 0.0f) {
            cell.velocity /= cell.mass;
            cell.velocity += dt * gravity;
        }

        // Get coords from flat index
        int x = index % size;
        int y = (index / size) % size;
        int z = index / (size * size);


        // 3.2: enforce boundary conditions
        if (x < 2 || x > size - 3 ||
            y < 2 || y > size - 3 ||
            z < 2 || z > size - 3) {
            cell.velocity = glm::vec3(0.0f);
        }

        ++index;
    }
}

void Grid::clear() {
    for (auto& c : cells) {
        c.velocity = glm::vec3(0.0f);
        c.mass = 0.0f;
    }
}

Cell& Grid::at(int x, int y, int z) {
    return cells[(z * size * size) + (y * size) + x];
}
