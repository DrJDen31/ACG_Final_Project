#include "grid.h"

Grid::Grid(int s) : size(s), cells(s * s) {}

// 3. calculate grid velocities
void Grid::update() {
    int index = 0;

    for (auto& cell : cells) {
        // 3.1: calculate grid velocity based on momentum found in the P2G stage
        if (cell.mass > 0.0f) {
            cell.velocity /= cell.mass;
            cell.velocity += dt * gravity;
        }

        // Get 2D coords from flat index
        int x = index % size;
        int y = index / size;

        // 3.2: enforce boundary conditions
        if (x < 2 || x > size - 3 || y < 2 || y > size - 3) {
            cell.velocity = glm::vec2(0.0f);
        }

        ++index;
    }
}

void Grid::clear() {
    for (auto& c : cells) {
        c.velocity = glm::vec2(0.0f);
        c.mass = 0.0f;
    }
}

Cell& Grid::at(int x, int y) {
    return cells[y * size + x];
}
