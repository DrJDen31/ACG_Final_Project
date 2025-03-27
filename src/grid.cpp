#include "grid.h"

Grid::Grid(int s) : size(s), cells(s * s) {}

void Grid::update() {
    for (auto& cell : cells) {
        if (cell.mass > 0) {
            cell.v /= cell.mass;
            cell.v += glm::vec2(0, -0.2f); // gravity
        }
    }
}

void Grid::clear() {
    for (auto& c : cells) {
        c.v = glm::vec2(0.0f);
        c.mass = 0.0f;
    }
}

Cell& Grid::at(int x, int y) {
    return cells[y * size + x];
}
