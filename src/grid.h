#pragma once
#include <vector>
#include <glm/glm.hpp>

const float dt = 1.0f / 1200.0f; // global timestepTODO: put somewhere else
const glm::vec2 gravity(0.0f, -9.8f);

struct Cell {
    glm::vec2 velocity;
    float mass;
};

class Grid {
public:
    int size;
    std::vector<Cell> cells;

    Grid(int s);
    void update(); // Compute grid velocities, apply boundary conditions
    void clear();  // Zero out all cell masses & velocities
    Cell& at(int x, int y);
};
