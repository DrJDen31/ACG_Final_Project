#pragma once
#include <vector>
#include <glm/glm.hpp>

const float dt = 1.0f / 60.0f; // global timestep
const glm::vec3 gravity(0.0f, -9.8f, 0.0f);

struct Cell {
    glm::vec3 velocity;
    float mass;
};

class Grid {
public:
    int size;
    std::vector<Cell> cells;

    glm::vec3 worldTranslation;
    glm::mat3 worldRotation; // This is actually the inverse rotation

    Grid(int s);
    void update(); // Compute grid velocities, apply boundary conditions
    void clear();  // Zero out all cell masses & velocities
    Cell& at(int x, int y, int z);
};
