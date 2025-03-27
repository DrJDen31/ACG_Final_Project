#pragma once
#include <vector>
#include <glm/glm.hpp>

struct Cell {
    glm::vec2 v = glm::vec2(0.0f);
    float mass = 10.0f;
};

class Grid {
public:
    int size;
    std::vector<Cell> cells;

    Grid(int s);
    void update();
    void clear();
    Cell& at(int x, int y);
};
