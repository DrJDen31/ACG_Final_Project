#include "mpm.h"

MPM::MPM(int gridSize, int particleCount)
    : grid(gridSize) {
    particles.init(particleCount, gridSize);
}

void MPM::step() {
    particles.p2g(grid);
    grid.update();
    particles.g2p(grid);
    particles.update();
}

void MPM::draw() const {
    particles.draw();
}
