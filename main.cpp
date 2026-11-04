#include <cstdio>
#include <random>
#include <vector>
#include <windows.h>

#ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#endif

const int   COLS = 80, ROWS = 24;      // size of the text "screen"
const float W = 80.f, H = 24.f;        // world size
const float GRAVITY = 30.f;            // downward acceleration
const float RESTITUTION = 0.8f;        // fraction of speed kept after a bounce
const float DRAG = 0.05f;              // air resistance
const float DT = 1.f / 120.f;          // physics time step

struct Particle { float x, y, vx, vy; };

void step(std::vector<Particle>& ps, float dt) {
    for (auto& p : ps) {
        // update velocity first, then position
        p.vx += (-DRAG * p.vx) * dt;
        p.vy += (GRAVITY - DRAG * p.vy) * dt;
        p.x += p.vx * dt;
        p.y += p.vy * dt;
        // bounce off the walls
        if (p.x < 0)     { p.x = 0;     p.vx = -p.vx * RESTITUTION; }
        if (p.x > W - 1) { p.x = W - 1; p.vx = -p.vx * RESTITUTION; }
        if (p.y < 0)     { p.y = 0;     p.vy = -p.vy * RESTITUTION; }
        if (p.y > H - 1) { p.y = H - 1; p.vy = -p.vy * RESTITUTION; }
    }
}

void draw(const std::vector<Particle>& ps, double t) {
    char grid[ROWS][COLS + 1];
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) grid[r][c] = ' ';
        grid[r][COLS] = '\0';
    }
    for (auto& p : ps) grid[(int)p.y][(int)p.x] = 'o';

    std::printf("\033[H");   // move cursor to top-left (no flicker)
    for (int r = 0; r < ROWS; r++) std::printf("%s\n", grid[r]);
    std::printf("t=%.2fs  particles=%d   (Ctrl+C to quit)\n", t, (int)ps.size());
    std::fflush(stdout);
}

int main() {
    // turn on ANSI escape codes in the Windows console
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    GetConsoleMode(h, &mode);
    SetConsoleMode(h, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);

    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> rx(-15.f, 15.f), ry(-25.f, 0.f);

    std::vector<Particle> ps;
    for (int i = 0; i < 40; i++)
        ps.push_back({W / 2, H / 2, rx(rng), ry(rng)});

    std::printf("\033[2J");   // clear the screen once
    double t = 0;
    while (true) {
        for (int i = 0; i < 2; i++) { step(ps, DT); t += DT; }
        draw(ps, t);
        Sleep(16);
    }
}