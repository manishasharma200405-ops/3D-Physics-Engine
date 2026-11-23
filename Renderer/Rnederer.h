#pragma once
#include <vector>
#include "Core/EntityManager.h"
#include "Solver/ConstraintSolver.h"

// draws the whole scene with a 3D look (no Windows code in here):
// a room (back wall, floor, side wall), lit spheres and rods, soft shadows,
// and water that is an extruded slab you can see through
class Renderer {
public:
    Renderer(int pxW, int pxH, float pixelsPerUnit);
    // fills pix (0x00RRGGBB, row by row from the top)
    void draw(const EntityManager& em, const Joints& jt, unsigned int* pix);
private:
    int pw, ph, fw, fh;
    float scale;
    std::vector<unsigned int> wall;         // the room, drawn once
    std::vector<float> dens, spd, ext;      // low-resolution water fields

    void buildWall();
    void fillFields(const EntityManager& em);
    void shadows(const EntityManager& em, const Joints& jt, unsigned int* pix) const;
    void solids(const EntityManager& em, const Joints& jt, unsigned int* pix) const;
    void waterShadow(unsigned int* pix) const;
    void waterBack(unsigned int* pix) const;
    void waterFront(unsigned int* pix) const;
    template <class F>
    void scan(const std::vector<float>& f, float minCorner, int offCx, int offCy, F fn) const;
};