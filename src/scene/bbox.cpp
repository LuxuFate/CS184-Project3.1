#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {
  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
    
    double a = (min.x - r.o.x) / r.d.x;
    double b = (max.x - r.o.x) / r.d.x;
    double tmaxx = std::max(a, b);
    double tminx = std::min(a, b);
    if (tminx > tmaxx) {
        std::swap(tminx,tmaxx);
    }

    a = (min.y - r.o.y) / r.d.y;
    b = (max.y - r.o.y) / r.d.y;
    double tmaxy = std::max(a, b);
    double tminy = std::min(a, b);
    if (tminy > tmaxy) {
        std::swap(tminy,tmaxy);
    }
    
    a = (min.z - r.o.z) / r.d.z;
    b = (max.z - r.o.z) / r.d.z;
    double tmaxz = std::max(a, b);
    double tminz = std::min(a, b);
    if (tminz > tmaxz) {
        std::swap(tminz,tmaxz);
    }
    
    t0 = std::max(std::max(tminx, tminy), tminz);
    t1 = std::min(std::min(tmaxx, tmaxy), tmaxz);
    
    if (t0 > r.max_t || t1 < r.min_t) {
        return false;
    }
    
    return t0 <= t1;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
