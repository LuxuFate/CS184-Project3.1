#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {
  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
    double a = dot(r.d, r.d);
    double b = dot((2.0 * (r.o - this->o)), r.d);
    double c = dot(r.o - this->o, r.o - this->o) - this->r2;
    double disc = b*b - 4.0*a*c;
    if (disc < 0) {
        return false;
    }
    
    t1 = (-b + sqrt(disc))/(2.0*a);
    t2 = (-b - sqrt(disc))/(2.0*a);
    
    if (t2 < t1) {
        std::swap(t1, t2);
    }
    
    if (t1 > r.min_t && t1 < r.max_t && t2 < r.max_t && t2 > r.min_t) {
        r.max_t = min(t1, t2);
    } else if (t1 > r.min_t && t1 < r.max_t) {
        r.max_t = t1;
    } else if (t2 < r.max_t && t2 > r.min_t) {
        r.max_t = t2;
    } else {
        return false;
    }
    return true;
}

bool Sphere::has_intersection(const Ray &r) const {
  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
    double t1;
    double t2;
    bool intersect = test(r, t1, t2);
    
    return intersect;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {
  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
    double t1;
    double t2;
    if (test(r, t1, t2)) {
        i->t = r.max_t;
        Vector3D n = r.o + r.d*r.max_t - this->o;
        n.normalize();
        i->n = n;
        i->primitive = this;
        i->bsdf = get_bsdf();
        return true;
    }
    return false;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
