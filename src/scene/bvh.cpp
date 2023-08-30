#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {
  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

    BBox bbox;

    size_t numPrimitive = 0;
    Vector3D avg = 0;
    for (auto p = start; p != end; p++) {
        BBox bb = (*p)->get_bbox();
        bbox.expand(bb);
        avg += (*p)->get_bbox().centroid();
        numPrimitive++;
    }
    avg = avg/numPrimitive;
    double midx = avg.x;
    double midy = avg.y;
    double midz = avg.z;
    
    BVHNode *node = new BVHNode(bbox);
    
    if (numPrimitive <= max_leaf_size) {
        node->start = start;
        node->end = end;
        return node;
    }
    
    std::vector<Primitive *>::iterator mid;
    
    Vector3D allC = bbox.centroid();
    int axis;
    if (allC.x > allC.y && allC.x > allC.z) {
        axis = 0;
    } else if (allC.y > allC.x && allC.y > allC.z) {
        axis = 1;
    } else {
        axis = 2;
    }
    
    for (auto prim = start; prim != end; prim++) {
//        Vector3D centroid = (*prim)->get_bbox().centroid();
        switch(axis) {
            case 0: {
                mid = std::partition(start, end, [midx](Primitive *p){return p->get_bbox().centroid().x <=
                    midx;});
                break;
            }
            case 1: {
                mid = std::partition(start, end, [midy](Primitive *p){return p->get_bbox().centroid().y <=
                    midy;});
                break;
            }
            case 2: {
                mid = std::partition(start, end, [midz](Primitive *p){return p->get_bbox().centroid().z <=
                    midz;});
                break;
            }
        }
    }
    
    node->l = construct_bvh(start, mid, max_leaf_size);
    node->r = construct_bvh(mid, end, max_leaf_size);
    
    node->start = start;
    node->end = end;

    return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
    
    double t0 = ray.min_t;
    double t1 = ray.max_t;

    if (!node->bb.intersect(ray, t0, t1)) {
        return false;
    }

    if (t1 < ray.min_t || t0 > ray.max_t){
        return false;
    }

    if (node->isLeaf()) {
        for (auto p = node->start; p != node->end; p++) {
            if ((*p) -> has_intersection(ray)) {
                return true;
            }
        }
    }
    return has_intersection(ray, node->l) || has_intersection(ray, node->r);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
    
    double t0 = ray.min_t;
    double t1 = ray.max_t;

    if (!node->bb.intersect(ray, t0, t1)) {
        return false;
    }

    if (t1 < ray.min_t || t0 > ray.max_t){
        return false;
    }

    if (node->isLeaf()) {
        bool hit = false;
        for (auto p = node->start; p != node->end; p++) {
//            total_isects++;
            if ((*p)->intersect(ray, i)) {
                hit = true;
            }
        }
        return hit;
    }
    
    bool hit1 = intersect(ray, i, node->l);
    bool hit2 = intersect(ray, i, node->r);

    return hit1 || hit2;
}


} // namespace SceneObjects
} // namespace CGL
