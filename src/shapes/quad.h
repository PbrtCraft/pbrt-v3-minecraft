#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_QUAD_H
#define PBRT_SHAPES_QUAD_H

// shapes/quad.h*
#include "shape.h"

namespace pbrt {

// Quad Declarations
class Quad : public Shape {
  public:
    // Quad Public Method
    Quad(const Transform *o2w, const Transform *w2o, bool ro,
         Float l1, Float l2, Float dir,
         Float u0, Float v0, Float u1, Float v1):
          Shape(o2w, w2o, ro), l1(l1), l2(l2), dir(dir),
          u0(u0), v0(v0), u1(u1), v1(v1),
          Du(u1 - u0), Dv(v1 - v0) { } ;

    bool Intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect,
                   bool testAlphaTexture) const {
        LOG(FATAL) << "Quad::Intersect not implemented.";
        return false;
    }

    Interaction Sample(const Point2f &u, Float *pdf) const {
        LOG(FATAL) << "Quad::Sample not implemented.";
        return Interaction();
    }

    Bounds3f ObjectBound() const {
        Float rad = std::max(l1, l2)/2;
        Point3f p1(-rad, -rad, -rad), p2(rad, rad, rad);
        return Bounds3f(p1, p2);
    }
    Float Area() const { return l1*l2; };

  protected:
    // Quad Private Method 
    const Float l1, l2, dir, u0, v0, u1, v1, Du, Dv;
    bool inRange(Float x, Float y) const {
        return -l1/2 <= x and x <= l1/2 and
               -l2/2 <= y and y <= l2/2;
    }
};

// QuadX Deckarations
class QuadX : public Quad {
  public:
  // QuadX Public Method
    bool Intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect,
                   bool testAlphaTexture) const;
    Interaction Sample(const Point2f &u, Float *pdf) const;
};

std::shared_ptr<QuadX> CreateQuadXShape(const Transform *o2w,
                                        const Transform *w2o,
                                        bool reverseOrientation,
                                        const ParamSet &params);

}  // namespace pbrt

#endif  // PBRT_SHAPES_CONE_H
