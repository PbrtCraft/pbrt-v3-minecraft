// shapes/quad.cpp*
#include "shapes/quad.h"
#include "paramset.h"
#include "efloat.h"
#include "stats.h"

namespace pbrt {

bool QuadX::Intersect(const Ray &r, Float *tHit, SurfaceInteraction *isect,
                     bool testAlphaTexture) const {
    // Transform _Ray_ to object space
    ProfilePhase p(Prof::ShapeIntersect);
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);

    if(ray.d.x == 0) return false;

    Float thit = -ray.o.x/ray.d.x;
    if (thit <= 0 or thit >= ray.tMax)
        return false;

    Point3f phit = ray(thit);
    if (not inRange(phit.y, phit.z))
      return false;

    Float u = phit.y/l1 +.5, v = phit.z/l2 +.5;
    u = Du*u + u0, v = Dv*v + v0;

    // Initialize _DifferentialGeometry_ from parametric information
    Vector3f dpdv(0, 0, l2/Dv), dpdu(0, l1/Du, 0);
    Normal3f dn(0, 0, 0);

    // Compute error bounds for disk intersection
    Vector3f pError(0, 0, 0);

    // Initialize _SurfaceInteraction_ from parametric information
    *isect = (*ObjectToWorld)(SurfaceInteraction(phit, pError, Point2f(v, u),
                                                 -ray.d, dpdv, dpdu, dn, dn,
                                                 ray.time, this));

    // Update _tHit_
    *tHit = thit;
    return true;
}

Interaction QuadX::Sample(const Point2f &_u, Float *pdf) const {
    Float u = _u.x, v = _u.y;
    Point3f pObj(0, ((u-u0)/Du-.5)*l1, ((v-v0)/Dv-.5)*l2);
    Interaction it;
    it.n = Normalize((*ObjectToWorld)(Normal3f(dir, 0, 0)));
    if (reverseOrientation) it.n *= -1;
    it.p = (*ObjectToWorld)(pObj, Vector3f(0, 0, 0), &it.pError);
    *pdf = 1 / Area();
    return it;
}

static Quad *CreateQuadShape(const Transform *o2w,
                             const Transform *w2o,
                             bool reverseOrientation,
                             const ParamSet &params) {
    Float l1 = params.FindOneFloat("l1", 1);
    Float l2 = params.FindOneFloat("l2", 1);
    Float u0  = params.FindOneFloat("u0", 0);
    Float v0  = params.FindOneFloat("v0", 0);
    Float u1  = params.FindOneFloat("u1", 1);
    Float v1  = params.FindOneFloat("v1", 1);
    Float dir = params.FindOneFloat("dir", 1);
    return new Quad(o2w, w2o, reverseOrientation, l1, l2, dir,
                     u0, v0, u1, v1);
}

std::shared_ptr<QuadX> CreateQuadXShape(const Transform *o2w,
                                        const Transform *w2o,
                                        bool reverseOrientation,
                                        const ParamSet &params) {
    return std::shared_ptr<QuadX>(
               (QuadX*)CreateQuadShape(o2w, w2o, reverseOrientation, params));
}

}  // namespace pbrt
