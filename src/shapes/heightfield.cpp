
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


// shapes/heightfield.cpp*
#include "shapes/heightfield.h"
#include "shapes/triangle.h"
#include "paramset.h"

namespace pbrt {

// Heightfield Definitions
std::vector<std::shared_ptr<Shape>> CreateHeightfield(
    const Transform *ObjectToWorld, const Transform *WorldToObject,
    bool reverseOrientation, const ParamSet &params) {
    int nx = params.FindOneInt("nu", -1);
    int nz = params.FindOneInt("nv", -1);
    int nitems;
    const Float *y = params.FindFloat("Py", &nitems);
    CHECK_EQ(nitems, nx * nz);
    CHECK(nx != -1 && nz != -1 && y != nullptr);

    int ntris = 2 * (nx - 1) * (nz - 1);
    std::unique_ptr<int[]> indices(new int[3 * ntris]);
    std::unique_ptr<Point3f[]> P(new Point3f[nx * nz]);
    std::unique_ptr<Point2f[]> uvs(new Point2f[nx * nz]);
    int nverts = nx * nz;
    // Compute heightfield vertex positions
    int pos = 0;
    for (int z = 0; z < nz; ++z) {
        for (int x = 0; x < nx; ++x) {
            P[pos].x = uvs[pos].x = (float)x / (float)(nx - 1);
            P[pos].z = uvs[pos].y = (float)z / (float)(nz - 1);
            P[pos].y = y[pos];
            ++pos;
        }
    }

    // Fill in heightfield vertex offset array
    int *vp = indices.get();
    for (int z = 0; z < nz - 1; ++z) {
        for (int x = 0; x < nx - 1; ++x) {
#define VERT(x, y) ((x) + (y)*nx)
            *vp++ = VERT(x, z);
            *vp++ = VERT(x + 1, z + 1);
            *vp++ = VERT(x + 1, z);

            *vp++ = VERT(x, z);
            *vp++ = VERT(x, z + 1);
            *vp++ = VERT(x + 1, z + 1);
        }
#undef VERT
    }

    return CreateTriangleMesh(ObjectToWorld, WorldToObject, reverseOrientation,
                              ntris, indices.get(), nverts, P.get(), nullptr,
                              nullptr, uvs.get(), nullptr, nullptr);
}

}  // namespace pbrt
