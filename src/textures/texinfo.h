#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_TEXTURES_TEXINFO_H
#define PBRT_TEXTURES_TEXINFO_H

#include "pbrt.h"
#include "texture.h"
#include "mipmap.h"
#include "paramset.h"
#include <map>

namespace pbrt {

// TexInfo Declarations
struct TexInfo {
    TexInfo(const std::string &f, bool dt, Float ma, ImageWrap wm, Float sc,
            bool gamma)
        : filename(f),
          doTrilinear(dt),
          maxAniso(ma),
          wrapMode(wm),
          scale(sc),
          gamma(gamma) {}
    std::string filename;
    bool doTrilinear;
    Float maxAniso;
    ImageWrap wrapMode;
    Float scale;
    bool gamma;
    bool operator<(const TexInfo &t2) const {
        if (filename != t2.filename) return filename < t2.filename;
        if (doTrilinear != t2.doTrilinear) return doTrilinear < t2.doTrilinear;
        if (maxAniso != t2.maxAniso) return maxAniso < t2.maxAniso;
        if (scale != t2.scale) return scale < t2.scale;
        if (gamma != t2.gamma) return !gamma;
        return wrapMode < t2.wrapMode;
    }
};

}  // namespace pbrt

#endif  // PBRT_TEXTURES_IMAGEMAP_H
