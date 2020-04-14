#pragma once


#include "p3d/core.h"

namespace p3d::palette::impl
{
inline Vec3f pal(float t, const Vec3f& a, const Vec3f& b, const Vec3f& c,
                 const Vec3f& d)
{
    Vec3f z = 6.28318*(c*t+d);
    Vec3f out;
    out << a(0) + b(0) * cosf(z(0)), a(1) + b(1) * cosf(z(1)),
        a(2) + b(2) * cosf(z(2));
    return out;
}
}  // namespace p3d::palette::impl

namespace p3d::palette
{
// from https://iquilezles.org/www/articles/palettes/palettes.htm
static Vec3f color1(float t)
{
    return impl::pal(t, Vec3f(0.5f, 0.5f, 0.5f), Vec3f(0.5f, 0.5f, 0.5f),
                     Vec3f(1.0f, 1.0f, 1.0f), Vec3f(0.0f, 0.33f, 0.67f));
}

static Vec3f color2(float t)
{
    return impl::pal(t, Vec3f(0.5f, 0.5f, 0.5f), Vec3f(0.5f, 0.5f, 0.5f),
                     Vec3f(1.0f, 1.0f, 1.0f), Vec3f(0.0f, 0.10f, 0.20f));
}

static Vec3f color6(float t)
{
    return impl::pal(t, Vec3f(0.5f, 0.5f, 0.5f), Vec3f(0.5f, 0.5f, 0.5f),
                     Vec3f(2.0f, 1.0f, 0.0f), Vec3f(0.5f, 0.20f, 0.25f));
}

static Vec3f color7(float t)
{
    return impl::pal(t, Vec3f(0.8f, 0.5f, 0.4f), Vec3f(0.2f, 0.4f, 0.2f),
                     Vec3f(2.0f, 1.0f, 1.0f), Vec3f(0.0f, 0.25f, 0.25f));
}
}  // namespace p3d::palette

//col = pal( p.x, vec3(0.8,0.5,0.4),vec3(0.2,0.4,0.2),vec3(2.0,1.0,1.0),vec3(0.0,0.25,0.25) );
