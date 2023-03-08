#ifndef ray_h
#define ray_h

#include "vec3.h"


class ray {

private:

    coordinate Ox;
    vec3 Dx;
    double Tx;


public:

    ray() {}

    ray(const coordinate& o, const vec3& d)
        : Ox(o), Dx(d), Tx(0)
    {}

    ray(const coordinate& o, const vec3& d, double t)
        : Ox(o), Dx(d), Tx(t)
    {}

    void set_origin(const coordinate& o) { Ox = o; }
    void set_direction(const vec3& d) { Dx = d; }
    void set_time(const double t) { Tx = t; }

    coordinate origin() const { return Ox; }
    vec3 direction() const { return Dx; }
    double time() const { return Tx; }

    coordinate at(double t) const {
        return Ox + t * Dx;
    }

};

#endif