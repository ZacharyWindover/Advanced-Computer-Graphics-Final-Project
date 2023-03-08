#ifndef aarect_h
#define aarect_h


#include "utilities_package.h"
#include "collision.h"


class xy_rect : public collision {

public:
    xy_rect() {}

    xy_rect(
        double _x0, double _x1, double _y0, double _y1, double _k, shared_ptr<material> mat
    ) : x0(_x0), x1(_x1), y0(_y0), y1(_y1), k(_k), mp(mat) {};

    virtual bool hit(const ray& r, double t_min, double t_max, collision_record& record) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
        output_box = aabb(coordinate(x0, y0, k - 0.0001), coordinate(x1, y1, k + 0.0001));
        return true;
    }

public:
    shared_ptr<material> mp;
    double x0, x1, y0, y1, k;
};


class xz_rect : public collision {

public:
    xz_rect() {}

    xz_rect(
        double _x0, double _x1, double _z0, double _z1, double _k, shared_ptr<material> mat
    ) : x0(_x0), x1(_x1), z0(_z0), z1(_z1), k(_k), mp(mat) {};

    virtual bool hit(const ray& r, double t_min, double t_max, collision_record& record) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
        // The bounding box must have non-zero width in each dimension, so pad the Y
        // dimension a small amount.
        output_box = aabb(coordinate(x0, k - 0.0001, z0), coordinate(x1, k + 0.0001, z1));
        return true;
    }

public:
    shared_ptr<material> mp;
    double x0, x1, z0, z1, k;
};


class yz_rect : public collision {

public:

    shared_ptr<material> mp;
    double y0, y1, z0, z1, k;

    yz_rect() {}

    yz_rect(
        double _y0, double _y1, double _z0, double _z1, double _k, shared_ptr<material> mat
    ) : y0(_y0), y1(_y1), z0(_z0), z1(_z1), k(_k), mp(mat) {};

    virtual bool hit(const ray& r, double t_min, double t_max, collision_record& record) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
        // The bounding box must have non-zero width in each dimension, so pad the X
        // dimension a small amount.
        output_box = aabb(coordinate(k - 0.0001, y0, z0), coordinate(k + 0.0001, y1, z1));
        return true;
    }

};


bool xy_rect::hit(const ray& r, double t_min, double t_max, collision_record& record) const {

    auto t = (k - r.origin().z()) / r.direction().z();

    if (t < t_min || t > t_max)
        return false;

    auto x = r.origin().x() + t * r.direction().x();
    auto y = r.origin().y() + t * r.direction().y();

    if (x < x0 || x > x1 || y < y0 || y > y1)
        return false;

    record.u = (x - x0) / (x1 - x0);
    record.v = (y - y0) / (y1 - y0);
    record.t = t;

    auto outward_normal = vec3(0, 0, 1);

    record.set_face_normal(r, outward_normal);
    record.mat_ptr = mp;
    record.p = r.at(t);

    return true;
}


bool xz_rect::hit(const ray& r, double t_min, double t_max, collision_record& record) const {

    auto t = (k - r.origin().y()) / r.direction().y();

    if (t < t_min || t > t_max)
        return false;

    auto x = r.origin().x() + t * r.direction().x();
    auto z = r.origin().z() + t * r.direction().z();

    if (x < x0 || x > x1 || z < z0 || z > z1)
        return false;

    record.u = (x - x0) / (x1 - x0);
    record.v = (z - z0) / (z1 - z0);
    record.t = t;

    auto outward_normal = vec3(0, 1, 0);

    record.set_face_normal(r, outward_normal);
    record.mat_ptr = mp;
    record.p = r.at(t);

    return true;

}


bool yz_rect::hit(const ray& r, double t_min, double t_max, collision_record& record) const {

    auto t = (k - r.origin().x()) / r.direction().x();

    if (t < t_min || t > t_max)
        return false;

    auto y = r.origin().y() + t * r.direction().y();
    auto z = r.origin().z() + t * r.direction().z();

    if (y < y0 || y > y1 || z < z0 || z > z1)
        return false;

    record.u = (y - y0) / (y1 - y0);
    record.v = (z - z0) / (z1 - z0);
    record.t = t;

    auto outward_normal = vec3(1, 0, 0);

    record.set_face_normal(r, outward_normal);
    record.mat_ptr = mp;
    record.p = r.at(t);

    return true;
}

#endif