#ifndef collision_h
#define collision_h

#include "utilities_package.h"
#include "aabb.h"


class material;


struct collision_record {

    coordinate p;
    vec3 normal;
    shared_ptr<material> mat_ptr;
    double t;
    double u;
    double v;
    bool front_face;

    inline void set_face_normal(const ray& r, const vec3& outward_normal) {
        front_face = dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }

};


class collision {

public:
    virtual bool hit(const ray& r, double t_min, double t_max, collision_record& record) const = 0;
    virtual bool bounding_box(double time0, double time1, aabb& output_box) const = 0;

};


class translate : public collision {


public:

    shared_ptr<collision> ptr;
    vec3 offset;

    translate(shared_ptr<collision> p, const vec3& displacement)
        : ptr(p), offset(displacement) {}

    virtual bool hit(
        const ray& r, double t_min, double t_max, collision_record& rec) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;

};


bool translate::hit(const ray& r, double t_min, double t_max, collision_record& record) const {
    ray moved_r(r.origin() - offset, r.direction(), r.time());
    if (!ptr->hit(moved_r, t_min, t_max, record))
        return false;

    record.p += offset;
    record.set_face_normal(moved_r, record.normal);

    return true;
}


bool translate::bounding_box(double time0, double time1, aabb& output_box) const {
    if (!ptr->bounding_box(time0, time1, output_box))
        return false;

    output_box = aabb(
        output_box.min() + offset,
        output_box.max() + offset);

    return true;
}


class rotate_y : public collision {

public:

    shared_ptr<collision> ptr;
    double sin_theta;
    double cos_theta;
    bool hasbox;
    aabb bbox;

    rotate_y(shared_ptr<collision> p, double angle);

    virtual bool hit(
        const ray& r, double t_min, double t_max, collision_record& record) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
        output_box = bbox;

        return hasbox;
    }

};


rotate_y::rotate_y(shared_ptr<collision> p, double angle) : ptr(p) {

    auto radians = degrees_to_radians(angle);
    sin_theta = sin(radians);
    cos_theta = cos(radians);
    hasbox = ptr->bounding_box(0, 1, bbox);

    coordinate min(infinity, infinity, infinity);
    coordinate max(-infinity, -infinity, -infinity);

    for (int i = 0; i < 2; i++) {

        for (int j = 0; j < 2; j++) {

            for (int k = 0; k < 2; k++) {

                auto x = i * bbox.max().x() + (1 - i) * bbox.min().x();
                auto y = j * bbox.max().y() + (1 - j) * bbox.min().y();
                auto z = k * bbox.max().z() + (1 - k) * bbox.min().z();

                auto newx = cos_theta * x + sin_theta * z;
                auto newz = -sin_theta * x + cos_theta * z;

                vec3 tester(newx, y, newz);

                for (int c = 0; c < 3; c++) {
                    min[c] = fmin(min[c], tester[c]);
                    max[c] = fmax(max[c], tester[c]);
                }

            }

        }

    }

    bbox = aabb(min, max);

}


bool rotate_y::hit(const ray& r, double t_min, double t_max, collision_record& record) const {

    auto origin = r.origin();
    auto direction = r.direction();

    origin[0] = cos_theta * r.origin()[0] - sin_theta * r.origin()[2];
    origin[2] = sin_theta * r.origin()[0] + cos_theta * r.origin()[2];

    direction[0] = cos_theta * r.direction()[0] - sin_theta * r.direction()[2];
    direction[2] = sin_theta * r.direction()[0] + cos_theta * r.direction()[2];

    ray rotated_r(origin, direction, r.time());

    if (!ptr->hit(rotated_r, t_min, t_max, record))
        return false;

    auto p = record.p;
    auto normal = record.normal;

    p[0] = cos_theta * record.p[0] + sin_theta * record.p[2];
    p[2] = -sin_theta * record.p[0] + cos_theta * record.p[2];

    normal[0] = cos_theta * record.normal[0] + sin_theta * record.normal[2];
    normal[2] = -sin_theta * record.normal[0] + cos_theta * record.normal[2];

    record.p = p;
    record.set_face_normal(rotated_r, normal);

    return true;

}


#endif