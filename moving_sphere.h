#ifndef moving_sphere_h
#define moving_sphere_h

#include "utilities_package.h"
#include "collision.h"


class moving_sphere : public collision {

public:

    moving_sphere() {}

    moving_sphere(
        coordinate cen0, coordinate cen1, double _time0, double _time1, double r, shared_ptr<material> m)
        : center0(cen0), center1(cen1), time0(_time0), time1(_time1), radius(r), mat_ptr(m)
    {};

    virtual bool hit(
        const ray& r, double t_min, double t_max, collision_record& record) const override;

    virtual bool bounding_box(double _time0, double _time1, aabb& output_box) const override;

    coordinate center(double time) const;

public:

    coordinate center0, center1;
    double time0, time1;
    double radius;
    shared_ptr<material> mat_ptr;

};


coordinate moving_sphere::center(double time) const {

    return center0 + ((time - time0) / (time1 - time0)) * (center1 - center0);

}


bool moving_sphere::bounding_box(double _time0, double _time1, aabb& output_box) const {
    
    aabb box0(
        center(_time0) - vec3(radius, radius, radius),
        center(_time0) + vec3(radius, radius, radius));

    aabb box1(
        center(_time1) - vec3(radius, radius, radius),
        center(_time1) + vec3(radius, radius, radius));

    output_box = surrounding_box(box0, box1);

    return true;
}


bool moving_sphere::hit(const ray& r, double t_min, double t_max, collision_record& record) const {
    
    vec3 oc = r.origin() - center(r.time());
    auto a = r.direction().length_squared();
    auto half_b = dot(oc, r.direction());
    auto c = oc.length_squared() - radius * radius;

    auto discriminant = half_b * half_b - a * c;

    if (discriminant < 0) return false;

    auto sqrtd = sqrt(discriminant);

    // Find the nearest root that lies in the acceptable range.
    auto root = (-half_b - sqrtd) / a;

    if (root < t_min || t_max < root) {
        root = (-half_b + sqrtd) / a;
        if (root < t_min || t_max < root)
            return false;
    }

    record.t = root;
    record.p = r.at(record.t);
    vec3 outward_normal = (record.p - center(r.time())) / radius;
    record.set_face_normal(r, outward_normal);
    record.mat_ptr = mat_ptr;

    return true;
}

#endif