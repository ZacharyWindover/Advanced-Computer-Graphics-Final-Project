#ifndef sphere_h
#define sphere_h

#include "utilities_package.h"
#include "collision.h"


class sphere : public collision {

public:

    coordinate center;
    double radius;
    shared_ptr<material> mat_ptr;

    sphere() {}

    sphere(coordinate c, double r, shared_ptr<material> m)
        : center(c), radius(r), mat_ptr(m) {};

    virtual bool hit(
        const ray& r, double t_min, double t_max, collision_record& record) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;


private:

    static void get_sphere_uv(const coordinate& p, double& u, double& v) {

        auto theta = acos(-p.y());
        auto phi = atan2(-p.z(), p.x()) + pi;

        u = phi / (2 * pi);
        v = theta / pi;

    }

};


bool sphere::bounding_box(double time0, double time1, aabb& output_box) const {

    output_box = aabb(
        center - vec3(radius, radius, radius),
        center + vec3(radius, radius, radius));

    return true;

}


bool sphere::hit(const ray& r, double t_min, double t_max, collision_record& record) const {

    vec3 oc = r.origin() - center;
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
    vec3 outward_normal = (record.p - center) / radius;
    record.set_face_normal(r, outward_normal);
    get_sphere_uv(outward_normal, record.u, record.v);
    record.mat_ptr = mat_ptr;

    return true;
}


#endif