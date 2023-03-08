#ifndef constant_medium_h
#define constant_medium_h

#include "utilities_package.h"
#include "collision.h"
#include "material.h"
#include "texture.h"


class constant_medium : public collision {

public:

    shared_ptr<collision> boundary;
    shared_ptr<material> phase_function;
    double neg_inv_density;

    constant_medium(shared_ptr<collision> b, double d, shared_ptr<texture> a)
        : boundary(b),
        neg_inv_density(-1 / d),
        phase_function(make_shared<isotropic>(a))
    {}

    constant_medium(shared_ptr<collision> b, double d, colour c)
        : boundary(b),
        neg_inv_density(-1 / d),
        phase_function(make_shared<isotropic>(c))
    {}

    virtual bool hit(
        const ray& r, double t_min, double t_max, collision_record& record) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
        return boundary->bounding_box(time0, time1, output_box);
    }


};


bool constant_medium::hit(const ray& r, double t_min, double t_max, collision_record& record) const {

    const bool enableDebug = false;
    const bool debugging = enableDebug && random_double() < 0.00001;

    collision_record record_1, record_2;

    if (!boundary->hit(r, -infinity, infinity, record_1))
        return false;

    if (!boundary->hit(r, record_1.t + 0.0001, infinity, record_2))
        return false;

    if (debugging) std::cerr << "\nt_min=" << record_1.t << ", t_max=" << record_2.t << '\n';

    if (record_1.t < t_min) record_1.t = t_min;
    if (record_2.t > t_max) record_2.t = t_max;

    if (record_1.t >= record_2.t)
        return false;

    if (record_1.t < 0)
        record_1.t = 0;

    const auto ray_length = r.direction().length();
    const auto distance_inside_boundary = (record_2.t - record_1.t) * ray_length;
    const auto hit_distance = neg_inv_density * log(random_double());

    if (hit_distance > distance_inside_boundary)
        return false;

    record.t = record_1.t + hit_distance / ray_length;
    record.p = r.at(record.t);

    if (debugging) {
        std::cerr << "hit_distance = " << hit_distance << '\n'
            << "rec.t = " << record.t << '\n'
            << "rec.p = " << record.p << '\n';
    }

    record.normal = vec3(1, 0, 0);  // arbitrary
    record.front_face = true;     // also arbitrary
    record.mat_ptr = phase_function;

    return true;
}

#endif
