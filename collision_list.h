#ifndef collision_list_h
#define collision_list_h

#include "utilities_package.h"
#include "collision.h"

#include <memory>
#include <vector>


class collision_list : public collision {

public:

    collision_list() {}
    collision_list(shared_ptr<collision> object) { add(object); }

    void clear() { objects.clear(); }
    void add(shared_ptr<collision> object) { objects.push_back(object); }

    virtual bool hit(
        const ray& r, double t_min, double t_max, collision_record& record) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;

public:
    std::vector<shared_ptr<collision>> objects;

};


bool collision_list::hit(const ray& r, double t_min, double t_max, collision_record& record) const {
    collision_record temp_rec;
    auto hit_anything = false;
    auto closest_so_far = t_max;

    for (const auto& object : objects) {
        if (object->hit(r, t_min, closest_so_far, temp_rec)) {
            hit_anything = true;
            closest_so_far = temp_rec.t;
            record = temp_rec;
        }
    }

    return hit_anything;
}


bool collision_list::bounding_box(double time0, double time1, aabb& output_box) const {
    if (objects.empty()) return false;

    aabb temp_box;
    bool first_box = true;

    for (const auto& object : objects) {
        if (!object->bounding_box(time0, time1, temp_box)) return false;
        output_box = first_box ? temp_box : surrounding_box(output_box, temp_box);
        first_box = false;
    }

    return true;
}


#endif
