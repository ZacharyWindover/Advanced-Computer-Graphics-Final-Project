#ifndef box_h
#define box_h

#include "utilities_package.h"
#include "aarect.h"
#include "collision_list.h"


class box : public collision {
public:
    box() {}
    box(const coordinate& p0, const coordinate& p1, shared_ptr<material> ptr);

    virtual bool hit(const ray& r, double t_min, double t_max, collision_record& record) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
        output_box = aabb(box_min, box_max);
        return true;
    }

public:

    coordinate box_min;
    coordinate box_max;
    collision_list sides;

};


box::box(const coordinate& p0, const coordinate& p1, shared_ptr<material> ptr) {
    
    box_min = p0;
    box_max = p1;

    sides.add(make_shared<xy_rect>(p0.x(), p1.x(), p0.y(), p1.y(), p1.z(), ptr));
    sides.add(make_shared<xy_rect>(p0.x(), p1.x(), p0.y(), p1.y(), p0.z(), ptr));

    sides.add(make_shared<xz_rect>(p0.x(), p1.x(), p0.z(), p1.z(), p1.y(), ptr));
    sides.add(make_shared<xz_rect>(p0.x(), p1.x(), p0.z(), p1.z(), p0.y(), ptr));

    sides.add(make_shared<yz_rect>(p0.y(), p1.y(), p0.z(), p1.z(), p1.x(), ptr));
    sides.add(make_shared<yz_rect>(p0.y(), p1.y(), p0.z(), p1.z(), p0.x(), ptr));
}


bool box::hit(const ray& r, double t_min, double t_max, collision_record& record) const {
    return sides.hit(r, t_min, t_max, record);
}


#endif