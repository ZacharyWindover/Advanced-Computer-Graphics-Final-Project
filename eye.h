#ifndef eye_h
#define eye_h


#include "utilities_package.h"


class eye {

private:
    coordinate origin;
    coordinate lower_left_corner;
    vec3 horizontal;
    vec3 vertical;
    vec3 u, v, w;
    double lens_radius;
    double time0, time1;  

public:

    eye() : eye(coordinate(0, 0, -1), coordinate(0, 0, 0), vec3(0, 1, 0), 40, 1, 0, 10) {}

    eye(coordinate look_from, coordinate look_at, vec3 v_up,
        double v_fov, double aspect_ratio, double aperture,
        double focus_distance, double time_0 = 0, double time_1 = 0
    ) {

        auto theta = degrees_to_radians(v_fov);
        auto h = tan(theta / 2);
        auto viewport_height = 2.0 * h;
        auto viewport_width = aspect_ratio * viewport_height;

        w = unit_vector(look_from - look_at);
        u = unit_vector(cross(v_up, w));
        v = cross(w, u);

        origin = look_from;
        horizontal = focus_distance * viewport_width * u;
        vertical = focus_distance * viewport_height * v;
        lower_left_corner = origin - horizontal / 2 - vertical / 2 - focus_distance * w;

        lens_radius = aperture / 2;
        time0 = time_0;
        time1 = time_1;


    } 


    void set_origin(coordinate o) { origin = o; }


    ray get_ray(double s, double t) const {
        vec3 rd = lens_radius * random_in_unit_disk();
        vec3 offset = u * rd.x() + v * rd.y();
        return ray(
            origin + offset,
            lower_left_corner + s * horizontal + t * vertical - origin - offset,
            random_double(time0, time1)
        );
    }

};


#endif