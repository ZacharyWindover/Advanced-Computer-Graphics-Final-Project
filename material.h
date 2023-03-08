#ifndef material_h
#define material_h

#include "utilities_package.h"
#include "collision.h"
#include "texture.h"


class material {
public:
    virtual colour emitted(double u, double v, const coordinate& p) const {
        return colour(0, 0, 0);
    }

    virtual bool scatter(
        const ray& r_in, const collision_record& record, colour& attenuation, ray& scattered
    ) const = 0;
};


class lambertian : public material {
public:
    lambertian(const colour& a) : albedo(make_shared<solid_colour>(a)) {}
    lambertian(shared_ptr<texture> a) : albedo(a) {}

    virtual bool scatter(
        const ray& r_in, const collision_record& record, colour& attenuation, ray& scattered
    ) const override {
        auto scatter_direction = record.normal + random_unit_vector();

        // Catch degenerate scatter direction
        if (scatter_direction.near_zero())
            scatter_direction = record.normal;

        scattered = ray(record.p, scatter_direction, r_in.time());
        attenuation = albedo->value(record.u, record.v, record.p);
        return true;
    }

public:
    shared_ptr<texture> albedo;
};


class metal : public material {

public:

    colour albedo;
    double fuzz;

    metal(const colour& a, double f) : albedo(a), fuzz(f < 1 ? f : 1) {}

    virtual bool scatter(
        const ray& r_in, const collision_record& record, colour& attenuation, ray& scattered
    ) const override {
        vec3 reflected = reflect(unit_vector(r_in.direction()), record.normal);
        scattered = ray(record.p, reflected + fuzz * random_in_unit_sphere(), r_in.time());
        attenuation = albedo;
        return (dot(scattered.direction(), record.normal) > 0);
    }

};


class dielectric : public material {

public:

    double ir; 

    dielectric(double index_of_refraction) : ir(index_of_refraction) {}

    virtual bool scatter(
        const ray& r_in, const collision_record& record, colour& attenuation, ray& scattered
    ) const override {

        attenuation = colour(1.0, 1.0, 1.0);
        double refraction_ratio = record.front_face ? (1.0 / ir) : ir;

        vec3 unit_direction = unit_vector(r_in.direction());
        double cos_theta = fmin(dot(-unit_direction, record.normal), 1.0);
        double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

        bool cannot_refract = refraction_ratio * sin_theta > 1.0;
        vec3 direction;

        if (cannot_refract || reflectance(cos_theta, refraction_ratio) > random_double())
            direction = reflect(unit_direction, record.normal);
        else
            direction = refract(unit_direction, record.normal, refraction_ratio);

        scattered = ray(record.p, direction, r_in.time());
        return true;
    }

private:

    // using Schlick approximation for reflectance
    static double reflectance(double cosine, double ref_idx) {
        auto r0 = (1 - ref_idx) / (1 + ref_idx);
        r0 = r0 * r0;
        return r0 + (1 - r0) * pow((1 - cosine), 5);
    }
};


class diffuse_light : public material {

public:
    diffuse_light(shared_ptr<texture> a) : emit(a) {}
    diffuse_light(colour c) : emit(make_shared<solid_colour>(c)) {}

    virtual bool scatter(
        const ray& r_in, const collision_record& record, colour& attenuation, ray& scattered
    ) const override {
        return false;
    }

    virtual colour emitted(double u, double v, const coordinate& p) const override {
        return emit->value(u, v, p);
    }

public:
    shared_ptr<texture> emit;
};


class isotropic : public material {
public:
    isotropic(colour c) : albedo(make_shared<solid_colour>(c)) {}
    isotropic(shared_ptr<texture> a) : albedo(a) {}

    virtual bool scatter(
        const ray& r_in, const collision_record& record, colour& attenuation, ray& scattered
    ) const override {
        scattered = ray(record.p, random_in_unit_sphere(), r_in.time());
        attenuation = albedo->value(record.u, record.v, record.p);
        return true;
    }

public:
    shared_ptr<texture> albedo;
};


#endif