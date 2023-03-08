#ifndef texture_h
#define texture_h

#include "utilities_package.h"
#include "perlin.h"

#include <iostream>


class texture {

public:
    virtual colour value(double u, double v, const vec3& p) const = 0;
};


class solid_colour : public texture {

private:
    colour colour_value;

public: 
    solid_colour() {}
    solid_colour(colour c) : colour_value(c) {}

    solid_colour(double red, double green, double blue)
        : solid_colour(colour(red, green, blue)) {}

    virtual colour value(double u, double v, const vec3& p) const override {
        return colour_value;
    }

};


class noise_texture : public texture {
public:
    noise_texture() {}
    noise_texture(double sc) : scale(sc) {}

    virtual colour value(double u, double v, const vec3& p) const override {
        // return color(1,1,1)*0.5*(1 + noise.turb(scale * p));
        // return color(1,1,1)*noise.turb(scale * p);
        return colour(1, 1, 1) * 0.5 * (1 + sin(scale * p.z() + 10 * noise.turb(p)));
    }

public:
    perlin noise;
    double scale;
};



#endif