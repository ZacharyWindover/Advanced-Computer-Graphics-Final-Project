#include "utilities_package.h"

#include "aabb.h"
#include "aarect.h"
#include "colour.h"
#include "collision.h"
#include "collision_list.h"
#include "eye.h"
#include "material.h"
#include "sphere.h"
#include "texture.h"
#include "vec3.h"

#include <iostream>
#include <cstdlib>
#include <thread>
#include <vector>
#include <functional>
#include <chrono>
#include <omp.h>
#include <time.h>

using namespace std::chrono;

int image_width;
int samples_per_pixel;
int max_depth;
int image_height;

int global_x, global_k, global_z;
int light_x0, light_x1, light_y0, light_y1, light_k;

collision_list world;
colour background;
//colour pixel_colour(0, 0, 0);
eye camera;

//std::vector<int> rgb_file_values;
std::vector<std::vector<int>> rgb_values (1, std::vector<int>(3, 0));


colour ray_colour(const ray& r, const colour& background, const collision& world, int depth) {
    
    collision_record record;

    // If reached max depth, won't branch out and get more ray light
    if (depth <= 0)
        return colour(0, 0, 0);

    // If ray hits nothing, return background colour;
    if (!world.hit(r, 0.001, infinity, record)) 
        return background;

    ray scattered;
    colour attenuation;
    colour emitted = record.mat_ptr->emitted(record.u, record.v, record.p);

    if (!record.mat_ptr->scatter(r, record, attenuation, scattered))
        return emitted;

    return emitted + attenuation * ray_colour(scattered, background, world, depth - 1);

}


colour ray_colour_path_trace(ray& r, const colour& background, const collision& world, int depth) {

    collision_record record;



    // If reached max depth, won't branch out and get more ray light
    if (depth <= 0)
        return colour(0, 0, 0);

    // If ray hits nothing, return background colour;
    if (!world.hit(r, 0.001, infinity, record))
        return background;

    ray scattered;
    colour attenuation;
    colour emitted = record.mat_ptr->emitted(record.u, record.v, record.p);

    // randomly generate point within light source
    double x_0 = (random_double(light_x0, light_x1)) * 2 - 2;
    double z_0 = (random_double(light_y0, light_y1)) * 2 - 2;

    // randomly decide if shadow
    bool ray_shadow = random_bool();

    if (ray_shadow == false)
        r.set_direction(record.p - vec3(x_0, light_k, z_0));

    if (!record.mat_ptr->scatter(r, record, attenuation, scattered))
        return emitted;

    return emitted + attenuation * ray_colour(scattered, background, world, depth - 1);

}


collision_list preset_scene() {

    collision_list objects;

    auto red = make_shared<lambertian>(colour(.65, .05, .05));
    auto white = make_shared<lambertian>(colour(.73, .73, .73));
    auto green = make_shared<lambertian>(colour(.12, .45, .15));
    auto blue = make_shared<lambertian>(colour(0.196, 0.196, 0.658));
    auto light = make_shared<diffuse_light>(colour(25, 25, 25));
    auto metal_material = make_shared<metal>(colour(0.7, 0.6, 0.5), 0.0);

    light_x0 = -1;
    light_x1 =  1;
    light_y0 = -1;
    light_y1 =  1;
    global_k = light_k = 5;

    // make the light source
    objects.add(make_shared<xz_rect>(light_x0, light_x1, light_y0, light_y1, light_k, light));

    // make the ground plane
    objects.add(make_shared<xz_rect>(-555, 555, -555, 555, 0, green));

    // add three spheres
    objects.add(make_shared<sphere>(coordinate(0, 1, -1), 1.0, metal_material));

    objects.add(make_shared<sphere>(coordinate(2, 0.75, -2), 0.75, red));
    objects.add(make_shared<sphere>(coordinate(3, 0.75, 2), 0.75, blue));

    return objects;

}

/*
void base_ray_trace(int i, int j) {

    auto u = (i + random_double()) / (image_width - 1);
    auto v = (j + random_double()) / (image_height - 1);
    ray r = camera.get_ray(u, v);
    pixel_colour += ray_colour(r, background, world, max_depth); 

}
*/


int main(int argc, char** argv) {

    // recording starting point
    auto start = high_resolution_clock::now();

    // obtain values from cmd line inputs
    image_width = atoi(argv[1]);
    samples_per_pixel = atoi(argv[2]);
    max_depth = atoi(argv[3]);

    int trace_type = atoi(argv[4]);

    // Image
    auto aspect_ratio = 16.0 / 9.0;
    //image_width = 1280;
    image_height = static_cast<int>(image_width / aspect_ratio);
    //samples_per_pixel = 16384;
    //max_depth = 16;
    
    // declare the vector to be used for storing the threads
    //std::vector<std::thread> threads;

    // set size of rgb_values vector
    int size = image_height * image_width;

    rgb_values.resize(size);

    for (int x = 0; x < size; x++)
        rgb_values[x].resize(3);
    
    //std::cerr << "point 7" << std::endl;

    // World
    world = preset_scene();
    coordinate look_from = coordinate(13, 2, 3);
    coordinate look_at = coordinate(0, 0, 0);
    auto v_fov = 40.0;

    // choose either blue or black
    //background = colour(0, 0, 0);
    background = colour(0.0823, 0.6745, 0.9294);
    
    // Eye
    const vec3 v_up(0, 1, 0);
    const auto distance_to_focus = 10.0;
    auto aperture = 0.0;

    eye camera(look_from, look_at, v_up, v_fov, aspect_ratio, aperture, distance_to_focus, 0.0, 1.0);

    // Render
    enum render_type {ray_trace, distributed_ray_trace, path_trace, omp_ray_trace, omp_path_trace};
    render_type render = omp_ray_trace;

    // change the render type based on the input
    if (trace_type == 1) render = path_trace;
    else if (trace_type == 2) render = ray_trace;
    else if (trace_type == 3) render = omp_path_trace;
    else render = omp_ray_trace;

    if (render == path_trace) {

        global_x = (light_x1 - light_x0) / 2;
        global_z = (light_y1 - light_y0) / 2;

        std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        for (int j = image_height - 1; j >= 0; j--) {

            std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;

            for (int i = 0; i < image_width; i++) {

                colour pixel_colour(0, 0, 0);

                for (int s = 0; s < samples_per_pixel; s++) {

                    auto u = (i + random_double()) / (image_width - 1);
                    auto v = (j + random_double()) / (image_height - 1);
                    ray r = camera.get_ray(u, v);
                    pixel_colour += ray_colour_path_trace(r, background, world, max_depth);

                }

                write_colour(std::cout, pixel_colour, samples_per_pixel);

            }

        }
        

    } 
    
    else if (render == distributed_ray_trace) {

        // create array grid for points on light
        const int num_squares = 4; // num_squares = num_points - 1
        const int num_points = num_squares + 1;
        const int light_grid_size = num_points * num_points;

        double light_points[num_points][num_points][2];

        //std::cerr << "num_squares: " << num_squares << "\n";
        //std::cerr << "num_points: " << num_points << "\n";
        //std::cerr << "light_grid_size: " << light_grid_size << "\n";

        double x_light_length = light_x1 - light_x0;
        double y_light_length = light_y1 - light_y0;
        double x_light_distribution = x_light_length / num_squares;
        double y_light_distribution = y_light_length / num_squares;

        double x_index_value = light_x0;
        double y_index_value = light_y0;

        for (int x = 0; x < num_points; x++) {

            for (int y = 0; y < num_points; y++) {

                light_points[x][y][0] = x_index_value;
                light_points[x][y][1] = y_index_value;
                //light_points[x][y][2] = light_k;

                //std::cerr << "light_points[" << x << "][" << y << "][0]: " << light_points[x][y][0] << "\n";
                //std::cerr << "light_points[" << x << "][" << y << "][1]: " << light_points[x][y][1] << "\n";

                x_index_value += x_light_distribution;

            }

            x_index_value = light_x0;
            y_index_value += y_light_distribution;

        }

        // do the render stuff

        std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        for (int j = image_height - 1; j >= 0; --j) {

            std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;

            for (int i = 0; i < image_width; ++i) {

                colour pixel_colour(0, 0, 0);


                for (int gpx = 0; gpx < num_points; gpx++) {
                        
                    for (int gpy = 0; gpy < num_points; gpy++) {

                        double u = light_points[gpx][gpy][0] * -1;
                        double v = light_points[gpx][gpy][1] * -1;
                        ray r = camera.get_ray(u, v);
                        pixel_colour += ray_colour(r, background, world, max_depth);

                    }

                }

                /*
                for (int s = 0; s < samples_per_pixel / 2; s++) {
                        
                    // randomly generate point within light source
                    double u = (random_double(light_x0, light_x1)) * 2 - 2;
                    double v = (random_double(light_y0, light_y1)) * 2 - 2;
                    ray r = camera.get_ray(u, v);
                    pixel_colour += ray_colour(r, background, world, max_depth);

                }
                */

                for (int s = 0; s < (samples_per_pixel / 2); ++s) {

                    auto u = (i + random_double()) / (image_width - 1);
                    auto v = (j + random_double()) / (image_height - 1);
                    ray r = camera.get_ray(u, v);
                    pixel_colour += ray_colour(r, background, world, max_depth);

                }

                write_colour(std::cout, pixel_colour, samples_per_pixel);

            }

        }



    } 

    else if (render == omp_path_trace) {

    global_x = (light_x1 - light_x0) / 2;
    global_z = (light_y1 - light_y0) / 2;

    std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

    for (int j = image_height - 1; j >= 0; j--) {

        std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;

        for (int i = 0; i < image_width; i++) {

            colour pixel_colour(0, 0, 0);

            #pragma omp parallel for schedule(dynamic, 4)
            for (int s = 0; s < samples_per_pixel; s++) {

                auto u = (i + random_double()) / (image_width - 1);
                auto v = (j + random_double()) / (image_height - 1);
                ray r = camera.get_ray(u, v);
                pixel_colour += ray_colour_path_trace(r, background, world, max_depth);

            }

            write_colour(std::cout, pixel_colour, samples_per_pixel);

        }

    }

    }

    else if (render == omp_ray_trace) {

        // ppm file header
        std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        for (int j = image_height - 1; j >= 0; --j) {

            std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;

            for (int i = 0; i < image_width; ++i) {

                colour pixel_colour(0, 0, 0);

                #pragma omp parallel for schedule(dynamic, 4)
                for (int s = 0; s < samples_per_pixel; ++s) {

                    auto u = (i + random_double()) / (image_width - 1);
                    auto v = (j + random_double()) / (image_height - 1);
                    ray r = camera.get_ray(u, v);
                    pixel_colour += ray_colour(r, background, world, max_depth);

                }

                write_colour(std::cout, pixel_colour, samples_per_pixel);

            }

        }

    }

    else {     // standard ray tracing       

        // ppm file header
        std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        for (int j = image_height - 1; j >= 0; --j) {

            std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;

            for (int i = 0; i < image_width; ++i) {

                colour pixel_colour(0, 0, 0);
                
                for (int s = 0; s < samples_per_pixel; ++s) {

                    auto u = (i + random_double()) / (image_width - 1);
                    auto v = (j + random_double()) / (image_height - 1);
                    ray r = camera.get_ray(u, v);
                    pixel_colour += ray_colour(r, background, world, max_depth);

                }

                write_colour(std::cout, pixel_colour, samples_per_pixel);

            }

        }

    }

    std::cerr << "\nDone.\n";

    // get the end time of the program
    auto stop = high_resolution_clock::now();

    // calculate time taken
    auto runtime = duration_cast<microseconds>(stop - start);

    // output time
    std::cerr << runtime.count() << " microseconds " << std::endl;

}