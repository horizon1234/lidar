/**
 * @file Example1dRay.cpp
 * @brief Entry point for the chapter 19 one-dimensional LiDAR ray example.
 */
#include "Example1dRayOutput.hpp"

#include <exception>
#include <iostream>

int main() {
    example_common::enable_ansi_color();
    try {
        example_1d_ray::run_1d_ray_demo();
    } catch (const std::exception& error) {
        std::cerr << "[ERROR] " << error.what() << "\n";
        return 1;
    }
    return 0;
}
