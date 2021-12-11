#pragma once

#include <vector>
#include <set>
#include <algorithm>
#include <mutex>
#include <condition_variable>
#include "point.hpp"
#include "triangle.hpp"
#include "edge.hpp"


/**
 * Bowyer Watson Algorithm https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm
 */

struct Incremental {
    std::vector<Point> points;


    explicit Incremental(std::vector<Point>& points);
    Triangle createBigTriangle(Point& p1_1, Point& p2_1, Point& p3_1);
    std::vector<Triangle> triangulate();
    std::vector<Triangle> triangulate(std::vector<Triangle>& triangles);
    std::vector<Triangle> triangulate(Point& p, std::vector<Triangle>& triangles);

    };