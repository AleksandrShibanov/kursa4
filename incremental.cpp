//
// Created by aleksandr on 10.05.2021.
//

#include "incremental.hpp"

#include <vector>
#include <set>
#include <algorithm>
#include "triangle.hpp"
#include "edge.hpp"


Incremental::Incremental(std::vector<Eigen::Vector2f>& points): points(points) {}

std::pair<Triangle, Triangle> Incremental::createBigTriangle(Eigen::Vector2f& p1_1, Eigen::Vector2f& p2_1, Eigen::Vector2f& p3_1, Eigen::Vector2f& p4_1) {

    // auto mmx = std::minmax_element(points.begin(), points.end(), [](  Eigen::Vector2f& p1,   Eigen::Vector2f& p2) {
    //     return p1.x() < p2.x();
    // });

    // auto mmy = std::minmax_element(points.begin(), points.end(), [](  Eigen::Vector2f& p1,   Eigen::Vector2f& p2) {
    //     return p1.y() < p2.y();
    // });

    // double min_x = mmx.first->x();
    // double max_x = mmx.second->x();
    // double min_y = mmy.first->y();
    // double max_y = mmy.second->y();

    // double dx = max_x - min_x;
    // double dy = max_y - min_y;
    // double deltaMax = std::max(dx, dy);
    // double midX = 0.5 * (min_x + max_x);
    // double midY = 0.5 * (min_y + max_y);

    // Eigen::Vector2f p1(midX - 20 * deltaMax, midY - deltaMax);
    // p1_1 = p1;
    // Eigen::Vector2f p2(midX, midY + 20 * deltaMax);
    // p2_1 = p2;
    // Eigen::Vector2f p3(midX + 20 * deltaMax, midY - deltaMax);
    // p3_1 = p3;

	Eigen::Vector2f min = points[0];
	Eigen::Vector2f max = min;
	for (const Eigen::Vector2f& p : points) 
	{
		min = min.cwiseMin(p);
		max = max.cwiseMax(p);
	}

	max += Eigen::Vector2f::Ones();
	min -= Eigen::Vector2f::Ones();

    // A --- B
    // |    /|
    // |  /  |
    // C --- D
    p1_1 = min;
    p2_1 = max;
    p3_1 = Eigen::Vector2f(min(0), max(1));
    p4_1 = Eigen::Vector2f(max(0), min(1));

    return std::make_pair(Triangle(p1_1,p3_1,p4_1), Triangle(p2_1,p3_1,p4_1));


    // Eigen::Vector2f p1(0.0, 0.0);  // normalized data coordinates from 0 to 1 so i can just hardcore coords of big tr
    // p1_1 = p1;
    // Eigen::Vector2f p2(0.0, 2.0);
    // p2_1 = p2;
    // Eigen::Vector2f p3(2.0, 0.0);
    // p3_1 = p3;
    // return Triangle(p1, p2, p3);
}

#include <iostream>
std::vector<Triangle> Incremental::triangulate() {

    Eigen::Vector2f p1, p2, p3, p4;
    auto [bigTriangle1, bigTriangle2] = createBigTriangle(p1, p2, p3, p4);
    std::vector<Triangle> triangles{bigTriangle1, bigTriangle2};
    size_t i = 0;
    for (const auto &point : points) {
        std::cout << "iteration " << i << std::endl;
        ++i;

        std::vector<Edge> polygon;

        for (auto it = triangles.begin(); it != triangles.end(); ++it) {
            if (it->circumscribedCircleContains(point)) {
                it->MakeBad();
                polygon.push_back(Edge(it->A, it->B, INC_PRECISION));
                polygon.push_back(Edge(it->B, it->C, INC_PRECISION));
                polygon.push_back(Edge(it->C, it->A, INC_PRECISION));
            }
        }

        triangles.erase(std::remove_if(begin(triangles), end(triangles), [](Triangle& t){
            return t.isBad();
        }), end(triangles));


        for (auto edge1 = polygon.begin(); edge1 != polygon.end(); edge1++) {
            for (auto edge2 = std::next(edge1, 1); edge2 != polygon.end(); edge2++) {
                if (*edge1 == *edge2) {
                    edge1->MakeBad();
                    edge2->MakeBad();
                }
            }
        }

        polygon.erase(std::remove_if(begin(polygon), end(polygon), [](Edge& e){
            return e.isBad();
        }), end(polygon));

        for(auto& edge : polygon){
            triangles.push_back(Triangle{edge.p1, edge.p2, point});
        }

        processed.push_back(point);
    }
    triangles.erase(std::remove_if(begin(triangles), end(triangles), [p1, p2, p3, p4](Triangle& t){
        return t.containsPoint(p1) || t.containsPoint(p2) || t.containsPoint(p3) || t.containsPoint(p4);
    }), end(triangles));

    for(auto t : triangles)
    {
        t.edges.push_back(Edge(t.A, t.B, INC_PRECISION));
        t.edges.push_back(Edge(t.B, t.C, INC_PRECISION));
        t.edges.push_back(Edge(t.C, t.A, INC_PRECISION));
    }

    return triangles;
}