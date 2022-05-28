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

Triangle Incremental::createBigTriangle(Eigen::Vector2f& p1_1, Eigen::Vector2f& p2_1, Eigen::Vector2f& p3_1) {

    auto mmx = std::minmax_element(points.begin(), points.end(), [](  Eigen::Vector2f& p1,   Eigen::Vector2f& p2) {
        return p1.x() < p2.x();
    });

    auto mmy = std::minmax_element(points.begin(), points.end(), [](  Eigen::Vector2f& p1,   Eigen::Vector2f& p2) {
        return p1.y() < p2.y();
    });

    double min_x = mmx.first->x();
    double max_x = mmx.second->x();
    double min_y = mmy.first->y();
    double max_y = mmy.second->y();

    double dx = max_x - min_x;
    double dy = max_y - min_y;
    double deltaMax = std::max(dx, dy);
    double midX = 0.5 * (min_x + max_x);
    double midY = 0.5 * (min_y + max_y);

    Eigen::Vector2f p1(midX - 20 * deltaMax, midY - deltaMax);
    p1_1 = p1;
    Eigen::Vector2f p2(midX, midY + 20 * deltaMax);
    p2_1 = p2;
    Eigen::Vector2f p3(midX + 20 * deltaMax, midY - deltaMax);
    p3_1 = p3;
    return Triangle(p1, p2, p3);
}

void insert(const Eigen::Vector2f& point, std::vector<Triangle>& triangles) {
    std::vector<Edge> polygon;

    for (auto it = triangles.begin(); it != triangles.end(); ++it) {
        if (it->circumscribedCircleContains(point)) {
            it->MakeBad();
            polygon.push_back(Edge(it->A, it->B));
            polygon.push_back(Edge(it->B, it->C));
            polygon.push_back(Edge(it->C, it->A));
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
}


std::vector<Triangle> Incremental::triangulate() {

    Eigen::Vector2f p1, p2, p3;
    Triangle bigTriangle = createBigTriangle(p1, p2, p3);
    std::vector<Triangle> triangles{bigTriangle};

    for (const auto &point : points) {
        std::vector<Edge> polygon;

        for (auto it = triangles.begin(); it != triangles.end(); ++it) {
            if (it->circumscribedCircleContains(point)) {
                it->MakeBad();
                polygon.push_back(Edge(it->A, it->B));
                polygon.push_back(Edge(it->B, it->C));
                polygon.push_back(Edge(it->C, it->A));
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
    }
    triangles.erase(std::remove_if(begin(triangles), end(triangles), [p1, p2, p3](Triangle& t){
        return t.containsPoint(p1) || t.containsPoint(p2) || t.containsPoint(p3);
    }), end(triangles));

    for(auto t : triangles)
    {
        t.edges.push_back(Edge(t.A, t.B));
        t.edges.push_back(Edge(t.B, t.C));
        t.edges.push_back(Edge(t.C, t.A));
    }

    return triangles;
}

std::vector<Triangle> Incremental::triangulate(std::vector<Triangle>& triangles) {

    Eigen::Vector2f p1, p2, p3;
    Triangle bigTriangle = createBigTriangle(p1, p2, p3);
    triangles.push_back(bigTriangle);

    for (auto& p: points) {
        insert(p, triangles);
    }

    triangles.erase(std::remove_if(begin(triangles), end(triangles), [p1, p2, p3](Triangle& t){
        return t.containsPoint(p1) || t.containsPoint(p2) || t.containsPoint(p3);
    }), end(triangles));

    for(auto t : triangles)
    {
        t.edges.clear();
        t.edges.push_back(Edge(t.A, t.B));
        t.edges.push_back(Edge(t.B, t.C));
        t.edges.push_back(Edge(t.C, t.A));
    }

    return triangles;
}

std::vector<Triangle> Incremental::triangulate(Eigen::Vector2f &p, std::vector<Triangle>& triangles) {
    Eigen::Vector2f p1, p2, p3;
    Triangle bigTriangle = createBigTriangle(p1, p2, p3);
    triangles.push_back(bigTriangle);

    insert(p, triangles);

    triangles.erase(std::remove_if(begin(triangles), end(triangles), [p1, p2, p3](Triangle& t){
        return t.containsPoint(p1) || t.containsPoint(p2) || t.containsPoint(p3);
    }), end(triangles));

    for(auto t : triangles)
    {
        t.edges.clear();
        t.edges.push_back(Edge(t.A, t.B));
        t.edges.push_back(Edge(t.B, t.C));
        t.edges.push_back(Edge(t.C, t.A));
    }

    return triangles;
}
