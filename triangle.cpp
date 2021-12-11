#include "triangle.hpp"

#include <cmath>
#include <ostream>
#include <vector>
#include <limits>
#include "point.hpp"
#include "edge.hpp"



    Triangle::Triangle(const Point& A, const Point& B, const Point& C) : A(A), B(B), C(C) {
        edges.clear();
        edges.push_back({A, B});
        edges.push_back({B, C});
        edges.push_back({C, A});
    }


    double Triangle::area() const {
        return 0.5 * std::fabs((A.x - C.x) * (B.y - A.y) - (A.x - B.x) * (C.y - A.y));
    }

    bool Triangle::hasCommonEdge(const Triangle &triangle) const {
        for (auto edge1 : edges) {
            for (auto edge2 : triangle.edges) {
                if (edge1 == edge2) {
                    return true;
                }
            }
        }
        return false;
    }

    bool Triangle::hasCommonPoint(const Triangle &triangle) const {
        for (auto edge1 : edges) {
            for (auto edge2 : triangle.edges) {
                if (edge1.p1 == edge2.p1 || edge1.p2 == edge2.p2) {
                    return true;
                }
            }
        }
        return false;
    }

    double Triangle::ABC() const {
        Edge AB{A, B};
        Edge BC{B, C};
        return AB.degree(BC);
    }

    double Triangle::BCA() const {
        Edge BC{B, C};
        Edge CA{C, A};
        return BC.degree(CA);
    }

    double Triangle::CAB() const {
        Edge CA{C, A};
        Edge AB{A, B};
        return AB.degree(CA);
    }

    double Triangle::max_alpha() const {
        return std::max(CAB(), std::max(ABC(), BCA()));
    }

    bool Triangle::isValidTriangulation(const Triangle& triangle) const {
        if (*this != triangle && hasCommonEdge(triangle)) {
            double sum = max_alpha() + triangle.max_alpha();
            return sum < 180;
        } else {
            return false;
        }
    }

    void Triangle::MakeBad() {
        bad = true;
    }

    void Triangle::MakeGood() {
        bad = false;
    }

    bool Triangle::isBad() {
        return bad;
    }

    Point Triangle::getPoint(const Edge& edge) {
        Point a = edge.p1;
        Point b = edge.p2;

        if (A == a && B == b) {
            return C;
        }
        if (B == a && A == b) {
            return C;
        }
        if (B == a && C == b) {
            return A;
        }
        if (C == a && B == b) {
            return A;
        }
        if (C == a && A == b) {
            return B;
        }
        if (A == a && C == b) {
            return B;
        }
    }

    bool Triangle::containsPoint(const Point& v) const {
        return v == A || v == B || v == C;
    }

    bool Triangle::containsEdge(const Edge& edge) const {
        return edges[0] == edge || edges[1] == edge || edges[2] == edge;
    }

    void Triangle::flip(Triangle& triangle) {
        for (auto edge1 : edges) {
            for (auto edge2 : triangle.edges) {
                if (edge1 == edge2) {
                    Point a = getPoint(edge1);
                    Point tr_a = triangle.getPoint(edge2);

                    *this = Triangle(a, edge1.p1, tr_a);
                    triangle = Triangle(tr_a, edge1.p2, a);
                }
            }
        }
    }

    bool Triangle::circumscribedCircleContains(const Point& D) const {
        const double ab = this->A.norm() * this->A.norm();
        const double cd = this->B.norm() * this->B.norm();
        const double ef = this->C.norm() * this->C.norm();

        const double ax = this->A.x;
        const double ay = this->A.y;
        const double bx = this->B.x;
        const double by = this->B.y;
        const double cx = this->C.x;
        const double cy = this->C.y;

        const double circumX = (ab * (cy - by) + cd * (ay - cy) + ef * (by - ay)) /
                               (ax * (cy - by) + bx * (ay - cy) + cx * (by - ay));

        const double circumY = (ab * (cx - bx) + cd * (ax - cx) + ef * (bx - ax)) /
                               (ay * (cx - bx) + by * (ax - cx) + cy * (bx - ax));

        const Point circum(circumX / 2, circumY / 2);
        return D.distance(circum) * D.distance(circum) <= A.distance(circum) * A.distance(circum);
//        double ab = (A.x * A.x) + (A.y * A.y);
//        double cd = (B.x * B.x) + (B.y * B.y);
//        double ef = (C.x * C.x) + (C.y * C.y);
//
//        double circum_x = (ab * (C.y - B.y) + cd * (A.y - C.y) + ef * (B.y - A.y)) / (A.x * (C.y - B.y) + B.x * (A.y - C.y) + C.x * (B.y - A.y)) / 2.f;
//        double circum_y = (ab * (C.x - B.x) + cd * (A.x - C.x) + ef * (B.x - A.x)) / (A.y * (C.x - B.x) + B.y * (A.x - C.x) + C.y * (B.x - A.x)) / 2.f;
//        double circum_radius = std::sqrt(((A.x - circum_x) * (A.x - circum_x)) + ((A.y - circum_y) * (A.y - circum_y)));
//
//        double dist = std::sqrt(((D.x - circum_x) * (D.x - circum_x)) + ((D.y - circum_y) * (D.y - circum_y)));
//        return dist <= circum_radius;

//        return ((A.x - D.x) * (B.y - D.y) * (std::pow(C.x - D.x,2) + std::pow(C.y - D.y,2)) +
//               (B.x - D.x) * (C.y - D.y) * (std::pow(A.x - D.x,2) + std::pow(A.y - D.y,2)) +
//               (C.x - D.x) * (A.y - D.y) * (std::pow(B.x - D.x,2) + std::pow(B.y - D.y,2)) -
//               (C.x - D.x) * (B.y - D.y) * (std::pow(A.x - D.x,2) + std::pow(A.y - D.y,2)) -
//               (A.x - D.x) * (C.y - D.y) * (std::pow(B.x - D.x,2) + std::pow(B.y - D.y,2)) -
//               (B.x - D.x) * (A.y - D.y) * (std::pow(C.x - D.x,2) + std::pow(C.y - D.y,2))) > 0;
    }

    bool Triangle::operator ==(const Triangle& rhs) const {
        return (A == rhs.A && B == rhs.B && C == rhs.C) ||
               (A == rhs.B && B == rhs.C && C == rhs.A) ||
               (A == rhs.C && B == rhs.A && C == rhs.B);
    }

    bool Triangle::operator !=(const Triangle& rhs) const {
        return !(rhs == *this);
    }

    bool Triangle::operator <(const Triangle& rhs) const {
        return area() < rhs.area();
    }

    bool Triangle::operator >(const Triangle& rhs) const {
        return rhs.area() < this->area();
    }
