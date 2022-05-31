#include "triangle.hpp"

#include <cmath>
#include "edge.hpp"

Triangle::Triangle(const Eigen::Vector2f& A, const Eigen::Vector2f& B, const Eigen::Vector2f& C) : A(A), B(B), C(C) {
    edges.push_back({A, B, INC_PRECISION});
    edges.push_back({B, C, INC_PRECISION});
    edges.push_back({C, A, INC_PRECISION});
}

bool Triangle::hasCommonEdge(const Triangle &triangle) const 
{
    for (const auto& edge1 : edges)
        for (const auto& edge2 : triangle.edges)
            if (edge1 == edge2)
                return true;
    return false;
}

bool Triangle::hasCommonPoint(const Triangle &triangle) const 
{
    for (const auto& edge1 : edges)
        for (const auto& edge2 : triangle.edges)
            if (edge1.p1 == edge2.p1 || edge1.p2 == edge2.p2)
                return true;
    return false;
}

void Triangle::MakeBad() 
{
    bad = true;
}

void Triangle::MakeGood() 
{
    bad = false;
}

bool Triangle::isBad() 
{
    return bad;
}

// Eigen::Vector2f Triangle::getPoint(const Edge& edge) 
// {
//     const auto& a = edge.p1;
//     const auto& b = edge.p2;

//     if (A == a && B == b)
//         return C;
//     if (B == a && A == b)
//         return C;
//     if (B == a && C == b)
//         return A;
//     if (C == a && B == b)
//         return A;
//     if (C == a && A == b)
//         return B;
//     if (A == a && C == b)
//         return B;
// }

bool Triangle::containsPoint(const Eigen::Vector2f& v) const 
{
    return v.isApprox(A, INC_PRECISION) || v.isApprox(B, INC_PRECISION)  || v.isApprox(C, INC_PRECISION);
}

bool Triangle::containsEdge(const Edge& edge) const 
{
    return edges[0] == edge || edges[1] == edge || edges[2] == edge;
}

bool Triangle::circumscribedCircleContains(const Eigen::Vector2f& D) const 
{
    const double ab = hypot(this->A.x(), this->A.y()) * hypot(this->A.x(), this->A.y());
    const double cd = hypot(this->B.x(), this->B.y()) * hypot(this->B.x(), this->B.y());
    const double ef = hypot(this->C.x(), this->C.y()) * hypot(this->C.x(), this->C.y());

    const double ax = this->A.x();
    const double ay = this->A.y();
    const double bx = this->B.x();
    const double by = this->B.y();
    const double cx = this->C.x();
    const double cy = this->C.y();

    const double circumX = (ab * (cy - by) + cd * (ay - cy) + ef * (by - ay)) /
                            (ax * (cy - by) + bx * (ay - cy) + cx * (by - ay));

    const double circumY = (ab * (cx - bx) + cd * (ax - cx) + ef * (bx - ax)) /
                            (ay * (cx - bx) + by * (ax - cx) + cy * (bx - ax));

    const Eigen::Vector2f circum(circumX / 2, circumY / 2);
    return hypot(D.x() - circum.x(), D.y() - circum.y()) * hypot(D.x() - circum.x(), D.y() - circum.y())
           <= 
           hypot(A.x() - circum.x(), A.y() - circum.y()) * hypot(A.x() - circum.x(), A.y() - circum.y());
}

#include <iostream>
double Triangle::getDelaunayQualityValue() const
{
    Edge AB(B, A, INC_PRECISION);
    Edge BC(C, B, INC_PRECISION);
    Edge CA(A, C, INC_PRECISION);

    // length of sides be a, b, c
    auto a = AB.length();
    auto b = BC.length();
    auto c = CA.length();

    auto a2 = a*a;
    auto b2 = b*b;
    auto c2 = c*c;
 
    // From Cosine law
    auto angleABC = acos((b2 + c2 - a2)/(2*b*c));
    auto angleBCA = acos((a2 + c2 - b2)/(2*a*c));
    auto angleCAB = acos((a2 + b2 - c2)/(2*a*b));

    // std::cout << angleABC + angleBCA + angleCAB << std::endl;
    // assert(angleABC + angleBCA + angleCAB > 3.14 && angleABC + angleBCA + angleCAB < 3.15);

    auto min = std::min(std::min(angleABC, angleBCA), angleCAB);
    auto max = std::max(std::max(angleABC, angleBCA), angleCAB);

    return min/max;
}

// bool Triangle::operator ==(const Triangle& rhs) const 
// {
//     return (A == rhs.A && B == rhs.B && C == rhs.C) ||
//             (A == rhs.B && B == rhs.C && C == rhs.A) ||
//             (A == rhs.C && B == rhs.A && C == rhs.B);
// }

// bool Triangle::operator !=(const Triangle& rhs) const 
// {
//     return !(rhs == *this);
// }
