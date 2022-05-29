#pragma once

#include <vector>
#include <algorithm>
#include <optional>
#include <unordered_set>
#include <set>
#include <deque>
#include <iostream>


#include "triangle.hpp"
#include "edge.hpp"
#include "incremental.hpp"
#include "utils.hpp"

namespace
{
    // The z-value of the cross product of segments 
// (a, b) and (a, c). Positive means c is ccw
// from (a, b), negative cw. Zero means its collinear.
float ccw(const Eigen::Vector2f& a, const Eigen::Vector2f& b, const Eigen::Vector2f& c) {
	return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
}

// Returns true if a is lexicographically before b.
bool isLeftOf(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
	return (a.x() < b.x() || (a.x() == b.x() && a.y() < b.y()));
}

// Used to sort points in ccw order about a pivot.
struct ccwSorter {
	const Eigen::Vector2f& pivot;

	ccwSorter(const Eigen::Vector2f& inPivot) : pivot(inPivot) { }

	bool operator()(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
		return ccw(pivot, a, b) < 0;
	}
};

// bool isGreaterByAlpha(const Edge& aFrontEdge, const Eigen::Vector2f& aPoint1, const Eigen::Vector2f& aPoint2)  // p1 > p2
// {
//     Edge e11(aFrontEdge.p1, aPoint1);
//     Edge e21(aFrontEdge.p2, aPoint1);

//     Edge e12(aFrontEdge.p1, aPoint2);
//     Edge e22(aFrontEdge.p2, aPoint2);

//     double sCrossProduct1 = aFrontEdge.cross(e11);
//     double sCrossProduct2 = aFrontEdge.cross(e12);

//     return sCrossProduct1/(e11.length()*e11.length()+e21.length()*e21.length()+aFrontEdge.length()*aFrontEdge.length()) 
//             > 
//             sCrossProduct2/(e12.length()*e12.length()+e22.length()*e22.length()+aFrontEdge.length()*aFrontEdge.length());
// }

bool isGreaterByLen(const Edge& aFrontEdge, const Eigen::Vector2f& aPoint1, const Eigen::Vector2f& aPoint2)  // p1 > p2
{
    Edge e11(aFrontEdge.p1, aPoint1, ADF_PRECISION);
    Edge e21(aFrontEdge.p2, aPoint1, ADF_PRECISION);

    Edge e12(aFrontEdge.p1, aPoint2, ADF_PRECISION);
    Edge e22(aFrontEdge.p2, aPoint2, ADF_PRECISION);

    return e11.length()*e11.length()+e21.length()*e21.length() > e12.length()*e12.length()+e22.length()*e22.length();
}
}

struct ADF
{
    private:
    std::set<Eigen::Vector2f, vecCompare> points;
    std::vector<Edge> edges;
    std::vector<Triangle> triangles;
    std::deque<Edge> front;

    public:
    ADF(std::set<Eigen::Vector2f, vecCompare> aPoints): points(aPoints) 
    {}

    void fillFront()
    {
        auto sPoints = convexHullPoints();
        auto sPrev = sPoints.begin();
        for (auto it = std::next(sPoints.begin()); it != sPoints.end(); ++it)
        {
            front.emplace_back(*sPrev, *it, ADF_PRECISION);
            sPrev = it;
        }

        // 2 points in sPoints we do not consider, this means something went bad, so just push without check >2
        front.emplace_back(*std::prev(sPoints.end()), *sPoints.begin(), ADF_PRECISION);
    }

    void splitFront(double aSplitSize)
    {
        auto sInitFrontSize = front.size();
        for (size_t i = 0; i < sInitFrontSize; ++i)
        {
            auto sEdge = front.front();
            front.pop_front();

            auto sPointFrom = sEdge.p1;
            auto sPointTo = sEdge.p2;

            int iterations = sEdge.length()/aSplitSize;
            Eigen::Vector2f startPoint(0, 0);
            Eigen::Vector2f endPoint(0, 0);
            for (int j = 0; j < iterations; ++j)
            {
                double deltaX = (sPointTo.x() - sPointFrom.x())/(iterations);
                double deltaY = (sPointTo.y() - sPointFrom.y())/(iterations);
                startPoint = Eigen::Vector2f(sPointFrom.x()+deltaX*j, sPointFrom.y()+deltaY*j);
                endPoint = Eigen::Vector2f(sPointFrom.x()+deltaX*(j+1), sPointFrom.y()+deltaY*(j+1));
                front.emplace_back(startPoint, endPoint, ADF_PRECISION);
            }
            
            if (iterations == 0)
            {
                front.emplace_back(sPointFrom, sPointTo, ADF_PRECISION);
            }
            

        }
    }

    std::vector<Triangle> triangulate()
    {
        for (size_t i = 0; i < MAX_ITERATIONS && !front.empty(); ++i)  // for debug
        // for (int i = 0; !front.empty(); ++i)
        {
            std::cout << "iteration " << i << std::endl;
            auto sFrontEdge = front.front();
            front.pop_front();

            auto sPoint = generateFrontEdgePoint(sFrontEdge);
            Edge e1(sFrontEdge.p1, sPoint, ADF_PRECISION);
            Edge e2(sPoint, sFrontEdge.p2, ADF_PRECISION);

            auto sPoints = getFrontPoints();
            std::erase_if(sPoints, [sFrontEdge](const Eigen::Vector2f& p) { 
                return ccw(sFrontEdge.p1, sFrontEdge.p2, p) > 0 || 
                       p.isApprox(sFrontEdge.p1, ADF_PRECISION) || 
                       p.isApprox(sFrontEdge.p2, ADF_PRECISION); 
            });
            auto lam = [&sFrontEdge](const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) 
            {
                return isGreaterByLen(sFrontEdge, p2, p1); 
            };
            std::set<Eigen::Vector2f, decltype(lam)> sFrontPoints(sPoints.begin(), 
                                        sPoints.end(), 
                                        lam);
            for (auto it = sFrontPoints.cbegin(); it != sFrontPoints.end() && (isIntersectsTriangulation(e1) || isIntersectsTriangulation(e2)); ++it)
            {
                e1 = Edge(sFrontEdge.p1, *it, ADF_PRECISION);
                e2 = Edge(*it, sFrontEdge.p2, ADF_PRECISION);
            }

            pushEdges(sFrontEdge, e1, e2);
        }

        return triangles;
        // std::vector<Edge> buf(front.begin(), front.end());
        // return std::make_pair(buf, edges);
    }

    std::vector<Edge> getFront() const
    {
        return std::vector<Edge>(front.begin(), front.end());
    }

    std::vector<Edge> getEdges() const
    {
        return edges;
    }

    private:
    // graham
    std::vector<Eigen::Vector2f> convexHullPoints()
    {
        std::vector<Eigen::Vector2f> sPoints(points.begin(), points.end());
        // Put our leftmost point at index 0
        std::swap(*sPoints.begin(), *std::min_element(sPoints.begin(), sPoints.end(), isLeftOf));

        // Sort the rest of the points in counter-clockwise order
        // from our leftmost point.
        std::sort(sPoints.begin() + 1, sPoints.end(), ccwSorter(sPoints[0]));
        
        // Add our first three points to the hull.
        std::vector<Eigen::Vector2f> hull;
        auto it = sPoints.begin();
        hull.push_back(*it++);
        hull.push_back(*it++);
        hull.push_back(*it++);
        
        while (it != sPoints.end()) {
            // Pop off any points that make a convex angle with *it
            while (ccw(*(hull.rbegin() + 1), *(hull.rbegin()), *it) >= 0) {
                hull.pop_back();
            }
            hull.push_back(*it++);
        }

        return hull;
    }

    bool isIntersectsTriangulation(const Edge& aEdge) const
    {
        for (const auto& sFrontEdge: front)
        {
            if (aEdge == sFrontEdge)
            {
                continue;
            }
            const auto& sIntersectionPoint = aEdge.isIntersects(sFrontEdge);
            if (sIntersectionPoint.has_value() && 
               !sFrontEdge.p1.isApprox(sIntersectionPoint.value(), ADF_PRECISION) && 
               !sFrontEdge.p2.isApprox(sIntersectionPoint.value(), ADF_PRECISION))
            {
                // std::cout << "front intersect" << std::endl;
                // std::cout << sIntersectionPoint.value() << std::endl;
                return true;
            }
        }
        for (const auto& sEdge: edges)
        {
            if (aEdge == sEdge)
            {
                continue;
            }
            const auto& sIntersectionPoint = aEdge.isIntersects(sEdge);
            if (sIntersectionPoint.has_value() && 
                !sEdge.p1.isApprox(sIntersectionPoint.value(), ADF_PRECISION) && 
                !sEdge.p2.isApprox(sIntersectionPoint.value(), ADF_PRECISION))
            {
                // std::cout << "edges intersect" << std::endl;
                // std::cout << sEdge << std::endl;
                // std::cout << sIntersectionPoint.value() << std::endl;
                return true;
            }
        }
        return false;
    }

    Eigen::Vector2f generateFrontEdgePoint(const Edge& aFrontEdge) const
    {
        return getCandidateFrontEdgePoint(aFrontEdge);
    }

    Eigen::Vector2f getCandidateFrontEdgePoint(const Edge& aFrontEdge) const
    {
        std::pair<double, double> sVector(aFrontEdge.p2.x()-aFrontEdge.p1.x(), aFrontEdge.p2.y()-aFrontEdge.p1.y());
        std::pair<double, double> sPerpendicularVector(sVector.second, -1.0*sVector.first);
        Eigen::Vector2f sMidPoint((aFrontEdge.p2.x()-aFrontEdge.p1.x())/2, (aFrontEdge.p2.y()-aFrontEdge.p1.y())/2);
        return Eigen::Vector2f(sPerpendicularVector.first+aFrontEdge.p1.x()+sMidPoint.x(), sPerpendicularVector.second+aFrontEdge.p1.y()+sMidPoint.y());
    }

    std::set<Eigen::Vector2f, vecCompare> getFrontPoints()
    {
        std::set<Eigen::Vector2f, vecCompare> sResult;
        for (const auto& sFrontEdge: front)
        {
            sResult.insert(sFrontEdge.p1);
            sResult.insert(sFrontEdge.p2);
        }

        return sResult;
    }

    void pushEdges(const Edge& aFrontEdge, const Edge& aNewEdge1, const Edge& aNewEdge2)
    {
        auto sFindE1It = std::find(front.begin(), front.end(), aNewEdge1);
        if (sFindE1It != front.end())
        {
            front.erase(sFindE1It);
            edges.push_back(aNewEdge1);
        }
        else
        {
            front.push_back(aNewEdge1);
        }

        auto sFindE2It = std::find(front.begin(), front.end(), aNewEdge2);
        if (sFindE2It != front.end())
        {
            front.erase(sFindE2It);
            edges.push_back(aNewEdge2);
        }
        else
        {
            front.push_back(aNewEdge2);
        }
        edges.push_back(aFrontEdge);
        triangles.emplace_back(aNewEdge1.p1, aNewEdge1.p2, aNewEdge2.p2);
    }

};