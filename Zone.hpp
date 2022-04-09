#pragma once

#include <vector>
#include <algorithm>
#include <optional>
#include <unordered_set>

#include "point.hpp"
#include "triangle.hpp"
#include "edge.hpp"
#include "incremental.hpp"


struct Zone
{
    std::set<Point> points;
    std::unordered_set<Edge, MyHash<Edge>> edges;
    std::vector<Triangle> triangles;

    template <class T>
    Point getMaxPointByFunc(T aFunc) const
    {
        return *std::max_element(points.begin(), points.end(), aFunc);
    }

    template <class T>
    Point getMinPointByFunc(T aFunc) const
    {
        return *std::min_element(points.begin(), points.end(), aFunc);
    }

    std::optional<Edge> getBottomEdge(const Zone& aZone)
    {
        if (points.empty() || aZone.points.empty())
            return std::nullopt;

        auto sGetSortedOtherWay = [](std::set<Point>::const_iterator cbegin, std::set<Point>::const_iterator cend)
        { 
            std::vector<Point> sPoints(cbegin, cend);
            std::sort(sPoints.begin(), sPoints.end(), [](const Point& p1, const Point& p2) { return p1.y > p2.y; }); 
            return sPoints; 
        }; 

        const auto& sSortedFromBottomToTop = sGetSortedOtherWay(points.cbegin(), points.cend());
        const auto& sOtherZoneSortedFromBottomToTop = sGetSortedOtherWay(aZone.points.cbegin(), aZone.points.cend());
        
        auto sOwnCandidateIt = sSortedFromBottomToTop.cbegin();
        auto sOtherCandidateIt = sOtherZoneSortedFromBottomToTop.cbegin();

        Edge sBottomEdge;

        while (sOwnCandidateIt != sSortedFromBottomToTop.cend() && sOtherCandidateIt != sOtherZoneSortedFromBottomToTop.cend())
        {
            sBottomEdge = Edge(*sOwnCandidateIt, *sOtherCandidateIt);
            bool sNeedToContinue = false;

            // std::cout << sOwnCandidateIt->x << " " << sOwnCandidateIt->y << std::endl;
            // std::cout << sOtherCandidateIt->x << " " << sOtherCandidateIt->y << std::endl;

            for (const auto& sEdge: edges)
            {
                Point sIntersectionPoint;
                if (sBottomEdge.isIntersects(sEdge, sIntersectionPoint) && sIntersectionPoint != *sOwnCandidateIt)
                {
                    // std::cout << "INTERSECTION LEFT " << sIntersectionPoint.x << " " << sIntersectionPoint.y << std::endl;
                    sOwnCandidateIt++;
                    sNeedToContinue = true;  // нужно вернуться континьюить вайл
                    break;
                }
            }

            if (sNeedToContinue)  // выглядит нелепо, нужно придумать как сделать иначе континью внешнего цикла. Ну не исключение же кидать
                continue;

            for (const auto& sEdge: aZone.edges)
            {
                Point sIntersectionPoint;
                if (sBottomEdge.isIntersects(sEdge, sIntersectionPoint) && sIntersectionPoint != *sOtherCandidateIt)
                {
                    // std::cout << "INTERSECTION RIGHT " << sIntersectionPoint.x << " " << sIntersectionPoint.y << std::endl;
                    sOtherCandidateIt++;
                    sNeedToContinue = true;  // нужно вернуться континьюить вайл
                    break;
                }
            }

            if (sNeedToContinue)  // выглядит нелепо, нужно придумать как сделать иначе континью внешнего цикла. Ну не исключение же кидать
                continue;

            return sBottomEdge;  // нашли самый нижний отрезок соединяющий две зоны и не пересекающий их
        }

        return std::nullopt;
    }

    // слева нам нужно считать /_ угол, а справа _\, поэтому без параметра указывающего какая зона относительно соединения рассматривается увы не обойтись
    std::optional<Point> getBestCandidate(const Edge& aEdge, bool aIsLeftZone)  
    {
        std::vector<Edge> sEdges;
        std::copy_if(edges.begin(), edges.end(), std::back_inserter(sEdges), [aEdge](const Edge& e) { return e.hasCommonPoint(aEdge); });

        if (sEdges.empty())
            return std::nullopt;

        const auto& sCommonPoint = sEdges.begin()->getCommonPoint(aEdge);
        const auto& sNotFromZonePoint = aEdge.getNotCommonPoint(*sEdges.begin());

        std::vector<Point> sPoints;
        std::transform(sEdges.begin(), sEdges.end(), std::back_inserter(sPoints), [aEdge](const Edge& e) { return e.getNotCommonPoint(aEdge); });

        if (sPoints.size() == 1)
            return *sPoints.begin();
 
        std::sort(sPoints.begin(), sPoints.end(), [sCommonPoint, sNotFromZonePoint, aIsLeftZone](const Point& p1, const Point& p2) {
            const auto& sInnerEdge1 = Edge(sCommonPoint, p1);
            const auto& sInnerEdge2 = Edge(sCommonPoint, p2);
            const auto& sOuterEdge = Edge(sNotFromZonePoint, sCommonPoint);
            if (aIsLeftZone)
                return sOuterEdge.clockwiseAngle(sInnerEdge1) > sOuterEdge.clockwiseAngle(sInnerEdge2);
            else
                return sOuterEdge.clockwiseAngle(sInnerEdge1) < sOuterEdge.clockwiseAngle(sInnerEdge2);
        });

        std::erase_if(sPoints, [sCommonPoint, sNotFromZonePoint, aIsLeftZone](const Point& p) {
            const auto& sOuterEdge = Edge(sCommonPoint, sNotFromZonePoint);
            const auto& sInnerEdge = Edge(sCommonPoint, p);
            // const auto& sTr1 = Triangle(sNotFromZonePoint, sCommonPoint, p);
            double sAngle = 0.0;
            if (aIsLeftZone)
                sAngle = sInnerEdge.clockwiseAngle(sOuterEdge);
            else
                sAngle = sOuterEdge.clockwiseAngle(sInnerEdge);
            // std::cout << sAngle << std::endl;
            return sAngle < 0;
        });

        if (sPoints.empty())
            return std::nullopt;

        // for (auto sPoint: sPoints)
        // {
        //     const auto& sTr1 = Triangle(sNotFromZonePoint, sCommonPoint, sPoint);
        //     std::cout << sPoint << std::endl;
        //     std::cout << sTr1.ABC() << std::endl;
        // }

        auto sCurrentBestCandidateIt = sPoints.cbegin();
        for (auto it = std::next(sCurrentBestCandidateIt); it != sPoints.cend(); ++it)
        {
            const auto& sCheckTriangle = Triangle(*sCurrentBestCandidateIt, sNotFromZonePoint, sCommonPoint);
            if (sCheckTriangle.circumscribedCircleContains(*it))
            {
                const auto& sEdgeToDelete = Edge(sCommonPoint, *sCurrentBestCandidateIt);
                edges.erase(sEdgeToDelete);
                std::erase_if(triangles, [sEdgeToDelete](const auto& sTriangle) { return sTriangle.containsEdge(sEdgeToDelete); });
                
                sCurrentBestCandidateIt = it;
            }
            else 
                break;
        }

        return *sCurrentBestCandidateIt;
    }

    void emplace(const Point& aPoint)
    {
        points.insert(aPoint);
    }

    void triangulate()
    {
        std::vector<Point> sPoints(points.cbegin(), points.cend());
        Incremental sInc(sPoints);
        triangles = sInc.triangulate();

        for (const auto& sTriangle: triangles)
        {
            edges.insert(sTriangle.edges.begin(), sTriangle.edges.end());
        }
    }

};