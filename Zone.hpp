#pragma once

#include <vector>
#include <algorithm>
#include <optional>
#include <unordered_set>
#include <set>
#include <Eigen/Dense>

#include "triangle.hpp"
#include "edge.hpp"
#include "incremental.hpp"
#include "utils.hpp"
#include "adf.hpp"


struct Zone
{
    std::set<Eigen::Vector2f, vecCompare> points;
    std::unordered_set<Edge, MyHash<Edge>> edges;
    std::vector<Triangle> triangles;

    void emplace(const Eigen::Vector2f& aPoint)
    {
        points.insert(aPoint);
    }

    void emplace(const Edge& aEdge)
    {
        emplace(aEdge.p1);
        emplace(aEdge.p2);

        edges.insert(aEdge);
    }

    void emplace(const Triangle& aTriangle)
    {
        for (const auto& sEdge: aTriangle.edges)
        {
            emplace(sEdge);
        }

        triangles.emplace_back(aTriangle);
    }

    bool empty() const
    {
        return points.empty();
    }

    void clear()
    {
        triangles.clear();
        edges.clear();
        points.clear();
    }

    auto triangulate()
    {
        ADF sADF(points);
        sADF.fillFront();
        sADF.splitFront(SPLIT);
        triangles = sADF.triangulate();

        // std::set<Eigen::Vector2f, vecCompare> sPoints;
        // for (const auto& tr: triangles)
        // {
        //     sPoints.insert(tr.A);
        //     sPoints.insert(tr.B);
        //     sPoints.insert(tr.C);
        // }
        // std::vector<Eigen::Vector2f> sBuf(sPoints.begin(), sPoints.end());
        // Incremental sInc(sBuf);
        // triangles = sInc.triangulate();

        for (const auto& sTriangle: triangles)
        {
            edges.insert(sTriangle.edges.begin(), sTriangle.edges.end());
        }
    }

    Zone operator +=(Zone& aZone)
    {
        return merge(aZone);
    }

    Zone operator |=(Zone& aZone)
    {
        return merge(aZone);
    }

    Zone merge(Zone& aZone)
    {
        assert(isLeft(aZone));
        return mergeImpl(aZone);
    }

    private:

    Zone mergeImpl(Zone& aZone)  // здесь this должен быть левее чем переданная зона
    {
        Zone sMergeZone;

        auto sLR_Edge = getBottomEdge(aZone);
        auto sLeftBestCandidate = getBestCandidate(sLR_Edge, true);
        auto sRightBestCandidate = aZone.getBestCandidate(sLR_Edge, false);

        for (size_t i = 0; i < MAX_MERGE_ITERATIONS && (sLeftBestCandidate.has_value() || sRightBestCandidate.has_value()); ++i)
        {
            const auto& [sLeftPointOfLR_Edge, sRightPointOfLR_Edge] = sLR_Edge.getPointsOrderedByX();

            if (sLeftBestCandidate.has_value() && !sRightBestCandidate.has_value())
            {
                sMergeZone.emplace(Triangle(sLeftBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge));
                sLR_Edge = Edge(sLeftBestCandidate.value(), sRightPointOfLR_Edge, MERGE_PRECISION);
            }
            else if (!sLeftBestCandidate.has_value() && sRightBestCandidate.has_value())
            {
                sMergeZone.emplace(Triangle(sRightBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge));
                sLR_Edge = Edge(sLeftPointOfLR_Edge, sRightBestCandidate.value(), MERGE_PRECISION);
            }
            else 
            {
                const auto& sLeftTriangle = Triangle(sLeftBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge);
                if (sLeftTriangle.circumscribedCircleContains(sRightBestCandidate.value()))
                {
                    sMergeZone.emplace(Triangle(sRightBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge));
                    sLR_Edge = Edge(sLeftPointOfLR_Edge, sRightBestCandidate.value(), MERGE_PRECISION);
                }
                else
                {
                    sMergeZone.emplace(Triangle(sLeftBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge));
                    sLR_Edge = Edge(sLeftBestCandidate.value(), sRightPointOfLR_Edge, MERGE_PRECISION);
                }
            }
            
            sLeftBestCandidate = getBestCandidate(sLR_Edge, true);
            sRightBestCandidate = aZone.getBestCandidate(sLR_Edge, false);
        }

        return sMergeZone;
    }

    Edge getBottomEdge(const Zone& aZone)
    {
        auto sGetSortedOtherWay = [](std::set<Eigen::Vector2f>::const_iterator cbegin, std::set<Eigen::Vector2f>::const_iterator cend)
        { 
            std::vector<Eigen::Vector2f> sPoints(cbegin, cend);
            std::sort(sPoints.begin(), sPoints.end(), [](const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) { 
                Eigen::Vector2f b(p1.x()-p2.x(), p1.y()-p2.y());
                Eigen::Vector2f zero(0.0, 0.0);
                if (b.isApprox(zero, MERGE_PRECISION))
                    return false;
                return p1.y() > p2.y(); 
            }); 
            return sPoints; 
        }; 

        const auto& sSortedFromBottomToTop = sGetSortedOtherWay(points.cbegin(), points.cend());
        const auto& sOtherZoneSortedFromBottomToTop = sGetSortedOtherWay(aZone.points.cbegin(), aZone.points.cend());
        
        auto sOwnCandidateIt = sSortedFromBottomToTop.cbegin();
        auto sOtherCandidateIt = sOtherZoneSortedFromBottomToTop.cbegin();

        Edge sBottomEdge;

        while (sOwnCandidateIt != sSortedFromBottomToTop.cend() && sOtherCandidateIt != sOtherZoneSortedFromBottomToTop.cend())
        {
            sBottomEdge = Edge(*sOwnCandidateIt, *sOtherCandidateIt, MERGE_PRECISION);
            bool sNeedToContinue = false;

            for (const auto& sEdge: edges)
            {
                const auto& sIntersectionPoint = sBottomEdge.isIntersects(sEdge);
                if (sIntersectionPoint.has_value() && !sIntersectionPoint.value().isApprox(*sOwnCandidateIt, MERGE_PRECISION))
                {
                    sOwnCandidateIt++;
                    sNeedToContinue = true;  // нужно вернуться континьюить вайл
                    break;
                }
            }

            if (sNeedToContinue)  // выглядит нелепо, нужно придумать как сделать иначе континью внешнего цикла. Ну не исключение же кидать
                continue;

            for (const auto& sEdge: aZone.edges)
            {
                const auto& sIntersectionPoint = sBottomEdge.isIntersects(sEdge);
                if (sIntersectionPoint.has_value() && !sIntersectionPoint.value().isApprox(*sOtherCandidateIt, MERGE_PRECISION))
                {
                    sOtherCandidateIt++;
                    sNeedToContinue = true;  // нужно вернуться континьюить вайл
                    break;
                }
            }

            if (sNeedToContinue)  // выглядит нелепо, нужно придумать как сделать иначе континью внешнего цикла. Ну не исключение же кидать
                continue;

            break;  // нашли самый нижний отрезок соединяющий две зоны и не пересекающий их
        }

        return sBottomEdge;
    }

    bool isLeft(const Zone& aZone)  // наша зона левее чем переданная
    {
        if (empty() || aZone.empty())
            return true;
        
        const auto& sOwnPoint = *points.cbegin();
        const auto& sOtherZonePoint = *aZone.points.cbegin();
        
        if (sOwnPoint.x() < sOtherZonePoint.x())
            return true;
        else
            return false;
    }

    // слева нам нужно считать /_ угол, а справа _\, поэтому без параметра указывающего какая зона относительно соединения рассматривается увы не обойтись
    std::optional<Eigen::Vector2f> getBestCandidate(const Edge& aEdge, bool aIsLeftZone)  
    {
        std::vector<Edge> sEdges;
        std::copy_if(edges.begin(), edges.end(), std::back_inserter(sEdges), [aEdge](const Edge& e) { return e.hasCommonPoint(aEdge); });

        if (sEdges.empty())
            return std::nullopt;

        const auto& sCommonPoint = sEdges.begin()->getCommonPoint(aEdge);
        const auto& sNotFromZonePoint = aEdge.getNotCommonPoint(*sEdges.begin());

        std::set<Eigen::Vector2f, vecCompare> sBufPoints;
        // for (const auto& sEdge: sEdges)
        // {
        //     sPoints.insert(sEdge.getNotCommonPoint(aEdge);
        // }
        std::transform(sEdges.begin(), sEdges.end(), sinserter(sBufPoints), [aEdge](const Edge& e) { return e.getNotCommonPoint(aEdge); });
        std::vector<Eigen::Vector2f> sPoints(sBufPoints.begin(), sBufPoints.end());

        if (sPoints.size() == 1)
            return *sPoints.begin();
 
        std::sort(sPoints.begin(), sPoints.end(), [sCommonPoint, sNotFromZonePoint, aIsLeftZone](const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) {
            const auto& sInnerEdge1 = Edge(sCommonPoint, p1, MERGE_PRECISION);
            const auto& sInnerEdge2 = Edge(sCommonPoint, p2, MERGE_PRECISION);
            const auto& sOuterEdge = Edge(sNotFromZonePoint, sCommonPoint, MERGE_PRECISION);
            if (aIsLeftZone)
                return sOuterEdge.clockwiseAngle(sInnerEdge1) > sOuterEdge.clockwiseAngle(sInnerEdge2);
            else
                return sOuterEdge.clockwiseAngle(sInnerEdge1) < sOuterEdge.clockwiseAngle(sInnerEdge2);
        });

        std::erase_if(sPoints, [sCommonPoint, sNotFromZonePoint, aIsLeftZone](const Eigen::Vector2f& p) {
            const auto& sOuterEdge = Edge(sCommonPoint, sNotFromZonePoint, MERGE_PRECISION);
            const auto& sInnerEdge = Edge(sCommonPoint, p, MERGE_PRECISION);
            double sAngle = 0.0;
            if (aIsLeftZone)
                sAngle = sInnerEdge.clockwiseAngle(sOuterEdge);
            else
                sAngle = sOuterEdge.clockwiseAngle(sInnerEdge);
            return sAngle < 1e-3;
        });

        if (sPoints.empty())
            return std::nullopt;

        auto sCurrentBestCandidateIt = sPoints.cbegin();
        for (auto it = std::next(sCurrentBestCandidateIt); it != sPoints.cend(); ++it)
        {
            const auto& sCheckTriangle = Triangle(*sCurrentBestCandidateIt, sNotFromZonePoint, sCommonPoint);
            if (sCheckTriangle.circumscribedCircleContains(*it))
            {
                const auto& sEdgeToDelete = Edge(sCommonPoint, *sCurrentBestCandidateIt, MERGE_PRECISION);
                edges.erase(sEdgeToDelete);
                std::erase_if(triangles, [sEdgeToDelete](const auto& sTriangle) { return sTriangle.containsEdge(sEdgeToDelete); });
                
                sCurrentBestCandidateIt = it;
            }
            else 
                break;
        }

        return *sCurrentBestCandidateIt;
    }

};