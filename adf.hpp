#pragma once

#include <vector>
#include <algorithm>
#include <optional>
#include <unordered_set>
#include <set>
#include <deque>
#include <iostream>


#include "hull.hpp"
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef K::Segment_3 Segment_3;
typedef K::Triangle_3 Triangle_3;
typedef K::Intersect_3 Intersect_3;

namespace
{
// Function that return
// dot product of two vector array.
// float dotProduct(Point3D AB, Point3D CD)
// {
 
//     float product = 0.0;
//     product = product + AB.x * CD.x;
//     product = product + AB.y * CD.y;
//     product = product + AB.z * CD.z;
//     return product;
// }
 
// Function to find
// cross product of two vector array.
Point3D crossProduct(Point3D AB, Point3D CD)
{
    Point3D res;
    res.x = AB.y * CD.z - AB.z * CD.y;
    res.y = AB.z * CD.x - AB.x * CD.z;
    res.z = AB.x * CD.y - AB.y * CD.x;
    return res;
}

double lenSqr(const Point3D& p1, const Point3D& p2)
{
    Point3D AB(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
    return AB.x*AB.x + AB.y*AB.y + AB.z*AB.z;
}

double VolumeSign(const Face& f, const Point3D& p)
{
  double vol;
  double ax, ay, az, bx, by, bz, cx, cy, cz;
  ax = f.vertices[0].x - p.x;  
  ay = f.vertices[0].y - p.y;  
  az = f.vertices[0].z - p.z;
  bx = f.vertices[1].x - p.x;  
  by = f.vertices[1].y - p.y;  
  bz = f.vertices[1].z - p.z;
  cx = f.vertices[2].x - p.x;  
  cy = f.vertices[2].y - p.y;  
  cz = f.vertices[2].z - p.z;
  vol = ax * (by * cz - bz * cy) +\
        ay * (bz * cx - bx * cz) +\
        az * (bx * cy - by * cx);
  return vol;
}

bool isGreater(const Face& aFrontFace, const Point3D& aPoint1, const Point3D& aPoint2)  // p1 > p2
{
    // double sVolume1 = VolumeSign(aFrontFace, aPoint1);
    // double sVolume2 = VolumeSign(aFrontFace, aPoint2);
    // assert(sVolume1 > 0 && sVolume2 > 0);

    double Len1 = std::sqrt(lenSqr(aFrontFace.vertices[0], aPoint1) + 
                     lenSqr(aFrontFace.vertices[1], aPoint1) +
                     lenSqr(aFrontFace.vertices[2], aPoint1) +
                     lenSqr(aFrontFace.vertices[0], aFrontFace.vertices[1]) +
                     lenSqr(aFrontFace.vertices[1], aFrontFace.vertices[2]) +
                     lenSqr(aFrontFace.vertices[0], aFrontFace.vertices[2]));

    double Len2 = std::sqrt(lenSqr(aFrontFace.vertices[0], aPoint2) + 
                     lenSqr(aFrontFace.vertices[1], aPoint2) +
                     lenSqr(aFrontFace.vertices[2], aPoint2) +
                     lenSqr(aFrontFace.vertices[0], aFrontFace.vertices[1]) +
                     lenSqr(aFrontFace.vertices[1], aFrontFace.vertices[2]) +
                     lenSqr(aFrontFace.vertices[0], aFrontFace.vertices[2]));

    // return sVolume1/std::sqrt(Len1*Len1*Len1*Len1*Len1) > sVolume2/std::sqrt(Len2*Len2*Len2*Len2*Len2);
    return Len1 > Len2;
}

bool FaceSegment(const Face& aFrontFace, const Point3D& aPoint1, const Point3D& aPoint2)  // p1 > p2
{
    // std::cout << aFrontFace << std::endl;
    // std::cout << aPoint1 << std::endl;
    // std::cout << aPoint2 << std::endl;
    if ((aFrontFace.vertices[0] == aPoint1 && aFrontFace.vertices[1] == aPoint2) || (aFrontFace.vertices[0] == aPoint2 && aFrontFace.vertices[1] == aPoint1))
        return true;
    if ((aFrontFace.vertices[1] == aPoint1 && aFrontFace.vertices[2] == aPoint2) || (aFrontFace.vertices[1] == aPoint2 && aFrontFace.vertices[2] == aPoint1))
        return true;
    if ((aFrontFace.vertices[0] == aPoint1 && aFrontFace.vertices[2] == aPoint2) || (aFrontFace.vertices[0] == aPoint2 && aFrontFace.vertices[2] == aPoint1))
        return true;
    return false;
}

Point3D getNormal(const Point3D& A, const Point3D& B, const Point3D& C)
{
    Point3D AB(B.x - A.x, B.y - A.y, B.z - A.z);
    Point3D BC(C.x - B.x, C.y - B.y, C.z - B.z);
    auto sCross = crossProduct(BC, AB);
    return sCross;
}

}

// bool isGreaterByAlpha(const Edge& aFrontEdge, const Point3D& aPoint1, const Point3D& aPoint2)  // p1 > p2
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

struct ADF
{
    private:
    std::set<Point3D> points;
    std::vector<Face> faces;
    std::deque<Face> front;
    std::vector<Face> triangles;

    public:
    ADF(std::set<Point3D> aPoints): points(aPoints) 
    {}

    void fillFront()
    {
        std::vector<Point3D> sPoints(points.begin(), points.end());
        ConvexHull C(sPoints);
        auto sFaces = C.GetFaces();
        for (const auto& sFace: sFaces)
        {
            front.emplace_back(sFace);
            triangles.emplace_back(sFace);
        }
    }

    void splitFront()
    {
        auto sInitFrontSize = front.size();
        for (size_t i = 0; i < sInitFrontSize; ++i)
        {
            auto sFace = front.front();
            front.pop_front();

            splitAndPush(sFace);
        }
        std::set<Point3D> sP;
        for (const auto& sFace: front)
        {
            sP.emplace(sFace.vertices[0]);
            sP.emplace(sFace.vertices[1]);
            sP.emplace(sFace.vertices[2]);
        }
        std::vector<Point3D> sPoints(sP.begin(), sP.end());
        front.clear();
        triangles.clear();
        ConvexHull C(sPoints);
        auto sFaces = C.GetFaces();
        for (const auto& sFace: sFaces)
        {
            front.emplace_back(sFace);
            triangles.emplace_back(sFace);
        }
    }

    std::vector<Face> triangulate()
    {
        for (size_t i = 0; i < 500 && !front.empty(); ++i)  // for debug
        // for (int i = 0; !front.empty; ++i)
        {
            std::cout << "iteration " << i << std::endl;
            std::cout << "front size " << front.size() << std::endl;
            auto sFrontFace = front.front();
            front.pop_front();

            auto sPoint = generateFrontFacePoint(sFrontFace);
            Face f1(sFrontFace.vertices[0], sFrontFace.vertices[1], sPoint);
            Face f2(sFrontFace.vertices[1], sFrontFace.vertices[2], sPoint);
            Face f3(sFrontFace.vertices[2], sFrontFace.vertices[0], sPoint);
            assert(VolumeSign(sFrontFace, sPoint) > 0);

            auto sPoints = getFrontPoints();
            std::erase_if(sPoints, [sFrontFace](const Point3D& p) { 
                return VolumeSign(sFrontFace, p) < 1e-5;
            });

            auto lam = [&sFrontFace](const Point3D& p1, const Point3D& p2) 
            {
                return isGreater(sFrontFace, p2, p1); 
            };
            std::set<Point3D, decltype(lam)> sFrontPoints(sPoints.begin(), 
                                        sPoints.end(), 
                                        lam);

            auto it = sFrontPoints.cbegin();
            for (; it != sFrontPoints.end() && (isIntersectsTriangulation(f1) || isIntersectsTriangulation(f2) || isIntersectsTriangulation(f3)); ++it)
            {
                f1 = Face(sFrontFace.vertices[0], sFrontFace.vertices[1], *it); // !!!
                f2 = Face(sFrontFace.vertices[1], sFrontFace.vertices[2], *it);
                f3 = Face(sFrontFace.vertices[2], sFrontFace.vertices[0], *it);
                assert(VolumeSign(sFrontFace, sPoint) > 0);
            }

            if (it != sFrontPoints.end())
                pushFaces(sFrontFace, f1, f2, f3);
        }

        return triangles;
        // std::vector<Edge> buf(front.begin(), front.end());
        // return std::make_pair(buf, edges);
    }
    

    std::vector<Face> getFront() const
    {
        return std::vector<Face>(front.end()-4, front.end());
    }

    std::vector<Face> getFaces() const
    {
        std::vector<Face> buf(faces.begin(), faces.end());
        return buf;
        // return faces;
    }

    private:    
    bool isIntersectsTriangulation(const Face& aFace) const
    {
        Triangle_3 tr(Point_3(aFace.vertices[0].x, aFace.vertices[0].y, aFace.vertices[0].z), 
                      Point_3(aFace.vertices[1].x, aFace.vertices[1].y, aFace.vertices[1].z), 
                      Point_3(aFace.vertices[2].x, aFace.vertices[2].y, aFace.vertices[2].z));        
        for (const auto& sFrontFace: front)
        {
            if (sFrontFace == aFace)
            {
                continue;
            }
            Triangle_3 tr1(Point_3(sFrontFace.vertices[0].x, sFrontFace.vertices[0].y, sFrontFace.vertices[0].z), 
                           Point_3(sFrontFace.vertices[1].x, sFrontFace.vertices[1].y, sFrontFace.vertices[1].z), 
                           Point_3(sFrontFace.vertices[2].x, sFrontFace.vertices[2].y, sFrontFace.vertices[2].z));
            const auto result = intersection(tr1, tr);
            if (result)
            {
                if (const Segment_3* s = boost::get<Segment_3>(&*result))
                {  
                    // std::cout << "front segment " << *s << std::endl;
                    if (FaceSegment(sFrontFace, 
                                        Point3D(to_double(s->source().x()), to_double(s->source().y()), to_double(s->source().z())), 
                                        Point3D(to_double(s->target().x()), to_double(s->target().y()), to_double(s->target().z()))))
                    {
                        continue;
                    }
                }
                if (boost::get<Point_3>(&*result))
                {  
                    // std::cout << "front point " << *s << std::endl;
                    continue;
                }
                return true;
            }
        }
        for (const auto& sFace: faces)
        {
            if (sFace == aFace)
            {
                continue;
            }
            Triangle_3 tr1(Point_3(sFace.vertices[0].x, sFace.vertices[0].y, sFace.vertices[0].z), 
                           Point_3(sFace.vertices[1].x, sFace.vertices[1].y, sFace.vertices[1].z), 
                           Point_3(sFace.vertices[2].x, sFace.vertices[2].y, sFace.vertices[2].z));            
            const auto result = intersection(tr1, tr);
            if (result)
            {
                if (const Segment_3* s = boost::get<Segment_3>(&*result))
                {
                    // std::cout << "faces segment " << *s << std::endl;
                    if (FaceSegment(sFace, 
                                    Point3D(to_double(s->source().x()), to_double(s->source().y()), to_double(s->source().z())), 
                                    Point3D(to_double(s->target().x()), to_double(s->target().y()), to_double(s->target().z()))))
                    {
                        continue;
                    }
                }
                if (boost::get<Point_3>(&*result))
                {  
                    // std::cout << "face point " << *s << std::endl;
                    continue;
                }
                return true;
            }
        }
        return false;
    }

    void splitAndPush(const Face& sFace)
    {
        auto ABmid = Point3D((sFace.vertices[1].x - sFace.vertices[0].x)/2, (sFace.vertices[1].y - sFace.vertices[0].y)/2, (sFace.vertices[1].z - sFace.vertices[0].z)/2);
        auto BCmid = Point3D((sFace.vertices[2].x - sFace.vertices[1].x)/2, (sFace.vertices[2].y - sFace.vertices[1].y)/2, (sFace.vertices[2].z - sFace.vertices[1].z)/2);
        auto CAmid = Point3D((sFace.vertices[0].x - sFace.vertices[2].x)/2, (sFace.vertices[0].y - sFace.vertices[2].y)/2, (sFace.vertices[0].z - sFace.vertices[2].z)/2);

        auto f1 = Face(sFace.vertices[0], sFace.vertices[0] + ABmid, sFace.vertices[2] + CAmid);
        auto f2 = Face(sFace.vertices[0] + ABmid, sFace.vertices[1], sFace.vertices[1] + BCmid);
        auto f3 = Face(sFace.vertices[2] + CAmid, sFace.vertices[1] + BCmid, sFace.vertices[2]);
        auto f4 = Face(sFace.vertices[2] + CAmid, sFace.vertices[1] + BCmid, sFace.vertices[0] + ABmid);

        auto f1NormalVec = getNormal(f1.vertices[0], f1.vertices[1], f1.vertices[2]);
        auto f2NormalVec = getNormal(f2.vertices[0], f2.vertices[1], f2.vertices[2]);
        auto f3NormalVec = getNormal(f3.vertices[0], f3.vertices[1], f3.vertices[2]);
        auto f4NormalVec = getNormal(f4.vertices[0], f4.vertices[1], f4.vertices[2]);

        auto f1NormalValue = f1NormalVec.x*f1NormalVec.x + f1NormalVec.y*f1NormalVec.y + f1NormalVec.z*f1NormalVec.z;
        auto f2NormalValue = f2NormalVec.x*f2NormalVec.x + f2NormalVec.y*f2NormalVec.y + f2NormalVec.z*f2NormalVec.z;
        auto f3NormalValue = f3NormalVec.x*f3NormalVec.x + f3NormalVec.y*f3NormalVec.y + f3NormalVec.z*f3NormalVec.z;
        auto f4NormalValue = f4NormalVec.x*f4NormalVec.x + f4NormalVec.y*f4NormalVec.y + f4NormalVec.z*f4NormalVec.z;

        constexpr double maxArea = 0.1;
        if (f1NormalValue > maxArea)
            splitAndPush(f1);
        else
            front.push_back(f1);

        if (f2NormalValue > maxArea)
            splitAndPush(f2);
        else
            front.push_back(f2);

        if (f3NormalValue > maxArea)
            splitAndPush(f3);
        else
            front.push_back(f3);

        if (f4NormalValue > maxArea)
            splitAndPush(f4);
        else
            front.push_back(f4);

    }

    Point3D generateFrontFacePoint(const Face& aFrontFace) const
    {
        return getCandidateFrontFacePoint(aFrontFace);
    }

    Point3D getCandidateFrontFacePoint(const Face& aFrontFace) const
    {
        float sCenterX = (aFrontFace.vertices[0].x + aFrontFace.vertices[1].x + aFrontFace.vertices[2].x)/3;
        float sCenterY = (aFrontFace.vertices[0].y + aFrontFace.vertices[1].y + aFrontFace.vertices[2].y)/3;
        float sCenterZ = (aFrontFace.vertices[0].z + aFrontFace.vertices[1].z + aFrontFace.vertices[2].z)/3;
        Point3D sMidPoint(sCenterX, sCenterY, sCenterZ);

        Point3D normalVec = getNormal(aFrontFace.vertices[0], aFrontFace.vertices[1], aFrontFace.vertices[2]);

        auto sLen = std::sqrt(normalVec.x*normalVec.x + normalVec.y*normalVec.y + normalVec.z*normalVec.z);
        normalVec.x /= sLen;
        normalVec.y /= sLen;
        normalVec.z /= sLen;
        return Point3D(normalVec.x/4+sMidPoint.x, normalVec.y/4+sMidPoint.y, normalVec.z/4+sMidPoint.z);
    }

    std::set<Point3D> getFrontPoints()
    {
        std::set<Point3D> sResult;
        for (const auto& sFrontFace: front)
        {
            sResult.insert(sFrontFace.vertices[0]);
            sResult.insert(sFrontFace.vertices[1]);
            sResult.insert(sFrontFace.vertices[2]);
        }

        return sResult;
    }

    void pushFaces(const Face& aFrontFace, const Face& aNewFace1, const Face& aNewFace2, const Face& aNewFace3)
    {
        auto sFindE1It = std::find(front.begin(), front.end(), aNewFace1);
        if (sFindE1It != front.end())
        {
            front.erase(sFindE1It);
            faces.push_back(aNewFace1);       
        }
        else
        {
            front.push_back(aNewFace1);
        }


        // auto n1 = getNormal(aNewFace1.vertices[0], aNewFace1.vertices[1],  aNewFace1.vertices[2]);
        // faces.push_back(Face(n1+aNewFace1.vertices[0], Point3D(0.0, 0.0, 0.0), Point3D(0.0, 0.0, 1.0)));

        auto sFindE2It = std::find(front.begin(), front.end(), aNewFace2);
        if (sFindE2It != front.end())
        {
            front.erase(sFindE2It);
            faces.push_back(aNewFace2);
        }
        else
        {
            front.push_back(aNewFace2);
        }


        // auto n2 = getNormal(aNewFace2.vertices[0], aNewFace2.vertices[1],  aNewFace2.vertices[2]);
        // faces.push_back(Face(n2+aNewFace2.vertices[0], Point3D(0.0, 0.0, 0.0), Point3D(0.0, 0.0, 1.0)));

        auto sFindE3It = std::find(front.begin(), front.end(), aNewFace3);
        if (sFindE3It != front.end())
        {
            front.erase(sFindE3It);
            faces.push_back(aNewFace3);
        }
        else
        {
            front.push_back(aNewFace3);
        }

        // auto n3 = getNormal(aNewFace3.vertices[0], aNewFace3.vertices[1],  aNewFace3.vertices[2]);
        // faces.push_back(Face(n3+aNewFace3.vertices[0], Point3D(0.0, 0.0, 0.0), Point3D(0.0, 0.0, 1.0)));
        // return;
        faces.push_back(aFrontFace);

        triangles.push_back(aFrontFace);
        triangles.push_back(aNewFace1);
        triangles.push_back(aNewFace2);
        triangles.push_back(aNewFace3);
    }

};