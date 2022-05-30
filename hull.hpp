#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <cassert>
#include "Point3D.hpp"

// Defined in CCW
struct Face
{
  Face(const Point3D& p1, const Point3D& p2, const Point3D& p3): visible(false)
      { vertices[0] = p1; vertices[1] = p2; vertices[2] = p3;};

  void Reverse(){std::swap(vertices[0], vertices[2]); };

  friend std::ostream& operator<<(std::ostream& os, const Face& f)
  {
    os << "[face pt1 = "   << f.vertices[0].ToString()
       << " | face pt2 = " << f.vertices[1].ToString()
       << " | face pt3 = " << f.vertices[2].ToString()<<"] ";
    return os;
  }

  bool operator==(const Face& f) const
  {
      return (vertices[0] == f.vertices[0] && vertices[1] == f.vertices[1] && vertices[2] == f.vertices[2]) ||
             (vertices[0] == f.vertices[0] && vertices[1] == f.vertices[2] && vertices[2] == f.vertices[1]) ||
             (vertices[0] == f.vertices[1] && vertices[1] == f.vertices[0] && vertices[2] == f.vertices[2]) ||
             (vertices[0] == f.vertices[2] && vertices[1] == f.vertices[0] && vertices[2] == f.vertices[1]) ||
             (vertices[0] == f.vertices[2] && vertices[1] == f.vertices[1] && vertices[2] == f.vertices[0]) ||
             (vertices[0] == f.vertices[1] && vertices[1] == f.vertices[2] && vertices[2] == f.vertices[0]);
  }

  bool visible;
  Point3D vertices[3];
};

struct Edge
{
  Edge(const Point3D& p1, const Point3D& p2): 
      adjface1(nullptr), adjface2(nullptr), remove(false) 
      { endpoints[0] = p1; endpoints[1] = p2; };
  
  void LinkAdjFace(Face* face) 
  {
    if( adjface1 != NULL && adjface2 != NULL )
    {
      std::cout<<"warning: property violated!\n";
    }
    (adjface1 == NULL ? adjface1 : adjface2)= face;
  };

  void Erase(Face* face) 
  {
    if(adjface1 != face && adjface2 != face) return;
    (adjface1 == face ? adjface1 : adjface2) = nullptr;
  };

  friend std::ostream& operator<<(std::ostream& os, const Edge& e)
  {
    os << "[edge pt1 = "   << e.endpoints[0].ToString()
       << " | edge pt2 = " << e.endpoints[1].ToString() << " ]";
    return os;
  }

  Face *adjface1, *adjface2;
  bool remove; 
  Point3D endpoints[2];
};

class ConvexHull
{
  public:
    template<typename T> ConvexHull(const std::vector<T>& points);
    // All major works are conducted upon calling of constructor

    ~ConvexHull() = default;

    template<typename T> bool Contains(T p) const;
    // In out test for a query point (surface point is considered outside)

    std::list<Face> GetFaces() const {return this->faces;};
    std::list<Edge> GetEdges() const {return this->edges;};
    
    const std::vector<Point3D> GetVertices() const \
        {return this->exterior_points;};
    // Return exterior vertices than defines the convell hull

    void Print(const std::string mode);
    // mode {face, edge, vertice}

    int Size() const {return this->exterior_points.size();};

  private:

    bool Colinear(const Point3D& a, const Point3D& b, const Point3D& c) const;

    bool CoPlanar(Face& f, Point3D& p);

    int VolumeSign(const Face& f, const Point3D& p) const;
    // A point is considered outside of a CCW face if the volume of tetrahedron
    // formed by the face and point is negative. Note that origin is set at p.

    size_t Key2Edge(const Point3D& a, const Point3D& b) const;
    // Hash key for edge. hash(a, b) = hash(b, a)

    void AddOneFace(const Point3D& a, const Point3D& b, 
        const Point3D& c, const Point3D& inner_pt);
    // Inner point is used to make the orientation of face consistent in counter-
    // clockwise direction

    bool BuildFirstHull(std::vector<Point3D>& pointcloud);
    // Build a tetrahedron as first convex hull

    void IncreHull(const Point3D& p);

    void ConstructHull(std::vector<Point3D>& pointcloud);

    void CleanUp();

    void ExtractExteriorPoints();

    Point3D FindInnerPoint(const Face* f, const Edge& e);
    // for face(a,b,c) and edge(a,c), return b

    std::vector<Point3D> pointcloud = {};
    std::vector<Point3D> exterior_points = {};
    std::list<Face> faces = {};
    std::list<Edge> edges = {};
    std::unordered_map<size_t, Edge*> map_edges;
};

template<typename T> ConvexHull::ConvexHull(const std::vector<T>& points)
{
  const int n = points.size();
  this->pointcloud.resize(n);
  for(int i = 0; i < n; i++)
  { 
    this->pointcloud[i].x = points[i].x;
    this->pointcloud[i].y = points[i].y;
    this->pointcloud[i].z = points[i].z;
  }
  this->ConstructHull(this->pointcloud);
}

template<typename T> bool ConvexHull::Contains(T p) const
{
  Point3D pt(p.x, p.y, p.z);
  for(auto& face : this->faces)
  {
    if(VolumeSign(face, pt) <= 0) return false;
  }
  return true;
}

size_t ConvexHull::Key2Edge(const Point3D& a, const Point3D& b) const
{
  point_hash ph;
  return ph(a) ^ ph(b);
}

int ConvexHull::VolumeSign(const Face& f, const Point3D& p) const
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
  if(vol == 0) return 0;
  return vol < 0 ? -1 : 1;
}

void ConvexHull::AddOneFace(const Point3D& a, const Point3D& b, 
    const Point3D& c, const Point3D& inner_pt)
{
  // Make sure face is CCW with face normal pointing outward
  this->faces.emplace_back(a, b, c);
  auto& new_face = this->faces.back();
  if(this->VolumeSign(this->faces.back(), inner_pt) < 0) new_face.Reverse();

  // Create edges and link them to face pointer
  auto create_edge = [&](const Point3D& p1, const Point3D& p2)
  {
    size_t key = this->Key2Edge(p1, p2);
    if(!this->map_edges.count(key)) 
    { 
      this->edges.emplace_back(p1, p2);
      this->map_edges.insert({key, &this->edges.back()});
    }
    this->map_edges[key]->LinkAdjFace(&new_face);
  };
  create_edge(a, b);
  create_edge(a, c);
  create_edge(b, c);
}

bool ConvexHull::Colinear(const Point3D& a, const Point3D& b, const Point3D& c) const
{
  return ((c.z - a.z) * (b.y - a.y) - 
          (b.z - a.z) * (c.y - a.y)) == 0 &&\
         ((b.z - a.z) * (c.x - a.x) - 
          (b.x - a.x) * (c.z - a.z)) == 0 &&\
         ((b.x - a.x) * (c.y - a.y) - 
          (b.y - a.y) * (c.x - a.x)) == 0;
}

bool ConvexHull::BuildFirstHull(std::vector<Point3D>& pointcloud)
{
  const int n = pointcloud.size();
  if(n <= 3)
  {
    std::cout<<"Tetrahedron: points.size() < 4\n";
    return false;    
  }

  int i = 2;
  while(this->Colinear(pointcloud[i], pointcloud[i-1], pointcloud[i-2]))
  {
    if(i++ == n - 1)
    {
      std::cout<<"Tetrahedron: All points are colinear!\n";
      return false;
    }
  }

  Face face(pointcloud[i], pointcloud[i-1], pointcloud[i-2]);

  int j = i;
  while(!this->VolumeSign(face, pointcloud[j]))
  {
    if(j++ == n-1)
    {
      std::cout<<"Tetrahedron: All pointcloud are coplanar!\n";
      return false;    
    }
  }

  auto& p1 = pointcloud[i];    auto& p2 = pointcloud[i-1];
  auto& p3 = pointcloud[i-2];  auto& p4 = pointcloud[j];
  p1.processed = p2.processed = p3.processed = p4.processed = true;
  this->AddOneFace(p1, p2, p3, p4);
  this->AddOneFace(p1, p2, p4, p3);
  this->AddOneFace(p1, p3, p4, p2);
  this->AddOneFace(p2, p3, p4, p1);
  return true;

}
 
Point3D ConvexHull::FindInnerPoint(const Face* f, const Edge& e)
{
  for(int i = 0; i < 3; i++)
  {
    if(f->vertices[i] == e.endpoints[0]) continue;
    if(f->vertices[i] == e.endpoints[1]) continue;
    return f->vertices[i];
  } 
  assert(false);
  return f->vertices[0];
}

void ConvexHull::IncreHull(const Point3D& pt)
{
  // Find the illuminated faces (which will be removed later)
  bool vis = false;
  for(auto& face : this->faces)
  {
    if(VolumeSign(face, pt) < 0) 
    {
      //std::cout<<"face illuminated by pt "<<pt<<" is \n"<<face<<"\n";
      face.visible = vis = true;
    }
  }
  if(!vis) return;

  // Find the edges to make new tagent surface or to be removed
  for(auto it = this->edges.begin(); it != this->edges.end(); it++)
  {
    auto& edge = *it;
    auto& face1 = edge.adjface1;
    auto& face2 = edge.adjface2;

    // Newly added edge
    if(face1 == NULL || face2 == NULL)
    {
      continue;
    }

    // This edge is to be removed because two adjacent faces will be removed 
    else if(face1->visible && face2->visible) 
    {
      edge.remove = true;
    }

    // Edge on the boundary of visibility, which will be used to extend a tagent
    // cone surface.
    else if(face1->visible|| face2->visible) 
    {
      if(face1->visible) std::swap(face1, face2);
      auto inner_pt = this->FindInnerPoint(face2, edge);
      edge.Erase(face2);
      this->AddOneFace(edge.endpoints[0], edge.endpoints[1], pt, inner_pt);
    }
  }
}

void ConvexHull::ConstructHull(std::vector<Point3D>& pointcloud)
{
  if(!this->BuildFirstHull(pointcloud)) return;
  for(const auto& pt : pointcloud)
  {
    if(pt.processed) continue;
    this->IncreHull(pt);
    this->CleanUp();
  }
  this->ExtractExteriorPoints();
}

void ConvexHull::CleanUp()
{
  auto it_edge = this->edges.begin();
  while(it_edge != this->edges.end())
  {
    if(it_edge->remove)
    {
      auto pt1 = it_edge->endpoints[0];
      auto pt2 = it_edge->endpoints[1];
      auto key_to_evict = this->Key2Edge(pt1, pt2);
      this->map_edges.erase(key_to_evict);
      this->edges.erase(it_edge++);
    }
    else it_edge++;
  };
  auto it_face = this->faces.begin();
  while(it_face != this->faces.end())
  {
    if(it_face->visible) this->faces.erase(it_face++);
    else it_face++;
  }
}

void ConvexHull::ExtractExteriorPoints()
{
  std::unordered_set<Point3D, point_hash> exterior_set;
  for(const auto& f : this->faces)
  {
    for(int i =0; i < 3; i++)
      exterior_set.insert(f.vertices[i]);
  }
  this->exterior_points = \
      std::vector<Point3D>(exterior_set.begin(), exterior_set.end());
}

void ConvexHull::Print(const std::string mode = "none")
{
  if(mode == "vertice")
  {
    for(const auto& pt : this->exterior_points)  std::cout<<pt<<"\n";
  }    
  else if(mode == "edge")
  {
    for(const auto& e : this->edges)  std::cout<<(e)<<"\n";
  }
  else if( mode == "face")
  {
    for(const auto& f : this->faces)  std::cout<<f<<"\n";
  }
  else
  {
    std::cout<<"Print Usage: Print {'vertice', 'edge', 'face'}\n";
  }
}
