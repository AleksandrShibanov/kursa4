#include <iostream>
#include <set>
#include <cmath>
#include <limits>
#include <random>
#include <SFML/Graphics.hpp>
#include <boost/thread.hpp>
#include <future>
#include <stack>
#include <queue>
#include <fstream>

#include "edge.hpp"
#include "point.hpp"
#include "triangle.hpp"
#include "incremental.hpp"
#include "Zone.hpp"

double determinant(  Point& a,   Point& b,   Point& c,   Point& d) {
    return (a.x - d.x) * (b.y - d.y) * (std::pow(c.x - d.x,2) + std::pow(c.y - d.y,2)) +
           (b.x - d.x) * (c.y - d.y) * (std::pow(a.x - d.x,2) + std::pow(a.y - d.y,2)) +
           (c.x - d.x) * (a.y - d.y) * (std::pow(b.x - d.x,2) + std::pow(b.y - d.y,2)) -
           (c.x - d.x) * (b.y - d.y) * (std::pow(a.x - d.x,2) + std::pow(a.y - d.y,2)) -
           (a.x - d.x) * (c.y - d.y) * (std::pow(b.x - d.x,2) + std::pow(b.y - d.y,2)) -
           (b.x - d.x) * (a.y - d.y) * (std::pow(c.x - d.x,2) + std::pow(c.y - d.y,2));
}

void writeDump(const std::set<Point>& aPoints)
{
    std::string filename("dump.txt");
    std::fstream output_fstream;

    output_fstream.open(filename, std::ios_base::out);
    if (!output_fstream.is_open()) {
        std::cout << "Failed to open " << filename << '\n';
    } else {
        for (const auto& sPoint: aPoints)
        {
            output_fstream << sPoint << std::endl;
        }
        std::cout << "Done Writing!" << std::endl;
    }
}

void readDump(std::set<Point>& aPoints)
{
    std::ifstream sInputFile;
    sInputFile.open("dump.txt");
    
    double sX, sY;
    while (sInputFile >> sX >> sY)
    {
        aPoints.insert(Point(sX, sY));
    }

    sInputFile.close();    
}

int main() {
    std::random_device rd;
    std::mt19937 gen_for_int(rd());
    std::mt19937 gen_for_double(rd());
    std::uniform_int_distribution<> int_gen(10, 100);
    std::uniform_real_distribution<> height_gen(1.0, 900.0);
    std::uniform_real_distribution<> width_gen(1.0, 1600.0);

    constexpr size_t N = 20;  // number of points
    constexpr size_t n = 4;  // average number of points in cell

    Zone sLeftZone, sRightZone, sMergeZone;
    std::set<Point> sPoints;
    // readDump(sPoints);

    for (int i = 0; i < N; ++i)
    {
        double sPointWidth = width_gen(gen_for_double);
        double sPointHeigth = height_gen(gen_for_double);
        sPoints.insert(Point(sPointWidth, sPointHeigth));
    }

    for (const auto& sPoint: sPoints)
    {
        if (sPoint.x < 800)
            sLeftZone.emplace(Point(sPoint.x, sPoint.y));
        else
            sRightZone.emplace(Point(sPoint.x, sPoint.y));
    }

    // writeDump(sPoints);

    sLeftZone.triangulate();
    sRightZone.triangulate();

    std::vector<Edge> edges;


    auto sLR_Edge = sLeftZone.getBottomEdge(sRightZone);
    edges.push_back(sLR_Edge.value());

    auto sLeftBestCandidate = sLeftZone.getBestCandidate(sLR_Edge.value(), true);
    auto sRightBestCandidate = sRightZone.getBestCandidate(sLR_Edge.value(), false);

    while (sLeftBestCandidate.has_value() || sRightBestCandidate.has_value())
    {
        Point sLeftPointOfLR_Edge, sRightPointOfLR_Edge;
        [&sLeftPointOfLR_Edge, &sRightPointOfLR_Edge](const auto& aLR_Edge){
            if (aLR_Edge.p1.x < aLR_Edge.p2.x)
            {
                sLeftPointOfLR_Edge = aLR_Edge.p1;
                sRightPointOfLR_Edge = aLR_Edge.p2;
            } 
            else 
            {
                sLeftPointOfLR_Edge = aLR_Edge.p2;
                sRightPointOfLR_Edge = aLR_Edge.p1;
            }
                
        }(sLR_Edge.value());


        if (sLeftBestCandidate.has_value() && !sRightBestCandidate.has_value())
        {
            sLeftBestCandidate.value().color = sf::Color::Red;
            sMergeZone.points.emplace(sLeftBestCandidate.value());
            sMergeZone.points.emplace(sLeftPointOfLR_Edge);
            sMergeZone.points.emplace(sRightPointOfLR_Edge);

            sMergeZone.edges.emplace(sLR_Edge.value());
            sMergeZone.edges.emplace(Edge(sLeftPointOfLR_Edge, sLeftBestCandidate.value()));
            sMergeZone.edges.emplace(Edge(sRightPointOfLR_Edge, sLeftBestCandidate.value()));

            sMergeZone.triangles.emplace_back(Triangle(sLeftBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge));

            sLR_Edge = Edge(sLeftBestCandidate.value(), sRightPointOfLR_Edge);
        }
        else if (!sLeftBestCandidate.has_value() && sRightBestCandidate.has_value())
        {
            sRightBestCandidate.value().color = sf::Color::Red;
            sMergeZone.points.emplace(sRightBestCandidate.value());
            sMergeZone.points.emplace(sLeftPointOfLR_Edge);
            sMergeZone.points.emplace(sRightPointOfLR_Edge);

            sMergeZone.edges.emplace(sLR_Edge.value());
            sMergeZone.edges.emplace(Edge(sLeftPointOfLR_Edge, sRightBestCandidate.value()));
            sMergeZone.edges.emplace(Edge(sRightPointOfLR_Edge, sRightBestCandidate.value()));

            sMergeZone.triangles.emplace_back(Triangle(sRightBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge));

            sLR_Edge = Edge(sLeftPointOfLR_Edge, sRightBestCandidate.value());
        }
        else 
        {
            const auto& sLeftTriangle = Triangle(sLeftBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge);
            if (sLeftTriangle.circumscribedCircleContains(sRightBestCandidate.value()))
            {
                sRightBestCandidate.value().color = sf::Color::Red;
                sMergeZone.points.emplace(sRightBestCandidate.value());
                sMergeZone.points.emplace(sLeftPointOfLR_Edge);
                sMergeZone.points.emplace(sRightPointOfLR_Edge);

                sMergeZone.edges.emplace(sLR_Edge.value());
                sMergeZone.edges.emplace(Edge(sLeftPointOfLR_Edge, sRightBestCandidate.value()));
                sMergeZone.edges.emplace(Edge(sRightPointOfLR_Edge, sRightBestCandidate.value()));

                sMergeZone.triangles.emplace_back(Triangle(sRightBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge));
                sLR_Edge = Edge(sLeftPointOfLR_Edge, sRightBestCandidate.value());
            }
            else
            {
                sLeftBestCandidate.value().color = sf::Color::Red;
                sMergeZone.points.emplace(sLeftBestCandidate.value());
                sMergeZone.points.emplace(sLeftPointOfLR_Edge);
                sMergeZone.points.emplace(sRightPointOfLR_Edge);

                sMergeZone.edges.emplace(sLR_Edge.value());
                sMergeZone.edges.emplace(Edge(sLeftPointOfLR_Edge, sLeftBestCandidate.value()));
                sMergeZone.edges.emplace(Edge(sRightPointOfLR_Edge, sLeftBestCandidate.value()));

                sMergeZone.triangles.emplace_back(Triangle(sLeftBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge));

                sLR_Edge = Edge(sLeftBestCandidate.value(), sRightPointOfLR_Edge);
            }
        }
        
        sLeftBestCandidate = sLeftZone.getBestCandidate(sLR_Edge.value(), true);
        sRightBestCandidate = sRightZone.getBestCandidate(sLR_Edge.value(), false);
    }

    /* for (auto i = 0; i < 10; ++i)
    {
        Point sLeftPointOfLR_Edge, sRightPointOfLR_Edge;
        [&sLeftPointOfLR_Edge, &sRightPointOfLR_Edge](const auto& aLR_Edge){
            if (aLR_Edge.p1.x < aLR_Edge.p2.x)
            {
                sLeftPointOfLR_Edge = aLR_Edge.p1;
                sRightPointOfLR_Edge = aLR_Edge.p2;
            } 
            else 
            {
                sLeftPointOfLR_Edge = aLR_Edge.p2;
                sRightPointOfLR_Edge = aLR_Edge.p1;
            }
                
        }(sLR_Edge.value());
        sLeftPointOfLR_Edge.color = sf::Color::Red;
        sRightPointOfLR_Edge.color = sf::Color::Red;

        if (sLeftBestCandidate.has_value() && !sRightBestCandidate.has_value())
        {
            sLeftBestCandidate.value().color = sf::Color::Red;
            sMergeZone.points.emplace(sLeftBestCandidate.value());
            sMergeZone.points.emplace(sLeftPointOfLR_Edge);
            sMergeZone.points.emplace(sRightPointOfLR_Edge);

            sMergeZone.edges.emplace(sLR_Edge.value());
            sMergeZone.edges.emplace(Edge(sLeftPointOfLR_Edge, sLeftBestCandidate.value()));
            sMergeZone.edges.emplace(Edge(sRightPointOfLR_Edge, sLeftBestCandidate.value()));

            sMergeZone.triangles.emplace_back(Triangle(sLeftBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge));

            sLR_Edge = Edge(sLeftBestCandidate.value(), sRightPointOfLR_Edge);
        }
        else if (!sLeftBestCandidate.has_value() && sRightBestCandidate.has_value())
        {
            sRightBestCandidate.value().color = sf::Color::Red;
            sMergeZone.points.emplace(sRightBestCandidate.value());
            sMergeZone.points.emplace(sLeftPointOfLR_Edge);
            sMergeZone.points.emplace(sRightPointOfLR_Edge);

            sMergeZone.edges.emplace(sLR_Edge.value());
            sMergeZone.edges.emplace(Edge(sLeftPointOfLR_Edge, sRightBestCandidate.value()));
            sMergeZone.edges.emplace(Edge(sRightPointOfLR_Edge, sRightBestCandidate.value()));

            sMergeZone.triangles.emplace_back(Triangle(sRightBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge));

            sLR_Edge = Edge(sLeftPointOfLR_Edge, sRightBestCandidate.value());
        }
        else if (sLeftBestCandidate.has_value() && sRightBestCandidate.has_value())
        {
            const auto& sLeftTriangle = Triangle(sLeftBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge);
            if (sLeftTriangle.circumscribedCircleContains(sRightBestCandidate.value()))
            {
                sRightBestCandidate.value().color = sf::Color::Red;
                sMergeZone.points.emplace(sRightBestCandidate.value());
                sMergeZone.points.emplace(sLeftPointOfLR_Edge);
                sMergeZone.points.emplace(sRightPointOfLR_Edge);

                sMergeZone.edges.emplace(sLR_Edge.value());
                sMergeZone.edges.emplace(Edge(sLeftPointOfLR_Edge, sRightBestCandidate.value()));
                sMergeZone.edges.emplace(Edge(sRightPointOfLR_Edge, sRightBestCandidate.value()));

                sMergeZone.triangles.emplace_back(Triangle(sRightBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge));
                sLR_Edge = Edge(sLeftPointOfLR_Edge, sRightBestCandidate.value());
            }
            else
            {
                sLeftBestCandidate.value().color = sf::Color::Red;
                sMergeZone.points.emplace(sLeftBestCandidate.value());
                sMergeZone.points.emplace(sLeftPointOfLR_Edge);
                sMergeZone.points.emplace(sRightPointOfLR_Edge);

                sMergeZone.edges.emplace(sLR_Edge.value());
                sMergeZone.edges.emplace(Edge(sLeftPointOfLR_Edge, sLeftBestCandidate.value()));
                sMergeZone.edges.emplace(Edge(sRightPointOfLR_Edge, sLeftBestCandidate.value()));

                sMergeZone.triangles.emplace_back(Triangle(sLeftBestCandidate.value(), sLeftPointOfLR_Edge, sRightPointOfLR_Edge));

                sLR_Edge = Edge(sLeftBestCandidate.value(), sRightPointOfLR_Edge);
            }
        }

        sLeftBestCandidate = sLeftZone.getBestCandidate(sLR_Edge.value(), true);
        sRightBestCandidate = sRightZone.getBestCandidate(sLR_Edge.value(), false);
    } */


    edges.insert(edges.end(), sLeftZone.edges.begin(), sLeftZone.edges.end());
    edges.insert(edges.end(), sRightZone.edges.begin(), sRightZone.edges.end());
    edges.insert(edges.end(), sMergeZone.edges.begin(), sMergeZone.edges.end());


    // create the window
    sf::RenderWindow window(sf::VideoMode(1600, 900), "My window");
    // run the program as long as the window is open
    while (window.isOpen())
    {
        // check all the window's events that were triggered since the last iteration of the loop
        sf::Event event;
        while (window.pollEvent(event))
        {
            // "close requested" event: we close the window
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // clear the window with black color
        sf::Color darkGreyGreen(10, 30, 10, 255);
        window.clear(darkGreyGreen);

        // if (sBottomEdge.has_value())
        // {
        //     sf::Vertex sBottom[] =
        //     {
        //             sf::Vertex(sf::Vector2f(sBottomEdge.value().p1.x, sBottomEdge.value().p1.y)),
        //             sf::Vertex(sf::Vector2f(sBottomEdge.value().p2.x, sBottomEdge.value().p2.y)),
        //     };

        //     window.draw(sBottom, 2, sf::Lines);
        // }


        if (sLeftBestCandidate.has_value())
        {
            sf::CircleShape circle(5);
            circle.setPosition(sLeftBestCandidate.value().x, sLeftBestCandidate.value().y);
            circle.setFillColor(sf::Color::Red);

            window.draw(circle);
        }

        if (sRightBestCandidate.has_value())
        {
            sf::CircleShape circle(5);
            circle.setPosition(sRightBestCandidate.value().x, sRightBestCandidate.value().y);
            circle.setFillColor(sf::Color::Red);

            window.draw(circle);
        }


        for (const auto& sEdge: edges) 
        {
            sf::Vertex line[] =
            {
                    sf::Vertex(sf::Vector2f(sEdge.p1.x, sEdge.p1.y), sEdge.p1.color),
                    sf::Vertex(sf::Vector2f(sEdge.p2.x, sEdge.p2.y), sEdge.p2.color),
            };
            window.draw(line, 2, sf::Lines);
        }


        // end the current frame
        window.display();
    }

    return 0;


}
