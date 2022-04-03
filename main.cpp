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

int main() {
    std::random_device rd;
    std::mt19937 gen_for_int(rd());
    std::mt19937 gen_for_double(rd());
    std::uniform_int_distribution<> int_gen(10, 100);
    std::uniform_real_distribution<> height_gen(1.0, 300.0);
    std::uniform_real_distribution<> width_gen(1.0, 400.0);

    constexpr size_t N = 20;  // number of points
    constexpr size_t n = 4;  // average number of points in cell

    Zone sLeftZone, sRightZone;

    for (int i = 0; i < N; ++i)
    {
        double sPointWidth = width_gen(gen_for_double);
        double sPointHeigth = height_gen(gen_for_double);
        
        if (sPointWidth < 200)
            sLeftZone.emplace(Point(sPointWidth, sPointHeigth));
        else
            sRightZone.emplace(Point(sPointWidth, sPointHeigth));
    }

    sLeftZone.triangulate();
    sRightZone.triangulate();

    const auto& sBottomEdge = sLeftZone.getBottomEdge(sRightZone);
    const auto& sLeftBestCandidate = sLeftZone.getBestCandidate(sBottomEdge.value());
    const auto& sRightBestCandidate = sRightZone.getBestCandidate(sBottomEdge.value());

    std::vector<Edge> edges;
    edges.insert(edges.end(), sLeftZone.edges.begin(), sLeftZone.edges.end());
    edges.insert(edges.end(), sRightZone.edges.begin(), sRightZone.edges.end());


    // create the window
    sf::RenderWindow window(sf::VideoMode(400, 300), "My window");
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

        if (sBottomEdge.has_value())
        {
            sf::Vertex sBottom[] =
            {
                    sf::Vertex(sf::Vector2f(sBottomEdge.value().p1.x, sBottomEdge.value().p1.y)),
                    sf::Vertex(sf::Vector2f(sBottomEdge.value().p2.x, sBottomEdge.value().p2.y)),
            };

            window.draw(sBottom, 2, sf::Lines);
        }


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
