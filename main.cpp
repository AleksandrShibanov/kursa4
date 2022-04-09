#include <iostream>
#include <random>
#include <SFML/Graphics.hpp>
#include <fstream>
#include <omp.h>
#include <chrono>

#include "edge.hpp"
#include "point.hpp"
#include "triangle.hpp"
#include "incremental.hpp"
#include "Zone.hpp"

constexpr double HEIGHT = 900.0;
constexpr double WIDTH = 1600.0;
constexpr uint8_t gNumberOfThreads = 4;

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

    omp_set_num_threads(gNumberOfThreads);

    std::random_device rd;
    std::mt19937 gen_for_double(rd());
    std::uniform_real_distribution<> height_gen(1.0, HEIGHT);
    std::uniform_real_distribution<> width_gen(1.0, WIDTH);

    std::set<Point> sPoints;
    // readDump(sPoints);

    constexpr size_t N = 5000;  // number of points

    for (int i = 0; i < N; ++i)
    {
        double sPointWidth = width_gen(gen_for_double);
        double sPointHeigth = height_gen(gen_for_double);
        sPoints.insert(Point(sPointWidth, sPointHeigth));
    }

    // writeDump(sPoints);

    constexpr size_t sZonesCount = 16;  // number of parallel calculated triangulation 'bricks'
    std::array<Zone, sZonesCount> sZones;
    size_t sIndex = 0;
    for (const auto& sPoint: sPoints)
    {
        if (WIDTH / sZonesCount * (sIndex + 1) < sPoint.x)
            sIndex++;

        sZones[sIndex].emplace(sPoint);
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    #pragma omp parallel for
    for (auto& sZone: sZones)
    {
        sZone.triangulate();
    }
    
    size_t sCurrentCounter = 1;
    for (size_t i = 0; i < std::log2(sZonesCount); ++i)
    {
        std::cout << "ITER " << i << std::endl;
        #pragma omp parallel for
        for (size_t j = 0; j < sZonesCount; j += sCurrentCounter * 2)
        {
            sZones[j] |= sZones[j + sCurrentCounter];
        }
        sCurrentCounter *= 2;
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[µs]" << std::endl;
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;

    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Awesome Delaunay Parallel Triangulation");
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        sf::Color darkGreyGreen(10, 30, 10, 255);
        window.clear(darkGreyGreen);

        for (const auto& sEdge: sZones[0].edges) 
        {
            sf::Vertex line[] =
            {
                    sf::Vertex(sf::Vector2f(sEdge.p1.x, sEdge.p1.y)),
                    sf::Vertex(sf::Vector2f(sEdge.p2.x, sEdge.p2.y)),
            };
            window.draw(line, 2, sf::Lines);
        }

        window.display();
    }

    return 0;
}
