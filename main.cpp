#include <iostream>
#include <random>
#include <SFML/Graphics.hpp>
#include <fstream>
#include <omp.h>
#include <chrono>
#include <Eigen/Dense>

#include "edge.hpp"
#include "triangle.hpp"
#include "incremental.hpp"
#include "Zone.hpp"
#include "adf.hpp"

void writeDump(const std::set<Eigen::Vector2f, vecCompare>& aPoints)
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

void readDump(std::set<Eigen::Vector2f, vecCompare>& aPoints)
{
    std::ifstream sInputFile;
    sInputFile.open("dump.txt");
    
    double sX, sY;
    while (sInputFile >> sX >> sY)
    {
        aPoints.insert(Eigen::Vector2f(sX, sY));
    }

    sInputFile.close();    
}




int main() {

    omp_set_num_threads(gNumberOfThreads);

    std::random_device rd;
    std::mt19937 gen_for_double(rd());
    std::uniform_real_distribution<> height_gen(0.11, 0.89);
    std::uniform_real_distribution<> width_gen(0.11, 0.89);

    std::set<Eigen::Vector2f, vecCompare> sPoints;
    // readDump(sPoints);

    for (size_t i = 0; i < N; ++i)
    {
        double sPointWidth = width_gen(gen_for_double);
        double sPointHeigth = height_gen(gen_for_double);
        sPoints.insert(Eigen::Vector2f(sPointWidth, sPointHeigth));
    }
    writeDump(sPoints);

    std::array<Zone, sZonesCount> sZones;
    size_t sIndex = 0;
    for (const auto& sPoint: sPoints)
    {
        if (1.0 / sZonesCount * (sIndex + 1) < sPoint.x())
            sIndex++;

        sZones[sIndex].emplace(sPoint);
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    #pragma omp parallel for
    for (auto& sZone: sZones)
    {
        sZone.triangulate();
    }
    
    std::vector<Zone> sMergeZones;
    #pragma omp parallel for
    for (size_t j = 0; j < sZonesCount; j += 2)
    {
        std::cout << "merging " << j << " |= " << j + 1 << " zones" << std::endl;
        sMergeZones.emplace_back(sZones[j] |= sZones[j + 1]);
    }
    #pragma omp parallel for
    for (size_t j = 1; j < sZonesCount-1; j += 2)
    {
        std::cout << "merging " << j << " |= " << j + 1 << " zones" << std::endl;
        sMergeZones.emplace_back(sZones[j] |= sZones[j + 1]);
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[µs]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;


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

        for (const auto& sZone: sZones)
        {
            for (const auto& sEdge: sZone.edges) 
            {
                sf::Vertex line[] =
                {
                        sf::Vertex(sf::Vector2f(sEdge.p1.x()*WIDTH, sEdge.p1.y()*HEIGHT)),
                        sf::Vertex(sf::Vector2f(sEdge.p2.x()*WIDTH, sEdge.p2.y()*HEIGHT)),
                };
                window.draw(line, 2, sf::Lines);
            }
        } 

        for (const auto& sZone: sMergeZones)
        {
            for (const auto& sEdge: sZone.edges) 
            {
                sf::Vertex line[] =
                {
                        sf::Vertex(sf::Vector2f(sEdge.p1.x()*WIDTH, sEdge.p1.y()*HEIGHT)),
                        sf::Vertex(sf::Vector2f(sEdge.p2.x()*WIDTH, sEdge.p2.y()*HEIGHT)),
                };
                window.draw(line, 2, sf::Lines);
            }
        } 


        // for (const auto& sEdge: sEdges) 
        // {
        //     sf::Vertex line[] =
        //     {
        //             sf::Vertex(sf::Vector2f(sEdge.p1.x(), sEdge.p1.y()), sf::Color::Green),
        //             sf::Vertex(sf::Vector2f(sEdge.p2.x(), sEdge.p2.y()), sf::Color::Green),
        //     };
        //     window.draw(line, 2, sf::Lines);
        // }

        // for (const auto& sEdge: sFront) 
        // {
        //     sf::Vertex line[] =
        //     {
        //             sf::Vertex(sf::Vector2f(sEdge.p1.x(), sEdge.p1.y()), sf::Color::Red),
        //             sf::Vertex(sf::Vector2f(sEdge.p2.x(), sEdge.p2.y()), sf::Color::Red),
        //     };
        //     window.draw(line, 2, sf::Lines);
        // }

        // for (auto p: sPoints)
        // {
        //     sf::Vertex point(sf::Vector2f(p.x(), p.y()), sf::Color::Green);
        //     window.draw(&point, 1, sf::Points);
        // }

        window.display();
    }

    return 0;
}
