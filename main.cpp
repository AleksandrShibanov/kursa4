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

constexpr double HEIGHT = 900.0;
constexpr double WIDTH = 1600.0;
constexpr uint8_t gNumberOfThreads = 4;
constexpr uint64_t SPLIT = 30;

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
    std::uniform_real_distribution<> height_gen(1.0, HEIGHT);
    std::uniform_real_distribution<> width_gen(1.0, WIDTH);

    std::set<Eigen::Vector2f, vecCompare> sPoints;
    readDump(sPoints);

    // constexpr size_t N = 10;  // number of points

    // for (size_t i = 0; i < N; ++i)
    // {
    //     double sPointWidth = width_gen(gen_for_double);
    //     double sPointHeigth = height_gen(gen_for_double);
    //     sPoints.insert(Eigen::Vector2f(sPointWidth, sPointHeigth));
    // }
    // writeDump(sPoints);

    ADF sADF(sPoints);
    sADF.fillFront();
    sADF.splitFront(SPLIT);
    sADF.triangulate();

    std::vector<Edge> sFront = sADF.getFront();
    std::vector<Edge> sEdges = sADF.getEdges();

    // for (int i = 0; i < 61 && !front.empty(); ++i)
    // {
    //     auto fe = *(front.cend()-1);
    //     front.pop_back();

    //     auto ssPoints = sPoints;
    //     std::sort(std::begin(ssPoints), std::end(ssPoints),
    //     [fe] (const Eigen::Vector2f& p1, const Eigen::Vector2f& p2)
    //     {
    //         Edge e11(fe.p1, p1);
    //         Edge e21(fe.p2, p1);

    //         Edge e12(fe.p1, p2);
    //         Edge e22(fe.p2, p2);

    //         double cross1 = fe.cross(e11);
    //         double cross2 = fe.cross(e12);

    //         return cross1/(e11.length()*e11.length()+e21.length()*e21.length()+fe.length()*fe.length()) < cross2/(e12.length()*e12.length()+e22.length()*e22.length()+fe.length()*fe.length());
    //     });
    //     Edge e1(fe.p1, *ssPoints.cbegin());
    //     Edge e2(fe.p2, *ssPoints.cbegin());

    //     auto erased = std::erase_if(front, [e1](const Edge& e) { return e1==e; });
    //     if (erased == 0)
    //     {
    //         front.insert(front.begin(), e1);
    //     }
    //     else
    //     {
    //         edges.push_back(e1);
    //     }

    //     erased = std::erase_if(front, [e2](const Edge& e) { return e2==e; });
    //     if (erased == 0)
    //     {
    //         front.insert(front.begin(), e2);
    //     }
    //     else
    //     {
    //         edges.push_back(e2);
    //     }

    //     edges.push_back(fe);
        
    //     // std::cout << i << " ITERATION" << std::endl;
    //     // for (auto e: front)
    //     // {
    //     //     std::cout << e << std::endl;
    //     // }

    // }






    // constexpr size_t sZonesCount = 4;  // number of parallel calculated triangulation 'bricks'
    // std::array<Zone, sZonesCount> sZones;
    // size_t sIndex = 0;
    // for (const auto& sPoint: sPoints)
    // {
    //     if (WIDTH / sZonesCount * (sIndex + 1) < sPoint.x())
    //         sIndex++;

    //     sZones[sIndex].emplace(sPoint);
    // }

    //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // #pragma omp parallel for
    // for (auto& sZone: sZones)
    // {
    //     sZone.triangulate();
    // }
    
    // size_t sCurrentCounter = 1;
    // for (size_t i = 0; i < std::log2(sZonesCount); ++i)
    // {
    //     std::cout << "ITER " << i << std::endl;
    //     #pragma omp parallel for
    //     for (size_t j = 0; j < sZonesCount; j += sCurrentCounter * 2)
    //     {
    //         sZones[j] |= sZones[j + sCurrentCounter];
    //     }
    //     sCurrentCounter *= 2;
    // }

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[µs]" << std::endl;
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

        for (const auto& sEdge: sEdges) 
        {
            sf::Vertex line[] =
            {
                    sf::Vertex(sf::Vector2f(sEdge.p1.x(), sEdge.p1.y()), sf::Color::Green),
                    sf::Vertex(sf::Vector2f(sEdge.p2.x(), sEdge.p2.y()), sf::Color::Green),
            };
            window.draw(line, 2, sf::Lines);
        }

        for (const auto& sEdge: sFront) 
        {
            sf::Vertex line[] =
            {
                    sf::Vertex(sf::Vector2f(sEdge.p1.x(), sEdge.p1.y()), sf::Color::Red),
                    sf::Vertex(sf::Vector2f(sEdge.p2.x(), sEdge.p2.y()), sf::Color::Red),
            };
            window.draw(line, 2, sf::Lines);
        }

        for (auto p: sPoints)
        {
            sf::Vertex point(sf::Vector2f(p.x(), p.y()), sf::Color::Green);
            window.draw(&point, 1, sf::Points);
        }

        window.display();
    }

    return 0;
}
