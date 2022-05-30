#include <iostream>
#include <random>
#include <SFML/Graphics.hpp>
#include <fstream>
#include <omp.h>
#include <chrono>

#include "hull.hpp"
#include "Point3D.hpp"
#include "adf.hpp"
#include "triangulation3D.hpp"

constexpr double HEIGHT = 1.0;
constexpr double WIDTH = 1.0;
constexpr double LENGTH = 1.0;
constexpr uint8_t gNumberOfThreads = 4;

void writeDump(const std::set<Point3D>& aPoints)
{
    std::string filename("dump.txt");
    std::fstream output_fstream;

    output_fstream.open(filename, std::ios_base::out);
    if (!output_fstream.is_open()) {
        std::cout << "Failed to open " << filename << '\n';
    } else {
        for (const auto& sPoint: aPoints)
        {
            output_fstream << sPoint.x << " " << sPoint.y << " " << sPoint.z << std::endl;
        }
        std::cout << "Done Writing!" << std::endl;
    }
}

void readDump(std::set<Point3D>& aPoints)
{
    std::ifstream sInputFile;
    sInputFile.open("dump.txt");
    
    double sX, sY, sZ;
    while (sInputFile >> sX >> sY >> sZ)
    {
        aPoints.insert(Point3D(sX, sY, sZ));
    }

    sInputFile.close();    
}

struct vecCompare {
    bool operator() (Eigen::Vector3f v, Eigen::Vector3f w) const {
        Eigen::Vector3f b(v.x()-w.x(), v.y()-w.y(), v.z()-w.z());
        Eigen::Vector3f zero(0.0, 0.0, 0.0);
        if (b.isApprox(zero, 1e-7))
            return false;
        for (int i = 0; i < 3; ++i) {
            if (v(i) < w(i)) return true;
            if (v(i) > w(i)) return false;
        }

        return false;
    }
};

int main() {

    omp_set_num_threads(gNumberOfThreads);

    std::random_device rd;
    std::mt19937 gen_for_double(rd());
    std::uniform_real_distribution<> height_gen(0.0, HEIGHT);
    std::uniform_real_distribution<> width_gen(0.0, WIDTH);
    std::uniform_real_distribution<> length_gen(0.0, LENGTH);

    std::set<Point3D> sPoints;
    // readDump(sPoints);

    constexpr size_t N = 50;  // number of points

    for (size_t i = 0; i < N; ++i)
    {
        double sPointWidth = width_gen(gen_for_double);
        double sPointHeigth = height_gen(gen_for_double);
        double sPointLength = length_gen(gen_for_double);
        sPoints.insert(Point3D(sPointWidth, sPointHeigth, sPointLength));
    }
    writeDump(sPoints);

    // sPoints.insert(Point3D(0.5, 0.5, 0.5));
    // sPoints.insert(Point3D(0, 0, 0));
    // sPoints.insert(Point3D(0, 0.5, 0.5));
    // sPoints.insert(Point3D(0.5, 0, 0.5));
    // sPoints.insert(Point3D(0.5, 0.5, 0));
    // sPoints.insert(Point3D(0.5, 0, 0));
    // sPoints.insert(Point3D(0, 0.5, 0));
    // sPoints.insert(Point3D(0, 0, 0.5));


    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    ADF sADF(sPoints);
    sADF.fillFront();
    // sADF.splitFront();
    auto sFaceBufs = sADF.triangulate();


    std::set<Eigen::Vector3f, vecCompare> vec;
    for (auto sFace : sFaceBufs)
    {
        vec.emplace(sFace.vertices[0].x, sFace.vertices[0].y, sFace.vertices[0].z);
        vec.emplace(sFace.vertices[1].x, sFace.vertices[1].y, sFace.vertices[1].z);
        vec.emplace(sFace.vertices[2].x, sFace.vertices[2].y, sFace.vertices[2].z);
    }

    std::vector<Eigen::Vector3f> s(vec.begin(), vec.end());

    auto tr = Triangulation::triangulate3D(s, false);

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[µs]" << std::endl;
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;


    std::string x, y, z;
    for (const auto& point: s)
    {
        assert(point(0) < 1.0);
        assert(point(1) < 1.0);
        assert(point(2) < 1.0);
        x += std::to_string(point(0)) + ",";
        y += std::to_string(point(1)) + ",";
        z += std::to_string(point(2)) + ",";
    }
    x.pop_back();
    y.pop_back();
    z.pop_back();

    std::string vertices;
    for (const auto& t: tr)
    {
        vertices += std::to_string(t.V0) + "," + std::to_string(t.V1) + "," + std::to_string(t.V2) + ",";
    }
    vertices.pop_back();
    // std::cout << x << std::endl;
    // std::cout << y << std::endl;
    // std::cout << z << std::endl;
    // std::cout << vertices << std::endl;
    
    std::string call_string = "python3 ../vis3.py " + x + " " + y + " " + z + " " + vertices; 
    // pass x, y, z, vertices_to_connect
    system(call_string.c_str());


    // auto sFaces = sADF.getFaces();
    // auto sFront = sADF.getFront();
    // std::cout << sFront.size() << std::endl;

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[µs]" << std::endl;
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;

    // std::string x_face,y_face,z_face;
    // std::string vertices_face;
    // int i = 0;
    // for (const auto& face: sFaces)
    // {
    //     x_face += std::to_string(face.vertices[0].x) + ",";
    //     x_face += std::to_string(face.vertices[1].x) + ",";
    //     x_face += std::to_string(face.vertices[2].x) + ",";
    //     y_face += std::to_string(face.vertices[0].y) + ",";
    //     y_face += std::to_string(face.vertices[1].y) + ",";
    //     y_face += std::to_string(face.vertices[2].y) + ",";
    //     z_face += std::to_string(face.vertices[0].z) + ",";
    //     z_face += std::to_string(face.vertices[1].z) + ",";
    //     z_face += std::to_string(face.vertices[2].z) + ",";
    //     vertices_face += std::to_string(i) + "," + std::to_string(i+1) + "," + std::to_string(i+2) + ",";
    //     i++;
    //     i++;
    //     i++;
    // }
    // x_face.pop_back();
    // y_face.pop_back();
    // z_face.pop_back();
    // vertices_face.pop_back();

    // std::string x_front,y_front,z_front;
    // std::string vertices_front;
    // i = 0;
    // for (const auto& face: sFront)
    // {
    //     x_front += std::to_string(face.vertices[0].x) + ",";
    //     x_front += std::to_string(face.vertices[1].x) + ",";
    //     x_front += std::to_string(face.vertices[2].x) + ",";
    //     y_front += std::to_string(face.vertices[0].y) + ",";
    //     y_front += std::to_string(face.vertices[1].y) + ",";
    //     y_front += std::to_string(face.vertices[2].y) + ",";
    //     z_front += std::to_string(face.vertices[0].z) + ",";
    //     z_front += std::to_string(face.vertices[1].z) + ",";
    //     z_front += std::to_string(face.vertices[2].z) + ",";
    //     vertices_front += std::to_string(i) + "," + std::to_string(i+1) + "," + std::to_string(i+2) + ",";
    //     i++;
    //     i++;
    //     i++;
    // }
    // x_front.pop_back();
    // y_front.pop_back();
    // z_front.pop_back();
    // vertices_front.pop_back();

    // std::string call_string = "python3 ../vis3.py " + x_face + " " + y_face + " " + z_face + " " + vertices_face;
    //                                                // + x_front + " " + y_front + " " + z_front + " " + vertices_front ; 
    // // pass x, y, z, vertices_to_connect
    // system(call_string.c_str());



    // constexpr size_t sZonesCount = 16;  // number of parallel calculated triangulation 'bricks'
    // std::array<Zone, sZonesCount> sZones;
    // size_t sIndex = 0;
    // for (const auto& sPoint: sPoints)
    // {
    //     if (WIDTH / sZonesCount * (sIndex + 1) < sPoint.x)
    //         sIndex++;

    //     sZones[sIndex].emplace(sPoint);
    // }

    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

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
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[µs]" << std::endl;
    // // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;

    // sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Awesome Delaunay Parallel Triangulation");
    // while (window.isOpen())
    // {
    //     sf::Event event;
    //     while (window.pollEvent(event))
    //     {
    //         if (event.type == sf::Event::Closed)
    //             window.close();
    //     }

    //     sf::Color darkGreyGreen(10, 30, 10, 255);
    //     window.clear(darkGreyGreen);

    //     for (const auto& sEdge: sZones[0].edges) 
    //     {
    //         sf::Vertex line[] =
    //         {
    //                 sf::Vertex(sf::Vector2f(sEdge.p1.x, sEdge.p1.y)),
    //                 sf::Vertex(sf::Vector2f(sEdge.p2.x, sEdge.p2.y)),
    //         };
    //         window.draw(line, 2, sf::Lines);
    //     }

    //     window.display();
    // }

    return 0;
}
