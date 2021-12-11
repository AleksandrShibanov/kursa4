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

double determinant(  Point& a,   Point& b,   Point& c,   Point& d) {
    return (a.x - d.x) * (b.y - d.y) * (std::pow(c.x - d.x,2) + std::pow(c.y - d.y,2)) +
           (b.x - d.x) * (c.y - d.y) * (std::pow(a.x - d.x,2) + std::pow(a.y - d.y,2)) +
           (c.x - d.x) * (a.y - d.y) * (std::pow(b.x - d.x,2) + std::pow(b.y - d.y,2)) -
           (c.x - d.x) * (b.y - d.y) * (std::pow(a.x - d.x,2) + std::pow(a.y - d.y,2)) -
           (a.x - d.x) * (c.y - d.y) * (std::pow(b.x - d.x,2) + std::pow(b.y - d.y,2)) -
           (b.x - d.x) * (a.y - d.y) * (std::pow(c.x - d.x,2) + std::pow(c.y - d.y,2));
}

Point right_best_candidate(std::vector<Edge>& edges, Edge& current_base, std::vector<Triangle>& right_triangulation) {
    std::vector<std::pair<Point, double> > right_degree_vec;
    for (auto& edge: edges) {
        if (edge.hasCommonPoint(current_base)) {
            Point A = current_base.p1;
            Point B = current_base.p2;
            Point C((edge.p1 == current_base.p1) ? edge.p2 : edge.p1);
            Triangle abc(A, B, C);
            right_degree_vec.push_back(std::make_pair(C, abc.ABC()));
        }
    }

    if (right_degree_vec.empty()) {
        return {-1, -1};
    }

    std::sort(right_degree_vec.begin(), right_degree_vec.end(), [](const std::pair<Point, double>& p1, const std::pair<Point, double>& p2) {
        return  p1.second < p2.second;
    });

    auto func = [](const std::pair<Point, double>& p1) {
        return isnanf(p1.second) || p1.second > 180 || p1.second < 0;
    };
    right_degree_vec.erase(std::remove_if(right_degree_vec.begin(), right_degree_vec.end(), func), right_degree_vec.end());

    Point potential_candidate = right_degree_vec[0].first;
    right_degree_vec.erase(right_degree_vec.begin());
    if (right_degree_vec.empty()) {
        return potential_candidate;
    }
    Point next_potential_candidate = right_degree_vec[0].first;
    right_degree_vec.erase(right_degree_vec.begin());
    Point A = current_base.p1;
    Point B = current_base.p2;
    Triangle buf_triangle(A, B, potential_candidate);
    while (buf_triangle.circumscribedCircleContains(next_potential_candidate)) {
        right_triangulation.erase(std::remove_if(right_triangulation.begin(), right_triangulation.end(), [&B, &potential_candidate](Triangle& tr) {
            return tr.containsEdge({B, potential_candidate});
        }), right_triangulation.end());

        potential_candidate = next_potential_candidate;
        if (right_degree_vec.empty()) {
            return potential_candidate;
        }
        next_potential_candidate = right_degree_vec[0].first;
        right_degree_vec.erase(right_degree_vec.begin());
        buf_triangle = Triangle(A, B, potential_candidate);
    }
    return potential_candidate;
}

Point left_best_candidate(std::vector<Edge>& edges, Edge& current_base, std::vector<Triangle>& left_triangulation) {
    std::vector<std::pair<Point, double> > left_degree_vec;
    for (auto& edge: edges) {
        if (edge.hasCommonPoint(current_base)) {
            Point A = current_base.p1;
            Point B = current_base.p2;
            Point C = (edge.p1 == current_base.p1) ? edge.p2 : edge.p1;
            Triangle abc(A, B, C);
            left_degree_vec.push_back(std::make_pair(C, abc.CAB()));
        }
    }



    if (left_degree_vec.empty()) {
        return {-1, -1};
    }

    std::sort(left_degree_vec.begin(), left_degree_vec.end(), [](const std::pair<Point, double>& p1, const std::pair<Point, double>& p2) {
        return  p1.second < p2.second;
    });

    std::cout << "After sort" << std::endl;


    auto func = [](const std::pair<Point, double>& p1) {
        return isnanf(p1.second) || p1.second > 180 || p1.second < 0;
    };
    left_degree_vec.erase(std::remove_if(left_degree_vec.begin(), left_degree_vec.end(), func), left_degree_vec.end());

    Point potential_candidate = left_degree_vec[0].first;
    left_degree_vec.erase(left_degree_vec.begin());

    if (left_degree_vec.empty()) {
        return potential_candidate;
    }

    Point next_potential_candidate = left_degree_vec[0].first;
    left_degree_vec.erase(left_degree_vec.begin());

    Point A = current_base.p1;
    Point B = current_base.p2;
    Triangle buf_triangle(A, B, potential_candidate);
    while (buf_triangle.circumscribedCircleContains(next_potential_candidate)) {
        left_triangulation.erase(std::remove_if(left_triangulation.begin(), left_triangulation.end(), [&A, &potential_candidate](Triangle& tr) {
            return tr.containsEdge({A, potential_candidate});
        }), left_triangulation.end());
        // std::remove(edges.begin(), edges.end(), Edge(A, potential_candidate));

        potential_candidate = next_potential_candidate;
        if (left_degree_vec.empty()) {
            return potential_candidate;
        }

        next_potential_candidate = left_degree_vec[0].first;
        left_degree_vec.erase(left_degree_vec.begin());

        buf_triangle = Triangle(A, B, potential_candidate);
    }
    return potential_candidate;
}

void boundary_points_insert(size_t y_start, size_t y_end, size_t x_start, size_t x_end, std::vector<Point>& boundary_points_top, std::vector<Point>& boundary_points_bottom, std::vector<Point>& boundary_points_left, std::vector<Point>& boundary_points_right, std::vector<std::vector<std::vector<Point> > >& points_in_cell) {
    for (size_t i = y_start; i < y_end; ++i) {
        for (size_t j = x_start; j < x_end; ++j) {
            if (i == y_start) {
                boundary_points_top.insert(boundary_points_top.end(), points_in_cell[i][j].begin(), points_in_cell[i][j].end());
            } else
            if (j == x_start) {
                boundary_points_left.insert(boundary_points_left.end(), points_in_cell[i][j].begin(), points_in_cell[i][j].end());
            } else
            if (i == y_end - 1) {
                boundary_points_bottom.insert(boundary_points_bottom.end(), points_in_cell[i][j].begin(), points_in_cell[i][j].end());
            } else
            if (j == x_end - 1) {
                boundary_points_right.insert(boundary_points_right.end(), points_in_cell[i][j].begin(), points_in_cell[i][j].end());
            }
        }
    }
}

bool check_for_boundary_cross(size_t y_start, size_t y_end, size_t x_start, size_t x_end, std::vector<Triangle>& boundary_triangles, std::vector<std::vector<std::vector<Point> > >& points_in_cell, std::set<Point>& zone_boundary_points, std::set<Point>& boundary_points, std::vector<Point>& bound, std::set<Point>& zone_points, std::vector<Triangle>& new_triangles) {
    bool flag = false;

    zone_boundary_points.insert(boundary_points.begin(), boundary_points.end());
    boundary_points.clear();

    for (size_t i = y_start; i < y_end; ++i) {
        for (auto it = points_in_cell[i][x_start].begin(); it != points_in_cell[i][x_start].end(); ++it) {
            boundary_points.insert(*it);
        }
    }

    for (size_t i = y_start; i < y_end; ++i) {
        for (auto it = points_in_cell[i][x_end - 1].begin(); it != points_in_cell[i][x_end - 1].end(); ++it) {
            boundary_points.insert(*it);
        }
    }

    for (size_t j = x_start; j < x_end; ++j) {
        for (auto it = points_in_cell[y_start][j].begin(); it != points_in_cell[y_start][j].end(); ++it) {
            boundary_points.insert(*it);
        }
    }

    for (size_t j = x_start; j < x_end; ++j) {
        for (auto it = points_in_cell[y_end - 1][j].begin(); it != points_in_cell[y_end - 1][j].end(); ++it) {
            boundary_points.insert(*it);
        }
    }

    for (auto it = boundary_points.begin(); it != boundary_points.end(); ++it) {
        for (Triangle &tr: boundary_triangles) {
            if (tr.circumscribedCircleContains(*it)) {
                flag = true;
                bound.push_back(*it);
                break;
            }
        }
    }

    auto inc1 = Incremental(bound);
    new_triangles = inc1.triangulate();


    auto func1 = std::remove_if(new_triangles.begin(),
                                new_triangles.end(),
                                [&boundary_points](Triangle &tr) {
                                    return boundary_points.contains(tr.A) && boundary_points.contains(tr.B) &&
                                           boundary_points.contains(tr.C);
                                });
    new_triangles.erase(func1, new_triangles.end());

    std::copy_if(new_triangles.begin(),
                 new_triangles.end(),
                 std::back_inserter(boundary_triangles),
                 [&zone_points, &boundary_points](Triangle &tr) {
                     return !(zone_points.contains(tr.A) && zone_points.contains(tr.B) &&
                              zone_points.contains(tr.C)) &&
                            !(boundary_points.contains(tr.A) && boundary_points.contains(tr.B) &&
                              boundary_points.contains(tr.C));
                 });
    return flag;
}

class PointCmp {
public:
    bool operator()(  Point& p1,   Point& p2)   {
        return (std::fabs(p1.x - p2.x) < std::numeric_limits<double>::epsilon()) ? p1.y > p2.y : p1.x < p2.x;
    }
    bool operator()(  std::pair<Point, Point>& p)   {
        return (std::fabs(p.first.x - p.second.x) < std::numeric_limits<double>::epsilon()) ? p.first.y > p.second.y : p.first.x < p.second.x;
    }
    bool operator()(  std::pair<Point, Point>& p1,   std::pair<Point, Point>& p2)   {
        return std::hypot(p1.second.x - p1.first.x, p1.second.y - p1.first.y) < std::hypot(p2.second.x - p2.first.x, p2.second.y - p2.first.y);
    }
//    bool operator()(  Point& p1,   Point& p2)   {
//        return (p1.y / p1.x) > (p2.y / p1.x);
//    }
};





int main() {
    std::random_device rd;
    std::mt19937 gen_for_int(rd());
    std::mt19937 gen_for_double(rd());
    std::uniform_int_distribution<> int_gen(10, 100);
    std::uniform_real_distribution<> height_gen(1.0, 900.0);
    std::uniform_real_distribution<> width_gen(1.0, 1600.0);

    constexpr size_t N = 500;  // number of points
    constexpr size_t n = 4;  // average number of points in cell

    std::vector<Point> points;
    points.reserve(N);
    while (points.size() != N) {
        Point p(width_gen(gen_for_double), height_gen(gen_for_double));
        points.push_back(p);
    }

    /*points.push_back({100, 100});
    points.push_back({150, 150});
    points.push_back({200, 100});
    points.push_back({150, 50});
    points.push_back({150, 25});*/

    /*
    points1.push_back({390, 150});
    points1.push_back({390, 100});
    points1.push_back({390, 25});
    points1.push_back({230, 25});
    points1.push_back({350, 75});*/

    std::vector<std::vector<Triangle> > triangles;
    Incremental inc(points);
    triangles.push_back(inc.triangulate());
    /*
    Incremental inc1(points1);
    triangles.push_back(inc1.triangulate());
*/

    /*auto mmx = std::minmax_element(points.begin(), points.end(), [](  Point& p1,   Point& p2) {
        return p1.x < p2.x;
    });

    auto mmy = std::minmax_element(points.begin(), points.end(), [](  Point& p1,   Point& p2) {
        return p1.y < p2.y;
    });

    double min_x = mmx.first->x;
    double max_x = mmx.second->x;
    double min_y = mmy.first->y;
    double max_y = mmy.second->y;

   std::cout << "min_x: " << min_x << "\nmax_x: " << max_x << "\nmin_y: " << min_y << "\nmax_y: " << max_y << std::endl;

    double R_x = max_x - min_x;  // x range
    double R_y = max_y - min_y;  // y range

    double lambda = std::sqrt(static_cast<double>(N) / (static_cast<double>(n) * R_x * R_y));

    double n_x = lambda * R_x;  // number of cell divisions along the x-axis
    double n_y = lambda * R_y;  // number of cell divisions along the y-axis

    size_t N_x = std::ceil(n_x);
    size_t N_y = std::ceil(n_y);

    std::cout << "Ammount of cells in accurate: x:" << n_x << " y: " << n_y << std::endl;
    std::cout << "Rounded ammount of cells: x: "  << N_x << " y: " << N_y << std::endl;

    double step_x = R_x / n_x;
    double step_y = R_y / n_y;

    std::vector<std::vector<std::vector<Point> > > points_in_cell;
    points_in_cell.resize(N_y);
    for (size_t i = 0; i < N_y - 1; ++i) {
        points_in_cell[i].resize(N_x);
    }
    points_in_cell[N_y - 1].resize(N - N_x * (N_y - 1));

    for (auto it = points.begin(); it != points.end(); ++it) {
        Point p(it->x, it->y);
        points_in_cell[std::floor(it->y / step_y)][std::floor(it->x / step_x)].push_back(p);
    }

    size_t check_counter = 0;
    for (auto y_it = points_in_cell.begin(); y_it != points_in_cell.end(); ++y_it) {
        for (auto x_it = y_it->begin(); x_it != y_it->end(); ++x_it) {
            check_counter += x_it->size();
        }
    }

    std::cout << "Check ammount of points: " << check_counter << std::endl;

      size_t Z_x = 3;  // zonal division along the x-axis
      size_t Z_y = 3;  // zonal division along the y-axis

    auto N_z = static_cast<size_t>(std::round(static_cast<double>(N_x)/static_cast<double>(Z_x)) * std::round(static_cast<double>(N_y)/static_cast<double>(Z_y)));  // number of cells in each zone
    std::cout << "Rounded ammount of cells in each zone: " << N_z << std::endl;



    std::vector<std::vector<std::vector<Point> > > points_in_zone;
    points_in_zone.resize(Z_y);
    for (auto it = points_in_zone.begin(); it != points_in_zone.end(); ++it) {
        it->resize(Z_x);
    }

    for (size_t i = 0; i < N_y; ++i) {
        size_t y_index = i / std::ceil(static_cast<double>(N_y) / static_cast<double>(Z_y));
        for (size_t j = 0; j < N_x; ++j) {
            size_t x_index = j / std::ceil(static_cast<double>(N_x) / static_cast<double>(Z_x));
            for (auto it = points_in_cell[i][j].begin(); it != points_in_cell[i][j].end(); ++it) {
                points_in_zone[y_index][x_index].push_back(*it);
            }
        }
    }

    check_counter = 0;
    for (auto y_it = points_in_zone.begin(); y_it != points_in_zone.end(); ++y_it) {
        for (auto x_it = y_it->begin(); x_it != y_it->end(); ++x_it) {
            check_counter += x_it->size();
        }
    }

    std::cout << "Check ammount of points: " << check_counter << std::endl;


    std::vector<std::shared_future<std::vector<Triangle> > > triangles;
    triangles.resize(Z_y * Z_x);
    size_t counter = 0;
    const size_t threads_num = 3;
    uint8_t used_threads = 1;
    for (auto y = points_in_zone.begin(); y != points_in_zone.end(); ++y) {
        for (auto x = y->begin(); x != y->end(); ++x) {

            std::vector<Point> tmp_points;
            for (auto cpy = x->begin(); cpy != x->end(); ++cpy) {
                Point p(cpy->x, cpy->y);
                tmp_points.push_back(p);
            }
            if (tmp_points.empty()) {
                continue;
            }

            Incremental inc(tmp_points);
            if (used_threads == threads_num) {
                triangles[counter - threads_num + 1].wait();
                --used_threads;
            }
            triangles[counter] = std::async(static_cast<std::vector<Triangle>(Incremental::*)()>(&Incremental::triangulate), inc);
            ++counter;
            ++used_threads;
        }
    }*/
/*

    std::vector<Triangle> left_triangulation;
    std::set<Point> left_triangulation_points_set;
    for (Triangle tr: triangles[0]) {
        left_triangulation.push_back(tr);
    }
    std::vector<Triangle> right_triangulation;
    std::set<Point> right_triangulation_points_set;
    for (Triangle tr: triangles[1]) {
        right_triangulation.push_back(tr);
    }

    */
/*size_t x_start = std::ceil(static_cast<double>(N_x) / static_cast<double>(Z_x));
    size_t x_end = 2 * std::ceil(static_cast<double>(N_x) / static_cast<double>(Z_x));
    size_t y_start = std::ceil(static_cast<double>(N_y) / static_cast<double>(Z_y));
    size_t y_end = 2 * std::ceil(static_cast<double>(N_y) / static_cast<double>(Z_y));

    for (size_t i = 0; i < y_start; ++i) {
        for (auto it = points_in_cell[i][x_start - 1].begin(); it != points_in_cell[i][x_start - 1].end(); ++it) {
            right_triangulation_points_set.insert(*it);
        }
    }

    for (size_t i = 0; i < y_start; ++i) {
        for (auto it = points_in_cell[i][x_start].begin(); it != points_in_cell[i][x_start].end(); ++it) {
            left_triangulation_points_set.insert(*it);
        }

    }*//*




    auto y_compare = [](const Point& p1, const Point& p2) {
        if (std::fabs(p1.y - p2.y) < std::numeric_limits<double>::epsilon()) {
            return p1.x > p2.x;
        }
        return p1.y > p2.y;
    };

//    std::vector left_triangulation_points(left_triangulation_points_set.begin(), left_triangulation_points_set.end());
//    std::vector right_triangulation_points(right_triangulation_points_set.begin(), right_triangulation_points_set.end());
    std::vector<Point> left_triangulation_points = points;
    std::vector<Point> right_triangulation_points = points1;

    std::sort(left_triangulation_points.begin(), left_triangulation_points.end(), y_compare);
    std::sort(right_triangulation_points.begin(), right_triangulation_points.end(), y_compare);

    std::vector<Edge> bases;
    Edge current_base = {left_triangulation_points[0], right_triangulation_points[0]};
    bases.push_back(current_base);
    left_triangulation_points.erase(left_triangulation_points.begin());
    right_triangulation_points.erase(right_triangulation_points.begin());

    std::vector<Triangle> boundary_triangles;

    std::vector<Edge> right_edges;
    for (auto& tr: right_triangulation) {
        Edge edge = tr.edges[0];
        if (edge.p1.x > edge.p2.x) {
            auto buf = edge.p2;
            edge.p2 = edge.p1;
            edge.p1 = buf;
        } else if (std::fabs(edge.p1.x - edge.p2.x) < std::numeric_limits<double>::epsilon() && edge.p1.y > edge.p2.y) {
            auto buf = edge.p2;
            edge.p2 = edge.p1;
            edge.p1 = buf;
        }
        right_edges.push_back(edge);
        edge = tr.edges[1];
        if (edge.p1.x > edge.p2.x) {
            auto buf = edge.p2;
            edge.p2 = edge.p1;
            edge.p1 = buf;
        } else if (std::fabs(edge.p1.x - edge.p2.x) < std::numeric_limits<double>::epsilon() && edge.p1.y > edge.p2.y) {
            auto buf = edge.p2;
            edge.p2 = edge.p1;
            edge.p1 = buf;
        }
        right_edges.push_back(edge);
        edge = tr.edges[2];
        if (edge.p1.x > edge.p2.x) {
            auto buf = edge.p2;
            edge.p2 = edge.p1;
            edge.p1 = buf;
        } else if (std::fabs(edge.p1.x - edge.p2.x) < std::numeric_limits<double>::epsilon() && edge.p1.y > edge.p2.y) {
            auto buf = edge.p2;
            edge.p2 = edge.p1;
            edge.p1 = buf;
        }
        right_edges.push_back(edge);
    }
    std::sort(right_edges.begin(), right_edges.end());
    std::unique(right_edges.begin(), right_edges.end());

    std::set<Edge> left_edges_set;
    for (auto& tr: left_triangulation) {
        left_edges_set.insert(tr.edges[0]);
        left_edges_set.insert(tr.edges[1]);
        left_edges_set.insert(tr.edges[2]);
    }

    std::vector<Edge> left_edges(left_edges_set.begin(), left_edges_set.end());

    Point rbc(0, 0);
    Point lbc(0, 0);
    int i = 0;
    while (i < 1) {
        ++i;
        rbc = right_best_candidate(right_edges, current_base, right_triangulation);
        lbc = left_best_candidate(left_edges, current_base, left_triangulation);
        if (lbc == Point(-1, -1) && rbc == Point(-1, -1)) {
            break;
        }

        if (rbc == Point(-1, -1) && lbc != Point(-1, -1)) {
            boundary_triangles.push_back(Triangle(current_base.p1, current_base.p2, lbc));
            current_base = Edge(lbc, current_base.p2);
            continue;
        }

        if (lbc == Point(-1, -1) && rbc != Point(-1, -1)) {
            boundary_triangles.push_back(Triangle(current_base.p1, current_base.p2, rbc));
            current_base = Edge(rbc, current_base.p1);
            continue;
        }

        Triangle buf_triangle(current_base.p1, current_base.p2, rbc);
        if (buf_triangle.containsPoint(lbc)) {
            boundary_triangles.push_back(Triangle(current_base.p1, current_base.p2, lbc));
            current_base = Edge(lbc, current_base.p2);
        } else {
            boundary_triangles.push_back(buf_triangle);
            current_base = Edge(rbc, current_base.p1);
        }
    }








*/


    /*std::set<Point> zone_boundary_points;
    std::set<Point> boundary_points;
    std::set<Point> zone_points(points_in_zone[1][1].begin(), points_in_zone[1][1].end());

    size_t x_start = std::ceil(static_cast<double>(N_x) / static_cast<double>(Z_x));
    size_t x_end = 2 * std::ceil(static_cast<double>(N_x) / static_cast<double>(Z_x));
    size_t y_start = std::ceil(static_cast<double>(N_y) / static_cast<double>(Z_y));
    size_t y_end = 2 * std::ceil(static_cast<double>(N_y) / static_cast<double>(Z_y));

    for (size_t i = y_start; i < y_end; ++i) {
        for (auto it = points_in_cell[i][x_start].begin(); it != points_in_cell[i][x_start].end(); ++it) {
            zone_boundary_points.insert(*it);
        }
    }

    for (size_t i = y_start; i < y_end; ++i) {
        for (auto it = points_in_cell[i][x_end - 1].begin(); it != points_in_cell[i][x_end - 1].end(); ++it) {
            zone_boundary_points.insert(*it);
        }
    }

    for (size_t j = x_start; j < x_end; ++j) {
        for (auto it = points_in_cell[y_start][j].begin(); it != points_in_cell[y_start][j].end(); ++it) {
            zone_boundary_points.insert(*it);
        }
    }

    for (size_t j = x_start; j < x_end; ++j) {
        for (auto it = points_in_cell[y_end - 1][j].begin(); it != points_in_cell[y_end - 1][j].end(); ++it) {
            zone_boundary_points.insert(*it);
        }
    }

    size_t buf_x_start = x_start - 1;
    size_t buf_x_end = x_end + 1;
    size_t buf_y_start = y_start - 1;
    size_t buf_y_end = y_end + 1;

    for (size_t i = buf_y_start; i < buf_y_end; ++i) {
        for (auto it = points_in_cell[i][buf_x_start].begin(); it != points_in_cell[i][buf_x_start].end(); ++it) {
            boundary_points.insert(*it);
        }
    }

    for (size_t i = buf_y_start; i < buf_y_end; ++i) {
        for (auto it = points_in_cell[i][buf_x_end - 1].begin(); it != points_in_cell[i][buf_x_end - 1].end(); ++it) {
            boundary_points.insert(*it);
        }
    }

    for (size_t j = buf_x_start; j < buf_x_end; ++j) {
        for (auto it = points_in_cell[buf_y_start][j].begin(); it != points_in_cell[buf_y_start][j].end(); ++it) {
            boundary_points.insert(*it);
        }
    }

    for (size_t j = buf_x_start; j < buf_x_end; ++j) {
        for (auto it = points_in_cell[buf_y_start - 1][j].begin(); it != points_in_cell[buf_y_start - 1][j].end(); ++it) {
            boundary_points.insert(*it);
        }
    }

    std::vector<Point> bound;
    for (auto it = zone_points.begin(); it != zone_points.end(); ++it) {
        bound.push_back(*it);
    }
    for (auto it = boundary_points.begin(); it != boundary_points.end(); ++it) {
        bound.push_back(*it);
    }
    auto inc = Incremental(bound);
    std::vector<Triangle> middle_triangles = inc.triangulate();

    auto func = std::remove_if(middle_triangles.begin(),
                                   middle_triangles.end(),
                                   [&boundary_points](Triangle& tr) {
                                       return boundary_points.contains(tr.A) && boundary_points.contains(tr.B) && boundary_points.contains(tr.C);
                                   });

    middle_triangles.erase(func, middle_triangles.end());


    std::vector<Triangle> boundary_triangles;
    std::copy_if(middle_triangles.begin(),
                 middle_triangles.end(),
                 std::back_inserter(boundary_triangles),
                                   [&zone_points, &boundary_points](Triangle& tr) {
                                       return !(zone_points.contains(tr.A) && zone_points.contains(tr.B) && zone_points.contains(tr.C)) &&
                                              !(boundary_points.contains(tr.A) && boundary_points.contains(tr.B) && boundary_points.contains(tr.C));
                                   });


    buf_x_start -= 1;
    buf_x_end += 1;
    buf_y_start -= 1;
    buf_y_end += 1;

    while (check_for_boundary_cross(buf_y_start, buf_y_end, buf_x_start, buf_x_end, boundary_triangles, points_in_cell, zone_boundary_points, boundary_points, bound, zone_points, middle_triangles) && buf_y_start != -1 && buf_x_end != N_x && buf_x_start != -1 && buf_y_end != N_y) {

        buf_x_start -= 1;
        buf_x_end += 1;
        buf_y_start -= 1;
        buf_y_end += 1;

    }

    zone_points.clear();
    boundary_points.clear();
    zone_boundary_points.clear();



    zone_points.insert(points_in_zone[0][0].begin(), points_in_zone[0][0].end());

    for (size_t i = 0; i < y_start; ++i) {
        for (auto it = points_in_cell[i][x_start - 1].begin(); it != points_in_cell[i][x_start - 1].end(); ++it) {
            zone_boundary_points.insert(*it);
        }
    }

    for (size_t j = 0; j < x_start; ++j) {
        for (auto it = points_in_cell[y_start - 1][j].begin(); it != points_in_cell[y_start - 1][j].end(); ++it) {
            zone_boundary_points.insert(*it);
        }
    }

    buf_x_start = 0;
    buf_y_start = 0;
    buf_x_end = x_start + 1;
    buf_y_end = y_start + 1;

    for (size_t i = 0; i < buf_y_end; ++i) {
        for (auto it = points_in_cell[i][buf_x_end - 1].begin(); it != points_in_cell[i][buf_x_end - 1].end(); ++it) {
            boundary_points.insert(*it);
        }
    }

    for (size_t j = 0; j < buf_x_end; ++j) {
        for (auto it = points_in_cell[buf_y_end - 1][j].begin(); it != points_in_cell[buf_y_end - 1][j].end(); ++it) {
            boundary_points.insert(*it);
        }
    }

    bound.clear();
    for (auto it = zone_points.begin(); it != zone_points.end(); ++it) {
        bound.push_back(*it);
    }
    for (auto it = boundary_points.begin(); it != boundary_points.end(); ++it) {
        bound.push_back(*it);
    }
    inc = Incremental(bound);
    std::vector<Triangle> left_triangles = inc.triangulate();

    func = std::remove_if(left_triangles.begin(),
                          left_triangles.end(),
                               [&boundary_points](Triangle& tr) {
                                   return boundary_points.contains(tr.A) && boundary_points.contains(tr.B) && boundary_points.contains(tr.C);
                               });

    left_triangles.erase(func, left_triangles.end());

    buf_x_end += 1;
    buf_y_end += 1;

    while (check_for_boundary_cross(buf_y_start, buf_y_end, buf_x_start, buf_x_end, boundary_triangles, points_in_cell, zone_boundary_points, boundary_points, bound, zone_points, left_triangles) && buf_y_end != N_y && buf_x_end != N_x) {

        buf_x_end += 1;
        buf_y_end += 1;

    }*/


    /*x_start -= 1;
    x_end += 1;
    y_start -= 1;
    y_end += 1;

    zone_boundary_points.insert(boundary_points.begin(), boundary_points.end());

    for (size_t i = y_start; i < y_end; ++i) {
        for (auto it = points_in_cell[i][x_start].begin(); it != points_in_cell[i][x_start].end(); ++it) {
            boundary_points.insert(*it);
        }
    }

    for (size_t i = y_start; i < y_end; ++i) {
        for (auto it = points_in_cell[i][x_end - 1].begin(); it != points_in_cell[i][x_end - 1].end(); ++it) {
            boundary_points.insert(*it);
        }
    }

    for (size_t j = x_start; j < x_end; ++j) {
        for (auto it = points_in_cell[y_start][j].begin(); it != points_in_cell[y_start][j].end(); ++it) {
            boundary_points.insert(*it);
        }
    }

    for (size_t j = x_start; j < x_end; ++j) {
        for (auto it = points_in_cell[y_end - 1][j].begin(); it != points_in_cell[y_end - 1][j].end(); ++it) {
            boundary_points.insert(*it);
        }
    }

    std::set<Point> already_added;
    for (auto it = boundary_points.begin(); it != boundary_points.end(); ++it) {
        for (Triangle &tr: boundary_triangles) {
            if (tr.circumscribedCircleContains(*it) && !already_added.contains(*it)) {
                bound.push_back(*it);
                already_added.insert(*it);
            }
        }
    }

    auto inc1 = Incremental(bound);
    std::vector<Triangle> new_middle_triangles = inc1.triangulate();

    std::cout << "size: " << new_middle_triangles.size() << std::endl;

    auto func1 = std::remove_if(new_middle_triangles.begin(),
                               new_middle_triangles.end(),
                               [&boundary_points](Triangle& tr) {
                                   return boundary_points.contains(tr.A) && boundary_points.contains(tr.B) && boundary_points.contains(tr.C);
                               });
    new_middle_triangles.erase(func1, new_middle_triangles.end());

    std::cout << "size: " << new_middle_triangles.size() << std::endl;


    std::copy_if(new_middle_triangles.begin(),
                 new_middle_triangles.end(),
                 std::back_inserter(boundary_triangles),
                 [&zone_points, &boundary_points](Triangle& tr) {
                     return !(zone_points.contains(tr.A) && zone_points.contains(tr.B) && zone_points.contains(tr.C)) &&
                            !(boundary_points.contains(tr.A) && boundary_points.contains(tr.B) && boundary_points.contains(tr.C));
                 });*/


    /*for (size_t i = y_start - 1; i != y_start + 1; ++i) {
        for (size_t j = x_start; j != x_end; ++j) {
            boundary_points.insert(points_in_cell[i][j].begin(), points_in_cell[i][j].end());
        }
    }

    for (size_t i = y_start; i != y_end; ++i) {
        for (size_t j = x_start - 1; j != x_start + 1; ++j) {
            boundary_points.insert(points_in_cell[i][j].begin(), points_in_cell[i][j].end());
        }
    }

    for (size_t i = y_end - 1; i != y_end + 1; ++i) {
        for (size_t j = x_start; j != x_end; ++j) {
            boundary_points.insert(points_in_cell[i][j].begin(), points_in_cell[i][j].end());
        }
    }

    for (size_t i = y_start; i != y_end; ++i) {
        for (size_t j = x_end - 1; j != x_end + 1; ++j) {
            boundary_points.insert(points_in_cell[i][j].begin(), points_in_cell[i][j].end());
        }
    }

    std::vector<Point> vec(1000);
    auto setd = std::set_difference(boundary_points.begin(), boundary_points.end(), zone_points.begin(), zone_points.end(), vec.begin());
    vec.resize(setd - vec.begin());
    std::set<Point> only_boundary(vec.begin(), vec.end());


    std::vector<Point> bound(only_boundary.begin(), only_boundary.end());
    for (auto it = triangles[4].get().begin(); it != triangles[4].get().end(); ++it) {
        bound.push_back(it->A);
        bound.push_back(it->B);
        bound.push_back(it->C);
    }
    auto inc = Incremental(bound);
    std::vector<Triangle> boundary_triangles = inc.triangulate();*/


//    auto func = std::remove_if(boundary_triangles.begin(),
//                               boundary_triangles.end(),
//                                   [&zone_points, &only_boundary](Triangle& tr) {
//                                       return (zone_points.contains(tr.A) && zone_points.contains(tr.B) && zone_points.contains(tr.C)) ||
//                                              (only_boundary.contains(tr.A) && only_boundary.contains(tr.B) && only_boundary.contains(tr.C));
//                                   });
//
//    boundary_triangles.erase(func, boundary_triangles.end());

    /*size_t buf_y_start = y_start - 1;
    while (check_for_boundary_cross(buf_y_start - 1, y_start, x_start, x_end, boundary_triangles, points_in_cell, bound)) {
        if (buf_y_start - 1 == 0) {
            break;
        }
        buf_y_start -= 1;
        boundary_triangles = Incremental(bound).triangulate();
    }


    size_t buf_x_start = x_start - 1;
    while (check_for_boundary_cross(y_start, y_end, buf_x_start - 1, buf_x_start, boundary_triangles, points_in_cell, bound)) {
        if (buf_x_start - 1 == 0) {
            break;
        }
        buf_x_start -= 1;
        boundary_triangles = Incremental(bound).triangulate();
    }

    size_t buf_y_end = y_end + 1;
    while (check_for_boundary_cross(buf_y_end, buf_y_end + 1, x_start, x_end, boundary_triangles, points_in_cell, bound)) {
        if (buf_y_end == N_y - 1) {
            break;
        }
        buf_y_end += 1;
        boundary_triangles = Incremental(bound).triangulate();
    }

    size_t buf_x_end = x_end + 1;
    while (check_for_boundary_cross(y_start, y_end, buf_x_end, buf_x_end + 1, boundary_triangles, points_in_cell, bound)) {
        if (buf_x_end == N_x - 1) {
            break;
        }
        buf_x_end += 1;
        boundary_triangles = Incremental(bound).triangulate();
    }*/








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

        /*for (auto it = left_triangulation.begin(); it != left_triangulation.end(); ++it) {
            sf::Vertex AB[] =
                    {
                            sf::Vertex(sf::Vector2f(it->A.x, it->A.y)),
                            sf::Vertex(sf::Vector2f(it->B.x, it->B.y))
                    };
            sf::Vertex BC[] =
                    {
                            sf::Vertex(sf::Vector2f(it->B.x, it->B.y)),
                            sf::Vertex(sf::Vector2f(it->C.x, it->C.y)),
                    };
            sf::Vertex AC[] =
                    {
                            sf::Vertex(sf::Vector2f(it->A.x, it->A.y)),
                            sf::Vertex(sf::Vector2f(it->C.x, it->C.y)),
                    };
            window.draw(AB, 2, sf::Lines);
            window.draw(BC, 2, sf::Lines);
            window.draw(AC, 2, sf::Lines);
        }

        for (auto it = right_triangulation.begin(); it != right_triangulation.end(); ++it) {
            sf::Vertex AB[] =
                    {
                            sf::Vertex(sf::Vector2f(it->A.x, it->A.y)),
                            sf::Vertex(sf::Vector2f(it->B.x, it->B.y))
                    };
            sf::Vertex BC[] =
                    {
                            sf::Vertex(sf::Vector2f(it->B.x, it->B.y)),
                            sf::Vertex(sf::Vector2f(it->C.x, it->C.y)),
                    };
            sf::Vertex AC[] =
                    {
                            sf::Vertex(sf::Vector2f(it->A.x, it->A.y)),
                            sf::Vertex(sf::Vector2f(it->C.x, it->C.y)),
                    };
            window.draw(AB, 2, sf::Lines);
            window.draw(BC, 2, sf::Lines);
            window.draw(AC, 2, sf::Lines);
        }*/

        /*for (auto it = left_edges.begin(); it != left_edges.end(); ++it) {
            sf::Vertex AC[] =
                    {
                            sf::Vertex(sf::Vector2f(it->p1.x, it->p1.y)),
                            sf::Vertex(sf::Vector2f(it->p2.x, it->p2.y)),
                    };
            window.draw(AC, 2, sf::Lines);
        }

        for (auto it = right_edges.begin(); it != right_edges.end(); ++it) {
            sf::Vertex AC[] =
                    {
                            sf::Vertex(sf::Vector2f(it->p1.x, it->p1.y)),
                            sf::Vertex(sf::Vector2f(it->p2.x, it->p2.y)),
                    };
            window.draw(AC, 2, sf::Lines);
        }*/

        for (size_t i = 0; i < triangles.size(); ++i) {
            for (auto it = triangles[i].begin(); it != triangles[i].end(); ++it) {
                sf::Vertex AB[] =
                        {
                                sf::Vertex(sf::Vector2f(it->A.x, it->A.y)),
                                sf::Vertex(sf::Vector2f(it->B.x, it->B.y))
                        };
                sf::Vertex BC[] =
                        {
                                sf::Vertex(sf::Vector2f(it->B.x, it->B.y)),
                                sf::Vertex(sf::Vector2f(it->C.x, it->C.y)),
                        };
                sf::Vertex AC[] =
                        {
                                sf::Vertex(sf::Vector2f(it->A.x, it->A.y)),
                                sf::Vertex(sf::Vector2f(it->C.x, it->C.y)),
                        };
                window.draw(AB, 2, sf::Lines);
                window.draw(BC, 2, sf::Lines);
                window.draw(AC, 2, sf::Lines);
            }
        }


       /*for (size_t i = 0; i < triangles.size(); ++i) {
            if (i == 4 || i == 0) {
                continue;
            }
            auto t = triangles[i].get();
            for (auto it = t.begin(); it != t.end(); ++it) {
                sf::Vertex AB[] =
                        {
                                sf::Vertex(sf::Vector2f(it->A.x, it->A.y)),
                                sf::Vertex(sf::Vector2f(it->B.x, it->B.y))
                        };
                sf::Vertex BC[] =
                        {
                                sf::Vertex(sf::Vector2f(it->B.x, it->B.y)),
                                sf::Vertex(sf::Vector2f(it->C.x, it->C.y)),
                        };
                sf::Vertex AC[] =
                        {
                                sf::Vertex(sf::Vector2f(it->A.x, it->A.y)),
                                sf::Vertex(sf::Vector2f(it->C.x, it->C.y)),
                        };
                window.draw(AB, 2, sf::Lines);
                window.draw(BC, 2, sf::Lines);
                window.draw(AC, 2, sf::Lines);
            }
       }

        for (auto it = middle_triangles.begin(); it != middle_triangles.end(); ++it) {
            sf::Vertex AB[] =
                    {
                            sf::Vertex(sf::Vector2f(it->A.x, it->A.y)),
                            sf::Vertex(sf::Vector2f(it->B.x, it->B.y))
                    };
            sf::Vertex BC[] =
                    {
                            sf::Vertex(sf::Vector2f(it->B.x, it->B.y)),
                            sf::Vertex(sf::Vector2f(it->C.x, it->C.y)),
                    };
            sf::Vertex AC[] =
                    {
                            sf::Vertex(sf::Vector2f(it->A.x, it->A.y)),
                            sf::Vertex(sf::Vector2f(it->C.x, it->C.y)),
                    };
            window.draw(AB, 2, sf::Lines);
            window.draw(BC, 2, sf::Lines);
            window.draw(AC, 2, sf::Lines);
        }

        for (auto it = left_triangles.begin(); it != left_triangles.end(); ++it) {
            sf::Vertex AB[] =
                    {
                            sf::Vertex(sf::Vector2f(it->A.x, it->A.y)),
                            sf::Vertex(sf::Vector2f(it->B.x, it->B.y))
                    };
            sf::Vertex BC[] =
                    {
                            sf::Vertex(sf::Vector2f(it->B.x, it->B.y)),
                            sf::Vertex(sf::Vector2f(it->C.x, it->C.y)),
                    };
            sf::Vertex AC[] =
                    {
                            sf::Vertex(sf::Vector2f(it->A.x, it->A.y)),
                            sf::Vertex(sf::Vector2f(it->C.x, it->C.y)),
                    };
            window.draw(AB, 2, sf::Lines);
            window.draw(BC, 2, sf::Lines);
            window.draw(AC, 2, sf::Lines);
        }*/

//        for (auto it = boundary_triangles.begin(); it != boundary_triangles.end(); ++it) {
//            sf::Vertex AB[] =
//                    {
//                            sf::Vertex(sf::Vector2f(it->A.x, it->A.y)),
//                            sf::Vertex(sf::Vector2f(it->B.x, it->B.y))
//                    };
//            sf::Vertex BC[] =
//                    {
//                            sf::Vertex(sf::Vector2f(it->B.x, it->B.y)),
//                            sf::Vertex(sf::Vector2f(it->C.x, it->C.y)),
//                    };
//            sf::Vertex AC[] =
//                    {
//                            sf::Vertex(sf::Vector2f(it->A.x, it->A.y)),
//                            sf::Vertex(sf::Vector2f(it->C.x, it->C.y)),
//                    };
//            window.draw(AB, 2, sf::Lines);
//            window.draw(BC, 2, sf::Lines);
//            window.draw(AC, 2, sf::Lines);
//        }

        // end the current frame
        window.display();
    }

    return 0;


}
