////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

double inline det(const Point &u, const Point &v) {
    // TODO
    return u.real()*v.imag() - u.imag()*v.real();
}

// Return true iff [a,b] intersects [c,d], and store the intersection in ans
bool intersect_segment(const Point &a, const Point &b, const Point &c, const Point &d, Point &ans) {
    // TODO
    if (c.imag() <= MIN(a.imag(), b.imag()) || c.imag() > MAX(a.imag(), b.imag())) {
        return false;
    }
    if (c.real() > MAX(a.real(), b.real())) {
        return false;
    }
    if (a.imag() == b.imag()) {
        return false;
    }
    double xi = (c.imag()-a.imag()) * (b.real()-a.real()) / (b.imag()-a.imag()) + a.real();
    if (a.real() == b.real() || c.real() <= xi) {
        return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const Polygon &poly, const Point &query) {
    // 1. Compute bounding box and set coordinate of a point outside the polygon
    // TODO
    double minX = 0;
    for (const auto &p: poly) {
        minX = MIN(minX, p.real());
    }
    Point outside(query.real(), minX - 10000);
    // 2. Cast a ray from the query point to the 'outside' point, count number of intersections
    // TODO
    int counter = 0, n = poly.size();
    Point p1 = poly[0], p2, ans;
    for (int i = 1; i <= n; i++) {
        p2 = poly[i % n];
        if (intersect_segment(p1, p2, query, outside, ans)) {
            counter++;
        }
        p1 = p2;
    }
    return counter % 2 == 1;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Point> load_xyz(const std::string &filename) {
    std::vector<Point> points;
    std::ifstream in(filename);
    // TODO
    if (!in.is_open()) {
        throw std::runtime_error("failed to open file " + filename);
    }
    int total = 0;
    in >> total;
    double x = 0.0, y = 0.0, z = 0.0;
    while (in >> x >> y >> z) {
        std::complex<double> point(x, y);
        points.push_back(point);
    }
    return points;
}

Polygon load_obj(const std::string &filename) {
    std::ifstream in(filename);
    // TODO
    if (!in.is_open()) {
        throw std::runtime_error("failed to open file " + filename);
    }
    std::vector<Point> vertexes;
    std::vector<int> indices;
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "v") {
            double x = 0, y = 0, z = 0;
            iss >> x >> y >> z;
            vertexes.emplace_back(x, y);
        } else if (type == "f") {
            int id = 0;
            while (iss >> id) {
                indices.push_back(id);
            }
        } else {
            // ignore
        }
    }
    Polygon polygon;
    for (int id : indices) {
        polygon.push_back(vertexes.at(id - 1));
    }
    return polygon;
}

void save_xyz(const std::string &filename, const std::vector<Point> &points) {
    // TODO
    std::ofstream out(filename);
    if (!out.is_open()) {
        throw std::runtime_error("failed to open file " + filename);
    }
    out << points.size() << "\n";
    for (const auto &p: points) {
        out << p.real() << " " << p.imag() << " 0"  << "\n";
    }
    out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
    if (argc <= 3) {
        std::cerr << "Usage: " << argv[0] << " points.xyz poly.obj result.xyz" << std::endl;
    }
    std::vector<Point> points = load_xyz(argv[1]);
    Polygon poly = load_obj(argv[2]);
    std::vector<Point> result;
    for (size_t i = 0; i < points.size(); ++i) {
        if (is_inside(poly, points[i])) {
            result.push_back(points[i]);
        }
    }
    std::cout << points.size() << std::endl;
    std::cout << result.size() << std::endl;
    save_xyz(argv[3], result);
    return 0;
}
