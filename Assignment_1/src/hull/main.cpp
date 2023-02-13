////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

double inline det(const Point &u, const Point &v) {
    // TODO
    return u.real()*v.imag() - u.imag()*v.real();
}

double inline distance(const Point &p1, const Point &p2) {
    return (p1.real()-p2.real()) * (p1.real()-p2.real()) + (p1.imag()-p2.imag()) * (p1.imag()-p2.imag());
}

struct Compare {
    Point p0; // Leftmost point of the poly
    bool operator ()(const Point &p1, const Point &p2) {
        // TODO
        Point p01(p1.real() - p0.real(), p1.imag() - p0.imag());
        Point p02(p2.real() - p0.real(), p2.imag() - p0.imag());
        double cross = det(p01, p02);

        if (cross == 0) {
            if (distance(p0, p1) < distance(p0, p2)) {
                return true;
            }
            return false;
        }
        return cross > 0.0;
    }
};

bool inline salientAngle(Point &a, Point &b, Point &c) {
    // TODO
    Point ab(b.real() - a.real(), b.imag() - a.imag());
    Point bc(c.real() - b.real(), c.imag() - b.imag());

    double cross = det(ab, bc);
    return cross > 0.0;
}

////////////////////////////////////////////////////////////////////////////////

Polygon convex_hull(std::vector<Point> &points) {
    Compare order;
    // TODO
    Point *p0 = nullptr;
    for (auto & point : points) {
        if (p0 == nullptr) {
            p0 = &point;
        } else {
            if (point.imag() < p0->imag()) {
                p0 = &point;
            } else if (point.imag() == p0->imag()) {
                if (point.real() < p0->real()) {
                    p0 = &point;
                }
            }
        }
    }
    std::cout << "P0 = (" << p0->real() << ", " << p0->imag() << ")" << std::endl;
    order.p0 = Point(p0->real(), p0->imag());
    std::sort(points.begin(), points.end(), order);
    Polygon hull;
    // TODO
    // use salientAngle(a, b, c) here
    for (auto & point : points) {
        if (hull.size() < 2) {
            hull.push_back(point);
            continue;
        }
        while (hull.size() > 1 && !salientAngle(hull.rbegin()[1], hull.back(), point)) {
            hull.pop_back();
        }
        hull.push_back(point);
    }
    return hull;
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

void save_obj(const std::string &filename, Polygon &poly) {
    std::ofstream out(filename);
    if (!out.is_open()) {
        throw std::runtime_error("failed to open file " + filename);
    }
    out << std::fixed;
    for (const auto &v : poly) {
        out << "v " << v.real() << ' ' << v.imag() << " 0\n";
    }
    for (size_t i = 0; i < poly.size(); ++i) {
        out << "l " << i+1 << ' ' << 1+(i+1)%poly.size() << "\n";
    }
    out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
    if (argc <= 2) {
        std::cerr << "Usage: " << argv[0] << " points.xyz output.obj" << std::endl;
    }
    std::vector<Point> points = load_xyz(argv[1]);
    Polygon hull = convex_hull(points);
    save_obj(argv[2], hull);
    return 0;
}
