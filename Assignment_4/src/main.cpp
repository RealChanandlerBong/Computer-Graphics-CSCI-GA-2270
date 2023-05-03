////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////
class AABBTree
{
public:
    class Node
    {
    public:
        AlignedBox3d bbox;
        int parent;   // Index of the parent node (-1 for root)
        int left;     // Index of the left child (-1 for a leaf)
        int right;    // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
        int index;    // Index of itself
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default;                           // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh

    Node buildTree(const MatrixXd &V, const MatrixXi &F, const MatrixXd &centroids, std::vector<int> &ids, int l, int r);
};

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
const std::string filename("raytrace.png");
const std::string mesh_filename(data_dir + "dragon.off");
//const std::string mesh_filename(data_dir + "bunny.off");

//Camera settings
const double focal_length = 2;
const double field_of_view = 0.7854; //45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 2);

// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

//Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
//Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    //setup tree
    bvh = AABBTree(vertices, facets);

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Code
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}

AABBTree::Node AABBTree::buildTree(const MatrixXd &V, const MatrixXi &F, const MatrixXd &centroids, std::vector<int> &ids, int l, int r) {
    const int total = r-l+1;
    if (total == 1) {
        // return the leaf node
        Node leaf {
            AlignedBox3d(),
            -1,
            -1,
            -1,
            ids.at(r),
            static_cast<int>(nodes.size()),
        };
        const Vector3d a = V.row(F(ids.at(r), 0));
        const Vector3d b = V.row(F(ids.at(r), 1));
        const Vector3d c = V.row(F(ids.at(r), 2));
        leaf.bbox.extend(bbox_from_triangle(a, b, c));
        nodes.emplace_back(leaf);
        return leaf;
    }

    // find the longest axis
    AlignedBox3d box;
    for (int i = l; i <= r; ++i) {
        const Vector3d a = V.row(F(ids.at(i), 0));
        const Vector3d b = V.row(F(ids.at(i), 1));
        const Vector3d c = V.row(F(ids.at(i), 2));
        box.extend(bbox_from_triangle(a, b, c));
    }
    int max_axis = -1;
    double max_length = std::numeric_limits<double>::min();
    const Vector3d axis_length = box.max() - box.min();
    for (int i = 0; i < box.dim(); ++i) {
        if (axis_length(i) > max_length) {
            max_length = axis_length(i);
            max_axis = i;
        }
    }

    // sort by selecting smallest
    std::sort(std::next(ids.begin(), l), std::next(ids.begin(), r), [&](const int a, const int b) {
        return centroids(ids.at(a), max_axis) < centroids(ids.at(b), max_axis);
    });

    // split
    const int size1 = total / 2;
    const int size2 = total - size1;

    // recursive call
    Node left = buildTree(V, F, centroids, ids, l, l+size1-1);
    Node right = buildTree(V, F, centroids, ids, l+size1, r);

    // build current node
    Node cur {
        AlignedBox3d(),
        -1,
        left.index,
        right.index,
        -1,
        static_cast<int>(nodes.size()),
    };
    cur.bbox.extend(left.bbox);
    cur.bbox.extend(right.bbox);
    nodes.emplace_back(cur);

    return cur;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
    // Compute the centroids of all the triangles in the input mesh
    MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int k = 0; k < F.cols(); ++k)
        {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }

    // TODO

    // Split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.

    // method (1): top down
    std::vector<int> ids;
    for (int i = 0; i < F.rows(); ++i) {
        ids.emplace_back(i);
    }
    Node root_node = buildTree(V, F, centroids, ids, 0, F.rows()-1);
    root = root_node.index;

    // method (2): bottom up
//    std::vector<Node> S;
//    for (int i = 0; i < F.rows(); ++i) {
//        Node node {
//            AlignedBox3d(),
//            -1,
//            -1,
//            -1,
//            i,
//            i,
//        };
//        for (int j = 0; j < F.cols(); j++) {
//            VectorXd triangle = V.row(F(i, j));
//            node.bbox.extend(triangle);
//        }
//        nodes.push_back(node);
//        S.push_back(node);
//    }
//    int nodes_size = nodes.size(), S_size = S.size();
//    const double double_max = std::numeric_limits<double>::max();
//
//    while (S_size > 1) {
//        double min_cost = double_max;
//        int min_i = -1, min_j = -1;
//        for (int i = 0; i < S_size; ++i) {
//            Node node1 = S.at(i);
//            for (int j = i+1; j < S_size; ++j) {
//                Node node2 = S.at(j);
//                double cost = (node1.bbox.center()-node2.bbox.center()).norm();
//                if (cost < min_cost) {
//                    min_cost = cost;
//                    min_i = i;
//                    min_j = j;
//                }
//            }
//        }
//        Node node {
//            AlignedBox3d(),
//            -1,
//            S.at(min_i).index,
//            S.at(min_j).index,
//            -1,
//            nodes_size,
//        };
//        node.bbox.extend(nodes.at(S.at(min_i).index).bbox);
//        node.bbox.extend(nodes.at(S.at(min_j).index).bbox);
//        nodes.at(S.at(min_i).index).parent = nodes_size;
//        nodes.at(S.at(min_j).index).parent = nodes_size;
//        nodes.push_back(node);
//        nodes_size = nodes.size();
//
//        S.erase(std::next(S.begin(), min_j));
//        S.erase(std::next(S.begin(), min_i));
//        S.push_back(node);
//        S_size = S.size();
//    }
//
//    root = nodes_size - 1;
}

////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////

double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &a, const Vector3d &b, const Vector3d &c, Vector3d &p, Vector3d &N)
{
    // TODO
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.
    const Vector3d ab = b - a, ac = c - a;
    const Vector3d m = ray_origin - a;
    Matrix3d M;
    M << ab, ac, ray_direction;
    const Vector3d x = M.fullPivLu().solve(m);
    p = ray_origin - x(2)*ray_direction;
    if ((M*x-m).norm() / m.norm() > exp(-6)) {
        return -1;
    }
    Matrix3d abc;
    abc << a, b, c;
    const Vector3d y = abc.fullPivLu().solve(p);
    if ((abc*y - p).norm() / p.norm() > exp(-6) ||
            y(0) < 0. || y(0) > 1. || y(1) < 0. || y(1) > 1. || y(2) < 0. || y(2) > 1.) {
        return -1;
    }
    N = ab.cross(ac).normalized();
    if (N.dot(ray_direction) > 0.) {
        N = -1*N;
    }
    return -1*x(2);
}

bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
    // TODO
    // Compute whether the ray intersects the given box.
    // we are not testing with the real surface here anyway.
    if (box.contains(ray_origin)) {
        return true;
    }

    const Vector3d& p1 = box.min();
    const Vector3d& p2 = box.max();

    // method (1): compute intersections with 6 faces
//    const Vector3d t1 = (p1-ray_origin).array() / ray_direction.array();
//    const Vector3d t2 = (p2-ray_origin).array() / ray_direction.array();
//
//    if (t1(0) < 0. && t1(1) < 0. && t1(2) < 0.) {
//        return false;
//    }
//    if (t2(0) < 0. && t2(1) < 0. && t2(2) < 0.) {
//        return false;
//    }
//
//    const Vector3d px1 = ray_origin + t1(0)*ray_direction;
//    if (t1(0) > 0. && (px1(1)-p1(1))*(px1(1)-p2(1)) <= 0. && (px1(2)-p1(2))*(px1(2)-p2(2)) <= 0.) {
//        return true;
//    }
//    const Vector3d px2 = ray_origin + t2(0)*ray_direction;
//    if (t2(0) > 0. && (px2(1)-p1(1))*(px2(1)-p2(1)) <= 0. && (px2(2)-p1(2))*(px2(2)-p2(2)) <= 0.) {
//        return true;
//    }
//
//    const Vector3d py1 = ray_origin + t1(1)*ray_direction;
//    if (t1(1) > 0. && (py1(0)-p1(0))*(py1(0)-p2(0)) <= 0. && (py1(2)-p1(2))*(py1(2)-p2(2)) <= 0.) {
//        return true;
//    }
//    const Vector3d py2 = ray_origin + t2(1)*ray_direction;
//    if (t2(1) > 0. && (py2(0)-p1(0))*(py2(0)-p2(0)) <= 0. && (py2(2)-p1(2))*(py2(2)-p2(2)) <= 0.) {
//        return true;
//    }
//
//    const Vector3d pz1 = ray_origin + t1(2)*ray_direction;
//    if (t1(2) > 0. && (pz1(0)-p1(0))*(pz1(0)-p2(0)) <= 0. && (pz1(1)-p1(1))*(pz1(1)-p2(1)) <= 0.) {
//        return true;
//    }
//    const Vector3d pz2 = ray_origin + t2(2)*ray_direction;
//    if (t2(2) > 0. && (pz2(0)-p1(0))*(pz2(0)-p2(0)) <= 0. && (pz2(1)-p1(1))*(pz2(1)-p2(1)) <= 0.) {
//        return true;
//    }

    // method (2): compute the clip
    const Vector3d dir_frac = Vector3d(1,1,1).array() / ray_direction.array();
    const Vector3d t1 = (p1-ray_origin).cwiseProduct(dir_frac);
    const Vector3d t2 = (p2-ray_origin).cwiseProduct(dir_frac);

    const double t_min = std::max(std::max(std::min(t1(0),t2(0)), std::min(t1(1),t2(1))), std::min(t1(2),t2(2)));
    const double t_max = std::min(std::min(std::max(t1(0),t2(0)), std::max(t1(1),t2(1))), std::max(t1(2),t2(2)));

    if (t_max < 0.) {
        return false;
    }
    if (t_min > t_max) {
        return false;
    }
    return true;
}

//Finds the closest intersecting object returns its index
//In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{
    Vector3d tmp_p, tmp_N;

    // TODO
    // Method (1): Traverse every triangle and return the closest hit.
    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.
    int closet_index = -1;
    double closest_t = std::numeric_limits<double>::max(); //closest t is "+ infinity"

    // Method (1)
//    for (int i = 0; i < facets.rows(); ++i) {
//        Vector3i facet;
//        facet << facets.row(i)(0), facets.row(i)(1), facets.row(i)(2);
//        Vector3d a, b, c;
//        a << vertices.row(facet(0))(0), vertices.row(facet(0))(1),vertices.row(facet(0))(2);
//        b << vertices.row(facet(1))(0), vertices.row(facet(1))(1), vertices.row(facet(1))(2);
//        c << vertices.row(facet(2))(0), vertices.row(facet(2))(1), vertices.row(facet(2))(2);
//
//        const double t = ray_triangle_intersection(ray_origin, ray_direction, a, b, c, tmp_p, tmp_N);
//        if (t > 0.) {
//            if (t < closest_t) {
//                closet_index = i;
//                closest_t = t;
//                p = tmp_p;
//                N = tmp_N;
//            }
//        }
//    }

    // Method (2)
    const int root = bvh.root;
    if (!ray_box_intersection(ray_origin, ray_direction, bvh.nodes.at(root).bbox)) {
        return false;
    }
    std::deque<int> queue;
    queue.emplace_back(root);
    while (!queue.empty()) {
        AABBTree::Node cur = bvh.nodes.at(queue.front());
        queue.pop_front();
        if (cur.left == -1 || cur.right == -1) { // leaf node
            assert(cur.triangle != -1);
            Vector3i facet;
            facet << facets.row(cur.triangle)(0), facets.row(cur.triangle)(1), facets.row(cur.triangle)(2);
            Vector3d a, b, c;
            a = vertices.row(facet(0));
            b = vertices.row(facet(1));
            c = vertices.row(facet(2));
            const double t = ray_triangle_intersection(ray_origin, ray_direction, a, b, c, tmp_p, tmp_N);
            if (t > 0. && t < closest_t) {
                closet_index = cur.triangle;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        } else if (cur.triangle == -1) { // internal node
            assert(cur.left != -1);
            assert(cur.right != -1);
            AABBTree::Node left = bvh.nodes.at(cur.left);
            AABBTree::Node right = bvh.nodes.at(cur.right);
            if (ray_box_intersection(ray_origin, ray_direction, left.bbox)) {
                queue.emplace_back(cur.left);
            }
            if (ray_box_intersection(ray_origin, ray_direction, right.bbox)) {
                queue.emplace_back(cur.right);
            }
        }
    }

    return closet_index != -1;
}

////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////

Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction)
{
    //Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;

    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (!nearest_object)
    {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }

    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    for (int i = 0; i < light_positions.size(); ++i)
    {
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];

        Vector4d diff_color = obj_diffuse_color;

        // Diffuse contribution
        const Vector3d Li = (light_position - p).normalized();
        const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

        // Specular contribution
        const Vector3d Hi = (Li - ray_direction).normalized();
        const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
        // Vector3d specular(0, 0, 0);

        // Attenuate lights according to the squared distance to the lights
        const Vector3d D = light_position - p;
        lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
    }

    // Rendering equation
    Vector4d C = ambient_color + lights_color;

    //Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Simple ray tracer." << std::endl;

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    //TODO
    double image_y = focal_length * tan(field_of_view/2);
    double image_x = image_y * aspect_ratio;

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

    for (unsigned i = 0; i < w; ++i)
    {
        for (unsigned j = 0; j < h; ++j)
        {
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = (pixel_center - camera_position).normalized();
            }
            else
            {
                // Orthographic camera
                ray_origin = pixel_center;
                ray_direction = Vector3d(0, 0, -1);
            }

            const Vector4d C = shoot_ray(ray_origin, ray_direction);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    auto start = std::chrono::steady_clock::now();
    setup_scene();

    raytrace_scene();
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "Total time: " << duration.count() << "s" << std::endl;
    return 0;
}