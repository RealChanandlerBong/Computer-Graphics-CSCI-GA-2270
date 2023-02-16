// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"


// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

void raytrace_sphere() {
	std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

	const std::string filename("sphere_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);

    // sphpere center
    const Vector3d sphere_center(0.6, -0.6, -0.2);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0),ray_origin(1));

            // project ray and sphere onto xy-plane
            Vector2d sphere_on_xy(sphere_center(0), sphere_center(1));
            Vector2d ray_from_sphere = ray_on_xy - sphere_on_xy;
            const double sphere_radius = 0.6;

			if (ray_from_sphere.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection(ray_on_xy(0),ray_on_xy(1),sqrt(sphere_radius*sphere_radius - ray_from_sphere.squaredNorm())-sphere_center(2));

				// Compute normal at the intersection point
				Vector3d ray_normal = (ray_intersection - sphere_center).normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);

}

// returns true if ray intersects with the parallelogram and stores the intersection in Vector3d ans;
// returns false otherwise
bool intersect_orthographic(const Vector3d &ray_origin,
                            const Vector3d &pgram_origin,
                            const Vector3d &pgram_u,
                            const Vector3d &pgram_v,
                            Vector3d &ans) {
    Matrix2d A;
    A(0, 0) = pgram_u(0);
    A(0, 1) = pgram_v(0);
    A(1, 0) = pgram_u(1);
    A(1, 1) = pgram_v(1);
    if (abs(A.determinant()) < exp(-6)) {
        return false;
    }
    Vector2d b(ray_origin(0)-pgram_origin(0), ray_origin(1)-pgram_origin(1));
    Vector2d x = A.partialPivLu().solve(b);
    if (x(0) <= 0 || x(0) >= 1 || x(1) <= 0 || x(1) >= 1) {
        return false;
    }
    ans(0) = ray_origin(0);
    ans(1) = ray_origin(1);
    ans(2) = pgram_origin(2) + x(0)*pgram_u(2) + x(1)*pgram_v(2);
    return true;
}

void raytrace_parallelogram() {
	std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

	const std::string filename("plane_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    Vector3d pgram_origin(0.5, -0.5, -1);
    Vector3d pgram_u(0.1, 0.8, 0.4);
    Vector3d pgram_v(-1, 0.5, -0.1);

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);
            Vector3d ray_intersection(0,0,0);
			// TODO: Check if the ray intersects with the parallelogram
			if (intersect_orthographic(ray_origin, pgram_origin, pgram_u, pgram_v, ray_intersection)) {
				// TODO: The ray hit the parallelogram, compute the exact intersection point

				// TODO: Compute normal at the intersection point
                Vector3d OU = pgram_u - pgram_origin;
                Vector3d OV = pgram_v - pgram_origin;
                Vector3d ray = OU.cross(OV);
                // make the normal vector upward
                if (ray(2) < 0) {
                    ray = -1 * ray;
                }
                Vector3d ray_normal = ray.normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

// returns true if the ray intersects with the parallelogram and stores the intersection in Vector3d ans;
// returns false otherwise
bool intersect_perspective(const Vector3d &ray_origin,
                           const Vector3d &ray_dir,
                           const Vector3d &pgram_origin,
                           const Vector3d &pgram_u,
                           const Vector3d &pgram_v,
                           Vector3d &ans) {
    Matrix3d A;
    A << pgram_u(0), pgram_v(0), -1*ray_dir(0),
            pgram_u(1), pgram_v(1), -1*ray_dir(1),
            pgram_u(2), pgram_v(2), -1*ray_dir(2);
    Vector3d b = ray_origin - pgram_origin;
    Vector3d x = A.fullPivLu().solve(b);
    if ((A * x - b).norm() / b.norm() > exp(-6)) {
        return false;
    }
    if (x(0) <= 0 || x(0) >= 1 || x(1) <= 0 || x(1) >= 1) {
        return false;
    }
    ans = ray_origin + x(2)*ray_dir;
    return true;
}

void raytrace_perspective() {
	std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

	const std::string filename("plane_perspective.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    Vector3d pgram_origin(0.5, -0.5, -1);
    Vector3d pgram_u(0.1, 0.8, 0.4);
    Vector3d pgram_v(-1, 0.5, -0.1);

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// TODO: Prepare the ray (origin point and direction)
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
            Vector3d pixel(ray_origin(0), ray_origin(1), 0);
			Vector3d ray_direction = pixel - ray_origin;

            Vector3d ray_intersection(0,0,0);
			// TODO: Check if the ray intersects with the parallelogram
			if (intersect_perspective(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v, ray_intersection)) {
				// TODO: The ray hit the parallelogram, compute the exact intersection point

				// TODO: Compute normal at the intersection point
                Vector3d OU = pgram_u - pgram_origin;
                Vector3d OV = pgram_v - pgram_origin;
                Vector3d ray = OU.cross(OV);
                // make the normal vector upward
                if (ray(2) < 0) {
                    ray = -1 * ray;
                }
				Vector3d ray_normal = ray.normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

void raytrace_shading(){
	std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

	const std::string filename("shading.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);
	double ambient = 0.1;
	MatrixXd diffuse = MatrixXd::Zero(800, 800);
	MatrixXd specular = MatrixXd::Zero(800, 800);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0),ray_origin(1));
			const double sphere_radius = 0.9;

			if (ray_on_xy.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0),ray_on_xy(1),sqrt(sphere_radius*sphere_radius - ray_on_xy.squaredNorm()));

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();
                Vector3d light_normal = (light_position-ray_intersection).normalized();
                Vector3d view_normal = (origin - ray_intersection).normalized();
                Vector3d h_normal = (light_normal + view_normal) / (light_normal + view_normal).norm();

                // TODO: Add shading parameter here
                double diffuse_param = 1.0;
                double specular_param = 0.2, specular_exp = 100;
				diffuse(i,j) = light_normal.transpose() * ray_normal;
				specular(i,j) = pow(h_normal.transpose() * ray_normal, specular_exp);

				// Simple diffuse model
                diffuse(i, j) = std::max(diffuse(i, j), 0.);
                specular(i, j) = std::max(specular(i, j), 0.);
				C(i,j) = ambient + diffuse_param * diffuse(i,j) + specular_param * specular(i,j);

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C*0.8,C*0.6,C*0.9,A,filename);
}

int main() {
	raytrace_sphere();
    raytrace_parallelogram();
    raytrace_perspective();
	raytrace_shading();

	return 0;
}
