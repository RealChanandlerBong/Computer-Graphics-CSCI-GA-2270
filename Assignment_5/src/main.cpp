// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "raster.h"
#include "gif.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

// Shading options
const string shading_option = "wireframe";
//const string shading_option = "flat_shading";
//const string shading_option = "per_vertex_shading";

// Camera settings
bool is_perspective = true;

// Export settings
bool use_gif = true;

int main() 
{
	// The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(500,500);

	// Global Constants (empty in this example)
	UniformAttributes uniform;

	// Basic rasterization program
	Program program;

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
        VertexAttributes out;
        out.position = uniform.M_cam * va.position;
        out.normal = uniform.M_cam.transpose().inverse() * va.normal;
        if (va.normal.dot(Vector4d(1,1,1,1)) == 0.) {
            out.color << 1, 1, 1, 1;
        } else {
            Vector4d light_color(0, 0, 0, 1);

            // Ambient color
            const Vector4d ambient = uniform.light.obj_ambient_color.array() * uniform.light.ambient_light.array();

            // Diffuse color
            const Vector4d l = (uniform.light.light_position - out.position).normalized();
            const Vector4d diffuse = uniform.light.obj_diffuse_color * std::max(l.dot(out.normal), 0.0);

            // Specular color
            const Vector4d v = uniform.camera.is_perspective? (uniform.camera.position-out.position).normalized():-1*uniform.camera.gaze_direction;
            const Vector4d h = (v + l).normalized();
            const Vector4d specular = uniform.light.obj_specular_color *
                                      std::pow(std::max(h.dot(out.normal), 0.0), uniform.light.obj_specular_exponent);

            // Attenuate lights according to the squared distance to the lights
            const Vector4d dist = uniform.light.light_position - out.position;
            light_color += (diffuse + specular).cwiseProduct(uniform.light.light_color) / dist.squaredNorm();
            light_color += ambient;

            out.color = light_color;
        }
        out.position = uniform.M_proj * out.position;
        return out;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
        FragmentAttributes out;
        out.color = va.color;
        out.position = va.position;
        out.depth = va.position(2);
		return out;
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
        if (fa.depth > previous.depth) {
            FrameBufferAttributes out(fa.color[0]*255, fa.color[1]*255, fa.color[2]*255, fa.color[3]*255);
            out.depth = fa.depth;
            return out;
        }
        return previous;
	};

    // Set camera parameters
    if (is_perspective) {
        uniform.camera.is_perspective = true;
    }
    uniform.camera.aspect_ratio = (double)frameBuffer.cols() / (double)frameBuffer.rows();

	// Load mesh
    MeshAttributes mesh(string(DATA_DIR) + "bunny.off");
    mesh.load();

	vector<VertexAttributes> vertices;
	vector<VertexAttributes> lines;

    // build face normals and lines
    if (shading_option == "wireframe")  {
        mesh.build_line_vertices(lines);
    } else if (shading_option == "flat_shading") {
        mesh.build_line_vertices(lines);
        mesh.build_face_vertices(vertices);
        mesh.build_facet_normals(vertices, uniform.camera);
    } else if (shading_option == "per_vertex_shading") {
        mesh.build_face_vertices(vertices);
        mesh.build_vertex_normals(vertices, uniform.camera);
    }

    // Matrices we need
    Matrix4d M_cam = uniform.camera.to_camera_space();

    // Compute the barycenter
    Vector4d barycenter(0,0,0,0);
    mesh.build_barycenter(barycenter);

    // Compute the AABB of the object
    AlignedBox3d bbox;
    mesh.build_alignedbox(bbox, M_cam);

    // Set more camera parameters
    uniform.camera.set_parameters(bbox);

    // Matrices we need
    Matrix4d M_canon = uniform.camera.to_canonical_view();
    Matrix4d M_proj = uniform.camera.to_perspective_projection();

    uniform.M_cam = M_cam;
    uniform.M_proj = M_canon * M_proj;

    // rasterize
    rasterize_triangles(program,uniform,vertices,frameBuffer);
    rasterize_lines(program,uniform,lines,0.6,frameBuffer);

    vector<uint8_t> image;
    framebuffer_to_uint8(frameBuffer,image);
    stbi_write_png("bunny.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(),frameBuffer.rows() * 4);
	return 0;
}
