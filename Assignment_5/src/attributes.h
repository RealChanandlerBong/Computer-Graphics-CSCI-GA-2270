#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>


class LightAttribute {
public:
    LightAttribute() {
        obj_ambient_color << 0.0, 0.5, 0.0, 0.0;
        obj_diffuse_color << 0.5, 0.5, 0.5, 0.0;
        obj_specular_color << 0.2, 0.2, 0.2, 0.0;
        obj_specular_exponent = 256.0;

        light_position << 0, -10, 10, 1;
        light_color << 16, 16, 16, 1;
        ambient_light << 0.2, 0.2, 0.2, 0;
    }

    // Object settings
    Eigen::Vector4d obj_ambient_color;
    Eigen::Vector4d obj_diffuse_color;
    Eigen::Vector4d obj_specular_color;
    double obj_specular_exponent;

    // Light
    Eigen::Vector4d light_position;
    Eigen::Vector4d light_color;

    //Ambient light
    Eigen::Vector4d ambient_light;
};

class CameraAttributes {
public:
    CameraAttributes() {
        is_perspective = true;

        field_of_view = 0.5; // 45 degrees

        position << 0, 0.1, 0.5, 1;
        gaze_direction << 0, 0, -1, 0;
    }

    Eigen::Matrix4d to_camera_space() {
        Eigen::Vector4d e = position;
        Eigen::Vector4d g = gaze_direction;
        Eigen::Vector3d w = -1 * Eigen::Vector3d(g(0), g(1), g(2)).normalized();
        Eigen::Vector3d t(0, 1, 0); // view-up vector
        Eigen::Vector3d u = t.cross(w).normalized();
        Eigen::Vector3d v = w.cross(u);

        Eigen::Matrix4d M;
        M.col(0) << u(0), u(1), u(2), 0;
        M.col(1) << v(0), v(1), v(2), 0;
        M.col(2) << w(0), w(1), w(2), 0;
        M.col(3) = e;
        return M.inverse();
    }

    Eigen::Matrix4d to_canonical_view() {
        Eigen::Matrix4d M;
        const double l = min(0), b = min(1), n = min(2);
        const double r = max(0), t = max(1), f = max(2);
        M << 2/(r-l), 0, 0, -(r+l)/(r-l),
            0, 2/(t-b), 0, -(t+b)/(t-b),
            0, 0, 2/(n-f), -(n+f)/(n-f),
            0, 0, 0, 1;
        return M;
    }

    Eigen::Matrix4d to_perspective_projection() {
        if (!is_perspective) {
            return Eigen::Matrix4d::Identity();
        }
        Eigen::Matrix4d M;
        const double n = min(2), f = max(2);
        M << n, 0, 0, 0,
            0, n, 0, 0,
            0, 0, n+f, -n*f,
            0, 0, 1, 0;
        return M;
    }

    void set_parameters(const Eigen::AlignedBox3d& bbox) {
        if (is_perspective) {
            const double n = -0.1, f = -2.5;
            const double t = tan(field_of_view/2) * abs( n);
            const double r = t*aspect_ratio;
            max << r, t, f;
            min << -r, -t, n;
        } else {
            max = bbox.max().array() + exp(-6);
            min = bbox.min().array() - exp(-6);
        }
    }

    bool is_perspective;
    double field_of_view;
    double aspect_ratio;

    Eigen::Vector4d position;
    Eigen::Vector4d gaze_direction;
    Eigen::Vector3d max, min;
};

class VertexAttributes
{
	public:
	VertexAttributes(double x = 0, double y = 0, double z = 0, double w = 1)
	{
		position << x,y,z,w;
	}

    // Interpolates the vertex attributes
    static VertexAttributes interpolate(
        const VertexAttributes& a,
        const VertexAttributes& b,
        const VertexAttributes& c,
        const double alpha, 
        const double beta, 
        const double gamma
    ) 
    {
        VertexAttributes r;
        r.position = alpha*a.position + beta*b.position + gamma*c.position;
        r.color = alpha*a.color + beta*b.color + gamma*c.color;
        return r;
    }

	Eigen::Vector4d position;
    Eigen::Vector4d color;
    Eigen::Vector4d normal;
};

class FragmentAttributes
{
	public:
	FragmentAttributes(double r = 0, double g = 0, double b = 0, double a = 1)
	{
		color << r,g,b,a;
	}

	Eigen::Vector4d color;
    Eigen::Vector4d position;
    double depth;
};

class FrameBufferAttributes
{
	public:
	FrameBufferAttributes(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255)
	{
		color << r,g,b,a;
        depth = -10000;
	}

	Eigen::Matrix<uint8_t,4,1> color;
    double depth;
};

class UniformAttributes
{
	public:

    LightAttribute light;
    CameraAttributes camera;
    Eigen::Matrix4d M_cam, M_proj;
};

class MeshAttributes
{
public:
    MeshAttributes(const std::string &filename) {
        this->filename = filename;
    }

    void load() {
        //Loads file
        std::ifstream in(filename);
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
    }

    void build_face_vertices(std::vector<VertexAttributes> &v) {
        int nf = facets.rows();
        v.reserve(nf*3);
        for (int i = 0; i < nf; ++i) {
            v.emplace_back(vertices(facets(i,0),0), vertices(facets(i,0),1), vertices(facets(i,0),2));
            v.emplace_back(vertices(facets(i,1),0), vertices(facets(i,1),1), vertices(facets(i,1),2));
            v.emplace_back(vertices(facets(i,2),0), vertices(facets(i,2),1), vertices(facets(i,2),2));
        }
    }

    void build_line_vertices(std::vector<VertexAttributes> &v) {
        int nf = facets.rows();
        v.reserve(nf*6);
        for (int i = 0; i < nf; ++i) {
            v.emplace_back(vertices(facets(i,0),0), vertices(facets(i,0),1), vertices(facets(i,0),2));
            v.emplace_back(vertices(facets(i,1),0), vertices(facets(i,1),1), vertices(facets(i,1),2));

            v.emplace_back(vertices(facets(i,1),0), vertices(facets(i,1),1), vertices(facets(i,1),2));
            v.emplace_back(vertices(facets(i,2),0), vertices(facets(i,2),1), vertices(facets(i,2),2));

            v.emplace_back(vertices(facets(i,2),0), vertices(facets(i,2),1), vertices(facets(i,2),2));
            v.emplace_back(vertices(facets(i,0),0), vertices(facets(i,0),1), vertices(facets(i,0),2));
        }
    }

    void build_facet_normals(std::vector<VertexAttributes> &v, const CameraAttributes &camera) {
        int nf = facets.rows();
        for (int i = 0; i < nf; ++i) {
            VertexAttributes &va = v.at(3*i);
            VertexAttributes &vb = v.at(3*i+1);
            VertexAttributes &vc = v.at(3*i+2);

            Eigen::Vector3d a,b,c;
            a << va.position(0), va.position(1), va.position(2);
            b << vb.position(0), vb.position(1), vb.position(2);
            c << vc.position(0), vc.position(1), vc.position(2);

            Eigen::Vector3d N = (b-a).cross(c-a).normalized();
            Eigen::Vector3d g;
            g << camera.gaze_direction(0), camera.gaze_direction(1), camera.gaze_direction(2);
            if (N.dot(g) > 0.) {
                N = -1*N;
            }
            va.normal << N(0),N(1),N(2),0;
            vb.normal << N(0),N(1),N(2),0;
            vc.normal << N(0),N(1),N(2),0;
        }
    }

    void build_vertex_normals(std::vector<VertexAttributes> &v, const CameraAttributes &camera) {
        int nv = vertices.rows(), nf = facets.rows();
        std::vector<int> count(nv, 0);
        std::vector<Eigen::Vector4d> normals(nv, Eigen::Vector4d(0,0,0,0));

        for (int i = 0; i < nf; ++i) {
            VertexAttributes &va = v.at(3*i);
            VertexAttributes &vb = v.at(3*i+1);
            VertexAttributes &vc = v.at(3*i+2);

            Eigen::Vector3d a,b,c;
            a << va.position(0), va.position(1), va.position(2);
            b << vb.position(0), vb.position(1), vb.position(2);
            c << vc.position(0), vc.position(1), vc.position(2);

            Eigen::Vector3d N0 = (b-a).cross(c-a).normalized();
            Eigen::Vector4d N(N0(0), N0(1), N0(2), 0);
            Eigen::Vector4d g;
            if (camera.is_perspective) {
                Eigen::Vector3d p = a+b+c / 2;
                g = (camera.position - Eigen::Vector4d(p(0),p(1),p(2),1)).normalized();
            } else {
                g = camera.gaze_direction;
            }
            if (N.dot(g) > 0.) {
                N = -1*N;
            }

            normals[facets(i,0)] += N;
            normals[facets(i,1)] += N;
            normals[facets(i,2)] += N;
            ++count[facets(i,0)];
            ++count[facets(i,1)];
            ++count[facets(i,2)];
        }

        for (int i = 0; i < nf; ++i) {
            v[3*i].normal = (normals[facets(i,0)] / count[facets(i,0)]).normalized();
            v[3*i+1].normal = (normals[facets(i,1)] / count[facets(i,1)]).normalized();
            v[3*i+2].normal = (normals[facets(i,2)] / count[facets(i,2)]).normalized();
        }
    }

    void build_alignedbox(Eigen::AlignedBox3d& bbox, const Eigen::Matrix4d& M) {
        int nv = vertices.rows();
        for (int i = 0; i < nv; ++i) {
            Eigen::Vector3d v = vertices.row(i);
            bbox.extend(v);
        }
        Eigen::Vector3d max = bbox.max();
        Eigen::Vector3d min = bbox.min();
        bbox.setEmpty();
        Eigen::Vector4d mmax = M*Eigen::Vector4d(max(0),max(1),max(2),1);
        Eigen::Vector4d mmin = M*Eigen::Vector4d(min(0),min(1),min(2),1);
        bbox.extend(Eigen::Vector3d(mmax(0),mmax(1),mmax(2)));
        bbox.extend(Eigen::Vector3d(mmin(0),mmin(1),mmin(2)));
    }

    void build_barycenter(Eigen::Vector4d& center) {
        int nv = vertices.rows();
        for (int i = 0; i < nv; ++i) {
            Eigen::Vector3d r0 = vertices.row(i);
            Eigen::Vector4d r(r0(0),r0(1),r0(2),1);
            center += r;
        }
        center /= nv;
    }

private:
    std::string filename;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi facets;
};
