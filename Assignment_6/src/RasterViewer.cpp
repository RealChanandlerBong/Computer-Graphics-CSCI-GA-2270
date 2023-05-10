#include "SDLViewer.h"

#include <Eigen/Core>

#include <functional>
#include <iostream>

#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

enum MODE {
    DEFAULT,
    INSERT,
    TRANSLATION,
    DELETE,
    COLOR,
};

std::vector<Eigen::Vector4d> colors {
    Eigen::Vector4d(0.9,0.7,0.8,1), // highlight color
    Eigen::Vector4d(0.7,0.9,0.8,1), // color 1 (default)
    Eigen::Vector4d(0.8,0.7,0.9,1),
    Eigen::Vector4d(1,1,0,1),
    Eigen::Vector4d(1,0,1,1),
    Eigen::Vector4d(0,1,1,1),
    Eigen::Vector4d(0.5,0.5,0,1),
    Eigen::Vector4d(0.5,0,0.5,1),
    Eigen::Vector4d(0,0.5,0.5,1),
    Eigen::Vector4d(0.5,0.5,0.5,1), // color 9
    Eigen::Vector4d(1,1,1,1), // white for lines
};

const Eigen::Vector4d highlight_color = colors[0];
const Eigen::Vector4d default_color = colors[1];
const Eigen::Vector4d line_color = colors[10];

void make_copy(std::vector<VertexAttributes> &local, std::vector<VertexAttributes> &committed) {
    local.clear();
    for (VertexAttributes &va: committed) {
        local.push_back(va);
    }
}

void commit_changes(std::vector<VertexAttributes> &local, std::vector<VertexAttributes> &committed) {
    committed.clear();
    for (VertexAttributes &va: local) {
        committed.push_back(va);
    }
}

void abort_changes(std::vector<VertexAttributes> &local) {
    local.clear();
}

void build_lines(const std::vector<VertexAttributes> &vertices, std::vector<VertexAttributes> &lines) {
    lines.clear();
    int v_num = vertices.size();
    for (int i = 0; i < v_num/3; ++i) {
        lines.push_back(vertices[3*i]);
        lines.push_back(vertices[3*i+1]);
        lines.push_back(vertices[3*i]);
        lines.push_back(vertices[3*i+2]);
        lines.push_back(vertices[3*i+1]);
        lines.push_back(vertices[3*i+2]);
    }
    if (v_num % 3 == 2) {
        lines.push_back(vertices[v_num-1]);
        lines.push_back(vertices[v_num-2]);
    }
}

int select_triangle(const std::vector<VertexAttributes> &vertices, const Eigen::Vector4d &cursor) {
    int selected = -1;
    for (int i = 0; i < vertices.size()/3; ++i) {
        Eigen::Vector4d a = vertices.at(3*i).position;
        Eigen::Vector4d b = vertices.at(3*i+1).position;
        Eigen::Vector4d c = vertices.at(3*i+2).position;
        Eigen::Matrix3d A;
        A << a(0),b(0),c(0),
            a(1),b(1),c(1),
            1, 1, 1;
        Eigen::Vector3d p(cursor(0), cursor(1), 1);
        Eigen::Vector3d t = A.fullPivLu().solve(p);
        if (t(0)>=0 && t(0)<=1 && t(1)>=0 && t(1)<=1 && t(2)>=0 && t(2)<=1) {
            selected = i;
        }
    }
    return selected;
}

int select_vertex(std::vector<VertexAttributes> &vertices, const Eigen::Vector4d &cursor) {
    int select = -1;
    double max_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < vertices.size(); ++i) {
        VertexAttributes &va = vertices.at(i);
        double dist = (va.position-cursor).norm();
        if (dist < max_dist) {
            max_dist = dist;
            select = i;
        }
    }
    return select;
}

void highlight_selected(std::vector<VertexAttributes> &local, const std::vector<VertexAttributes> &committed, int target) {
    if (target != -1) {
        local[3*target].color = highlight_color;
        local[3*target+1].color = highlight_color;
        local[3*target+2].color = highlight_color;
    }
}

void cancel_highlight_selected(std::vector<VertexAttributes> &local, const std::vector<VertexAttributes> &committed, int target) {
    if (target != -1) {
        local[3*target].color = committed[3*target].color;
        local[3*target+1].color = committed[3*target+1].color;
        local[3*target+2].color = committed[3*target+2].color;
    }
}

Eigen::Vector4d get_barycenter(const VertexAttributes &a, const VertexAttributes &b, const VertexAttributes &c) {
    return (a.position+b.position+c.position) / 3;
}

Eigen::Matrix4d get_rotation(double angle) {
    double theta = angle*M_PI / 180;
    Eigen::Matrix4d M;
    M << std::cos(theta), -std::sin(theta), 0, 0,
        std::sin(theta), std::cos(theta), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return M;
}

Eigen::Matrix4d get_scale(double t) {
    Eigen::Matrix4d M;
    M << t, 0, 0, 0,
        0, t, 0, 0,
        0, 0, t, 0,
        0, 0, 0, 1;
    return M;
}

Eigen::Matrix4d get_translate(const Eigen::Vector4d &origin) {
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M.col(3) = -origin;
    M(3,3) = 1;
    return M;
}

int main(int argc, char *args[])
{
    int width = 500;
    int height = 500;

    MODE cur_mode = DEFAULT;
    MODE pre_mode = DEFAULT;

    bool committed = true;
    bool mouse_hold = false;
    bool animation = false;

    int target = -1;

    VertexAttributes cursor, click_pos;
    cursor.color = line_color;

    std::vector<VertexAttributes> keyframe_start, keyframe_end;

    // The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(width, height);

	// Global Constants (empty in this example)
	UniformAttributes uniform;

	// Basic rasterization program
	Program program;

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
        VertexAttributes out;
        out.position = uniform.view_M*va.M * va.position;
        out.color = va.color;
		return out;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		return FragmentAttributes(va.color(0),va.color(1),va.color(2));
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		return FrameBufferAttributes(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
	};

	// One triangle in the center of the screen
	std::vector<VertexAttributes> vertices_committed;
    std::vector<VertexAttributes> vertices_local;
    vertices_committed.push_back(VertexAttributes(-1,-1,0));
    vertices_committed.push_back(VertexAttributes(1,-1,0));
    vertices_committed.push_back(VertexAttributes(0,1,0));
    vertices_committed[0].color << 1,0,0,1;
    vertices_committed[1].color << 0,1,0,1;
    vertices_committed[2].color << 0,0,1,1;

    std::vector<VertexAttributes> lines_committed;
    std::vector<VertexAttributes> lines_local;

    // Initialize the viewer and the corresponding callbacks
    SDLViewer viewer;
    viewer.init("Viewer Example", width, height);

    viewer.mouse_move = [&](int x, int y, int xrel, int yrel){
        cursor.position << (double(x)/double(width) * 2) - 1, (double(height-1-y)/double(height) * 2) - 1, 0, 1;

        if (cur_mode == INSERT) {
            // only redraw when there are changes
            viewer.redraw_next = !committed;
        } else if (cur_mode == TRANSLATION) {
            if (mouse_hold && target != -1) {
                printf("[DRAG] mouse_hold=%d, target=%d\n", mouse_hold, target);
                // update triangle
                vertices_local[3*target].position = cursor.position+vertices_committed[3*target].position-click_pos.position;
                vertices_local[3*target+1].position = cursor.position+vertices_committed[3*target+1].position-click_pos.position;
                vertices_local[3*target+2].position = cursor.position+vertices_committed[3*target+2].position-click_pos.position;
            } else {
                // highlight
                int select = select_triangle(vertices_committed, cursor.position);
                printf("[MOVE] mouse_hold=%d, select=%d, target=%d\n", mouse_hold, select, target);
                highlight_selected(vertices_local, vertices_committed, select);
                if (target != select) {
                    cancel_highlight_selected(vertices_local, vertices_committed, target);
                    target = select;
                }
            }
            committed = false;
            viewer.redraw_next = true;
        }
    };

    viewer.mouse_pressed = [&](int x, int y, bool is_pressed, int button, int clicks) {
        //printf("[EVENT] mouse pressed=(%d, %d), is_pressed=%d, button=%d, clicks=%d\n",x,y,is_pressed,button,clicks);
        mouse_hold = is_pressed;

        // cursor in canonical view space [-1,1]*[-1,1]
        VertexAttributes p;
        p.position << (double(x)/double(width) * 2) - 1, (double(height-1-y)/double(height) * 2) - 1, 0, 1;
        p.color = default_color;

        if (cur_mode == INSERT && is_pressed) {
            p.position = uniform.view_M.inverse() * p.position;
            vertices_local.push_back(p);
            int diff = vertices_local.size() - vertices_committed.size();
            if (diff == 3) {
                commit_changes(vertices_local, vertices_committed);
            }
            committed = false;
            viewer.redraw_next = true;
        } else if (cur_mode == TRANSLATION) {
            if (is_pressed) {
                // mouse click_pos
                int select = select_triangle(vertices_committed, p.position);
                printf("[CLICKED] mouse_hold=%d, select=%d, target=%d\n", mouse_hold, select, target);

                // cancel highlight either select something or nothing
                cancel_highlight_selected(vertices_local, vertices_committed, target);
                target = select;
                click_pos = p;
            } else {
                // mouse released
                printf("[RELEASED] mouse_hold=%d, target=%d\n", mouse_hold, target);
                vertices_local[3*target].position = p.position+vertices_committed[3*target].position-click_pos.position;
                vertices_local[3*target+1].position = p.position+vertices_committed[3*target+1].position-click_pos.position;
                vertices_local[3*target+2].position = p.position+vertices_committed[3*target+2].position-click_pos.position;

                commit_changes(vertices_local, vertices_committed);
                highlight_selected(vertices_local, vertices_committed, target);
            }
            committed = false;
            viewer.redraw_next = true;
        } else if (cur_mode == DELETE) {
            if (is_pressed) {
                target = select_triangle(vertices_local, p.position);
                if (target != -1) {
                    auto begin = std::next(vertices_local.begin(), 3*target);
                    auto end = std::next(begin, 3);
                    vertices_local.erase(begin, end);
                    commit_changes(vertices_local, vertices_committed);
                    committed = false;
                    viewer.redraw_next = true;
                }
            }
        } else if (cur_mode == COLOR && is_pressed) {
            target = select_vertex(vertices_local, p.position);
            printf("[COLOR] select vertex %d\n", target);
            committed = false;
            viewer.redraw_next = true;
        }
    };

    viewer.mouse_wheel = [&](int dx, int dy, bool is_direction_normal) {
        //printf("[EVENT] mouse wheel=(%d, %d), is_direction_normal=%d\n", dx, dy, is_direction_normal);
    };

    viewer.key_pressed = [&](char key, bool is_pressed, int modifier, int repeat) {
//        printf("[EVENT] key pressed=%d, is_pressed=%d, modifier=%d, repeat=%d\n",key,is_pressed,modifier,repeat);

        if (!is_pressed) {
            return; // only consider the push-down
        }
        if (key == 'i') {
            std::cout << "[MODE] INSERT" << std::endl;
            cur_mode = INSERT;
        } else if (key == 'o') {
            std::cout << "[MODE] TRANSLATION" << std::endl;
            cur_mode = TRANSLATION;
        } else if (key == 'p') {
            std::cout << "[MODE] DELETE" << std::endl;
            cur_mode = DELETE;
        } else if (key == 'h' || key == 'j' || key == 'k' || key == 'l') {
            if (cur_mode == TRANSLATION && target != -1) {
                VertexAttributes &a = vertices_local[3*target];
                VertexAttributes &b = vertices_local[3*target+1];
                VertexAttributes &c = vertices_local[3*target+2];
                Eigen::Vector4d barycenter = get_barycenter(a, b, c);
                Eigen::Matrix4d M;
                if (key == 'h') {
                    Eigen::Matrix4d M_r = get_rotation(10);
                    Eigen::Matrix4d M_tr1 = get_translate(barycenter);
                    Eigen::Matrix4d M_tr2 = get_translate(-barycenter);
                    M = M_tr2*M_r*M_tr1;
                } else if (key == 'j') {
                    Eigen::Matrix4d M_r = get_rotation(-10);
                    Eigen::Matrix4d M_tr1 = get_translate(barycenter);
                    Eigen::Matrix4d M_tr2 = get_translate(-barycenter);
                    M = M_tr2*M_r*M_tr1;
                } else if (key == 'k') {
                    Eigen::Matrix4d M_s = get_scale(1.25);
                    Eigen::Matrix4d M_tr1 = get_translate(barycenter);
                    Eigen::Matrix4d M_tr2 = get_translate(-barycenter);
                    M = M_tr2*M_s*M_tr1;
                } else {
                    Eigen::Matrix4d M_s = get_scale(0.75);
                    Eigen::Matrix4d M_tr1 = get_translate(barycenter);
                    Eigen::Matrix4d M_tr2 = get_translate(-barycenter);
                    M = M_tr2*M_s*M_tr1;
                }
                a.M = M*a.M;
                b.M = M*b.M;
                c.M = M*c.M;
                cancel_highlight_selected(vertices_local, vertices_committed, target);
                target = -1;
                commit_changes(vertices_local, vertices_committed);
                committed = false;
                viewer.redraw_next = true;
            }
        } else if (key == 'c') {
            std::cout << "[MODE] COLOR" << std::endl;
            cur_mode = COLOR;
        } else if (key >= '1' && key <= '9' && cur_mode == COLOR && target != -1) {
            if (target >= vertices_local.size()) {
                return;
            }
            vertices_local[target].color = colors[key-'0'];
            committed = false;
            viewer.redraw_next = true;
        } else if (key == '=') { // don't know how to put in '+'
            std::cout << "[VIEW] zoom in 20%" << std::endl;
            Eigen::Matrix4d M = get_scale(1.2);
            uniform.view_M = M*uniform.view_M;
            viewer.redraw_next = true;
        } else if (key == '-') {
            std::cout << "[VIEW] zoom out 20%" << std::endl;
            Eigen::Matrix4d M = get_scale(0.8);
            uniform.view_M = M*uniform.view_M;
            viewer.redraw_next = true;
        } else if (key == 'w') {
            std::cout << "[VIEW] translate down 20%" << std::endl;
            uniform.view_M(1,3) -= 0.4;
            viewer.redraw_next = true;
        } else if (key == 'a') {
            std::cout << "[VIEW] translate right 20%" << std::endl;
            uniform.view_M(0,3) += 0.4;
            viewer.redraw_next = true;
        } else if (key == 's') {
            std::cout << "[VIEW] translate up 20%" << std::endl;
            uniform.view_M(1,3) += 0.4;
            viewer.redraw_next = true;
        } else if (key == 'd') {
            std::cout << "[VIEW] translate down 20%" << std::endl;
            uniform.view_M(0,3) -= 0.4;
            viewer.redraw_next = true;
        } else if (key == 'f') { // choose keyframe
            std::cout << "[ANIMATION] choose keyframe" << std::endl;
            if (keyframe_start.empty()) {
                make_copy(keyframe_end, vertices_local);
            } else {
                make_copy(keyframe_start, vertices_local);
            }
        } else if (key == 'g') { // play keyframe
            std::cout << "[ANIMATION] play keyframe" << std::endl;
            animation = true;
            viewer.redraw_next = true;
        } else if (key == 'q') { // clear keyframes
            std::cout << "[ANIMATION] clear keyframes" << std::endl;
            abort_changes(keyframe_end);
        }
        else {
            std::cout << "[MODE] DEFAULT" << std::endl;
            cur_mode = DEFAULT;
        }
        if (cur_mode != pre_mode) {
            // reset the parameters
            abort_changes(vertices_local);
            make_copy(vertices_local, vertices_committed);
            committed = true;
            mouse_hold = false;
            animation = false;
            abort_changes(keyframe_start);
            abort_changes(keyframe_end);
            target = -1;
            viewer.redraw_next = true;
        }
        pre_mode = cur_mode;
    };

    viewer.redraw = [&](SDLViewer &viewer) {
        // Clear the framebuffer
        for (unsigned i=0;i<frameBuffer.rows();i++)
            for (unsigned j=0;j<frameBuffer.cols();j++)
                frameBuffer(i,j).color << 0,0,0,1;

        std::cout << "[REDRAW] committed=" << committed << std::endl;
        if (committed) {
            rasterize_triangles(program, uniform,vertices_committed, frameBuffer);
            build_lines(vertices_committed, lines_committed);
            rasterize_lines(program, uniform, lines_committed, 0.5, frameBuffer);
        } else {
            rasterize_triangles(program, uniform,vertices_local, frameBuffer);
            build_lines(vertices_local, lines_local);
            rasterize_lines(program, uniform, lines_local, 0.5, frameBuffer);

            if (cur_mode == INSERT) {
                int diff = (vertices_local.size()-vertices_committed.size()) % 3;
                for (int i = vertices_local.size()-diff; i < vertices_local.size(); ++i) {
                    rasterize_line(program, uniform, cursor, vertices_local[i], 0.5, frameBuffer);
                }
            }
        }

        // Buffer for exchanging data between rasterizer and sdl viewer
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> R(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> G(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> B(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> A(width, height);

        for (unsigned i=0; i<frameBuffer.rows();i++)
        {
            for (unsigned j=0; j<frameBuffer.cols();j++)
            {
                R(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(0);
                G(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(1);
                B(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(2);
                A(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(3);
            }
        }
        viewer.draw_image(R, G, B, A);
    };

    viewer.launch(10);

    return 0;
}
