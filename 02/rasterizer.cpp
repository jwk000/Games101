// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    Vector3f p(x, y, 1.0f);
    Vector3f v[3] = { p - _v[0],p - _v[1],p - _v[2] };
	Vector3f e[3] = { _v[1] - _v[0],_v[2] - _v[1],_v[0] - _v[2] };
	auto z01 = e[0].cross(v[0]);
    auto z12 = e[1].cross(v[1]);
	auto z20 = e[2].cross(v[2]);
    if (z01.z() > 0 && z12.z() > 0 && z20.z() > 0) {
        return true;
    }
	if (z01.z() < 0 && z12.z() < 0 && z20.z() < 0) {
		return true;
	}
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = { 255, 255, 255 };

    int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

    dx = x2 - x1;
    dy = y2 - y1;
    dx1 = fabs(dx);
    dy1 = fabs(dy);
    px = 2 * dy1 - dx1;
    py = 2 * dx1 - dy1;

    if (dy1 <= dx1)
    {
        if (dx >= 0)
        {
            x = x1;
            y = y1;
            xe = x2;
        }
        else
        {
            x = x2;
            y = y2;
            xe = x1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(x,y, line_color);
        for (i = 0; x < xe; i++)
        {
            x = x + 1;
            if (px < 0)
            {
                px = px + 2 * dy1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    y = y + 1;
                }
                else
                {
                    y = y - 1;
                }
                px = px + 2 * (dy1 - dx1);
            }
            //            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(x,y, line_color);
        }
    }
    else
    {
        if (dy >= 0)
        {
            x = x1;
            y = y1;
            ye = y2;
        }
        else
        {
            x = x2;
            y = y2;
            ye = y1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(x,y, line_color);
        for (i = 0; y < ye; i++)
        {
            y = y + 1;
            if (py <= 0)
            {
                py = py + 2 * dx1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    x = x + 1;
                }
                else
                {
                    x = x - 1;
                }
                py = py + 2 * (dx1 - dy1);
            }
            //            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(x,y, line_color);
        }
    }
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
			vert.z() = (vert.z() + 1.0f) * 0.5f; // Map z from [-1, 1] to [0, 1]
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& tr) {
    auto v = tr.toVector4();

    float l, r, t, b;
	l = std::min({ v[0].x(), v[1].x(), v[2].x() });
	r = std::max({ v[0].x(), v[1].x(), v[2].x() });
	t = std::max({ v[0].y(), v[1].y(), v[2].y() });
	b = std::min({ v[0].y(), v[1].y(), v[2].y() });

	float offset[4][2] = { {0.25, 0.25},
						{0.75, 0.25},
						{0.25, 0.75},
						{0.75, 0.75} };
    //x,y必须取整，否则计算图像坐标的时候会出现坐标位置偏移！！！
    for (int x = (int)l; x < r; x++) {
        for (int y = (int)b; y < t; y++) {
            bool inside = false;
            for (int i = 0; i < 4; i++) {
                if (insideTriangle(x + offset[i][0], y + offset[i][1], tr.v)) {
                    auto [alpha, beta, gamma] = computeBarycentric2D(x + offset[i][0], y + offset[i][1], tr.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    //三角形边缘出现黑边是因为直接用了背景色平均，没有考虑其他三角形的颜色
                    //只考虑一个三角形无法处理一个像素的4个虚拟像素分布在不同三角形内的情况
                    //为什么要判断每个虚拟像素的深度？因为虚拟像素的颜色可能被其他三角形覆盖了
                    if (msaa_depth_buf[get_index(x, y)*4+i] < z_interpolated) {
                        msaa_depth_buf[get_index(x, y)*4+i] = z_interpolated;
                        msaa_frame_buf[get_index(x, y) * 4 + i] = tr.getColor();
                        inside = true;
                    }
                }
            }
            if (inside) {
				Eigen::Vector3f color = (msaa_frame_buf[get_index(x, y) * 4 + 0]+
					msaa_frame_buf[get_index(x, y) * 4 + 1] +
					msaa_frame_buf[get_index(x, y) * 4 + 2] +
					msaa_frame_buf[get_index(x, y) * 4 + 3]) / 4.0f; // Average color for MSAA
                set_pixel(x, y, color);
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
		std::fill(msaa_frame_buf.begin(), msaa_frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), 0);
		std::fill(msaa_depth_buf.begin(), msaa_depth_buf.end(), 0);
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    msaa_frame_buf.resize(w * h * 4);
	msaa_depth_buf.resize(w * h * 4);
}

void rst::rasterizer::resize(int w, int h) {
	width = w;
	height = h;
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);
    msaa_frame_buf.resize(w * h * 4);
    msaa_depth_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    //return x + y * width;
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(int x, int y, const Eigen::Vector3f& color)
{
   //auto ind = point.x() + point.y() * width;
    auto ind = (height-1-y)*width + x;
	if (ind < 0 || ind >= frame_buf.size()) {
		return; // Out of bounds, do not set pixel
	}
    frame_buf[ind] = color;

}

Eigen::Vector3f  rst::rasterizer::get_pixel(int x, int y)
{
	//auto ind = point.x() + point.y() * width;
	auto ind = (height - 1 - y) * width + x;
	if (ind < 0 || ind >= frame_buf.size()) {
		return Eigen::Vector3f{ 0, 0, 0 }; // Return black if out of bounds
	}
	return frame_buf[ind];
}

// clang-format on