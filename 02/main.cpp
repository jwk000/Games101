// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    model << cos(rotation_angle * MY_PI / 180.0f), -sin(rotation_angle * MY_PI / 180.0f), 0, 0,
        sin(rotation_angle * MY_PI / 180.0f), cos(rotation_angle * MY_PI / 180.0f), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f M_p = Eigen::Matrix4f::Identity();//这个是投影转正交矩阵
    M_p << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, (-1.0 * zNear * zFar),
        0, 0, 1, 0;

    //[l,r]   [b,t]    [f,n]
    float angle = eye_fov * MY_PI / 180;     //求角度
    float t = tan(angle / 2) * abs(zNear);   //更具直角三角形性质求tb（高）
    float b = -1.0 * t;
    float r = t * aspect_ratio;              //根据宽高比求（宽）
    float l = -1.0 * r;

    Eigen::Matrix4f M_s = Eigen::Matrix4f::Identity(); //这个是将立方体进行规范化（-1，1）
    M_s << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f M_t = Eigen::Matrix4f::Identity(); //这里是将三角形位移到原点
    M_t << 1, 0, 0, (-1.0)* (r + l) / 2,
        0, 1, 0, (-1.0)* (t + b) / 2,
        0, 0, 1, (-1.0)* (zNear + zFar) / 2,
        0, 0, 0, 1;
    projection = M_s * M_t * M_p * projection;   //这里是左乘所以是先进行透视转正交，然后位移，然后规范化

    return projection;

}

const int Width = 1000;
const int Height = 1000;
int main(int argc, const char** argv)
{
    float angle = 0;

    Eigen::Vector3f eye_pos = {0,0,5};

    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5},
            };

    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    rst::rasterizer r(Width, Height);

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    while(key != 27)
    {
		r.resize(Width, Height);
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(Width, Height, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(1000);
    }

    return 0;
}
// clang-format on