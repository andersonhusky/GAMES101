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




static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

static bool insideTriangle(int x, int y, const Vector3f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    //Vector3f p(x,y,)
    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, _v);
    return alpha >=0 && beta >=0 && gamma >=0;
    //return false;
}
void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

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
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
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
void* rst::rasterizer::data()
{
    return frame_buf.data();
}
//
//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto vs = t.toVector4();
    int left = width;
    int right = 0;
    int down = height;
    int up =0;
    for (Vector4f v : vs) {
        if(v.x() < left) left = std::max(v.x(),0.0f);
        if(v.x() > right) right = std::min(v.x(),(float)width);
        if(v.y() < down) down = std::max(v.y(),0.0f);
        if(v.y() > up) up = std::min(v.y(), (float)height);
    }
    for(int y_ = down;y_<up;++y_)
    {
        for(int x_ = left;x_<right;++x_)
        {
            int coverageCount = 0;
            //anti aliasing
            
            for(int xA=0;xA<AA;++xA)
            {
                for (int yA=0; yA<AA; ++yA) {
                    float x = x_ + 1.0f/AA*xA + 0.5f/AA;
                    float y = y_ + 1.0f/AA*yA + 0.5f/AA;
                    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    if (alpha >=0 && beta >=0 && gamma >=0) {
                        
                        float oldz = get_depth(x, y);
                        float newDepth = alpha * t.v[0].z() + beta * t.v[1].z() * gamma * t.v[2].z();
                        if(newDepth < oldz){
                            ++coverageCount;
                            set_depth(x, y, newDepth);
                            //set_pixel(x, y,t.getColor());
                        }
                        
                    }
                }
            }
            
            if(coverageCount > 0)
            {
                Eigen::Vector3f color = t.getColor();
                float coverage = 1.0f/(AA*AA) * coverageCount;
                set_pixel(x_, y_,color*coverage);
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
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h*AA*AA);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(float x ,float y, const Eigen::Vector3f& color)
{
    auto ind = (height-1-y)*width + (x);
    frame_buf[ind] = color;
}

void rst::rasterizer::set_depth(float x ,float y, float depth)
{
    auto ind = (height*AA-1-y*AA)*width*AA + (x*AA);
    depth_buf[ind] = depth;

}
float rst::rasterizer::get_depth(float x, float y)
{
    auto ind = (height*AA-1-y*AA)*width*AA + (x*AA);
    return depth_buf[ind];
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    set_pixel(point.x(), point.y(), color);
}

// clang-format on
