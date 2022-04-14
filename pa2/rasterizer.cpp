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

// 在三角形内判断
static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    float a = (x-_v[0][0])*(_v[1][1]-_v[0][1]) - (y-_v[0][1])*(_v[1][0]-_v[0][0]);
    float b = (x-_v[1][0])*(_v[2][1]-_v[1][1]) - (y-_v[1][1])*(_v[2][0]-_v[1][0]);
    float c = (x-_v[2][0])*(_v[0][1]-_v[2][1]) - (y-_v[2][1])*(_v[0][0]-_v[2][0]);
    return (a>0 && b>0 && c>0) || (a<0 && b<0 && c<0);
}

// 给的深度差值函数
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    // 遍历三角形，两个三角形所以循环执行两次
    for (auto& i : ind)
    {
        Triangle t;
        // 取三角形三个顶点，经过透视投影
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        // 除去第四位得到投影后坐标[x,y,z,1]
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        // (vert.x()+1.0)把坐标范围转换到[0, 2]
        // *0.5*width转换到像素坐标
        // 深度: 0.1+ 0.5(50-0.1)*(z+1.0) 同上(深度已经转化成正数了)
        // 转化后：(50-0.1)/2*z + (50-0.1)/2 + 0.1
        // 在转化：(50-0.1)/2*z + (50+0.1)/2
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        // 设置三角形顶点
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        // 设置顶点颜色
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
// 三角形栅格化
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    // TODO : Find out the bounding box of current triangle.
    int boundingBox[4];
    boundingBox[0] = std::floor(std::min(std::min(t.v[0][0], t.v[1][0]), t.v[2][0]));
    boundingBox[1] = std::floor(std::min(std::min(t.v[0][1], t.v[1][1]), t.v[2][1]));
    boundingBox[2] = std::ceil(std::max(std::max(t.v[0][0], t.v[1][0]), t.v[2][0]));
    boundingBox[3] = std::ceil(std::max(std::max(t.v[0][1], t.v[1][1]), t.v[2][1]));
    
    // super-sampling 2*2
    

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    bool MSAA = true;                                                                                                           //Multi-sample Anti-Aliasing（MSAA）
    if(MSAA){
        std::vector<Eigen::Vector2f> pos{{0.25, 0.25}, {0.25, 0.75}, {0.75, 0.25}, {0.75, 0.75}};
        for(int i=boundingBox[0]; i<=boundingBox[2]; ++i)
        {
            for(int j=boundingBox[1]; j<=boundingBox[3]; ++j)
            {
                float count=0;
                std::vector<float> samplelist(4, FLT_MAX);
                for(int k=0; k<4; k++)
                {
                    if(insideTriangle(float(i)+pos[k][0], float(j)+pos[k][1], t.v)){
                        auto[alpha, beta, gamma] = computeBarycentric2D(float(i)+pos[k][0], float(j)+pos[k][1], t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        samplelist[k] = z_interpolated;
                        ++count;
                    }
                }

                int num=0;
                for(int k=0; k<4; k++)
                {
                    if(samplelist[k] < get_depth(i, j, k)){
                        ++num;
                        set_depth(i, j, samplelist[k], k);
                    }
                }
                if(num>=2)    set_pixel(i, j, t.getColor()*count/4.0);
            }
        }
    }
    else{
        for(int i=boundingBox[0]; i<=boundingBox[2]; i++)
        {
            for(int j=boundingBox[1]; j<=boundingBox[3]; j++)
            {
                if(insideTriangle(float(i)+0.5, float(j)+0.5, t.v)){
                    // 给的深度插值
                    auto[alpha, beta, gamma] = computeBarycentric2D(float(i)+0.5, float(j)+0.5, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    if(z_interpolated < get_depth(i, j, 0)){
                        Vector3f color = t.getColor();
                        set_pixel(i, j, color);
                        set_depth(i, j, z_interpolated, 0);
                    }
                }
            }
        }
    }
    return;
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

void rst::rasterizer::set_depth(int x ,int y, float depth, int num)
{
    assert(num>=0 && num<=3);
    auto ind = y*height*AA*AA + x*AA*AA + num;
    depth_buf[ind] = depth;
}

float rst::rasterizer::get_depth(int x, int y, int num)
{
    assert(num>=0 && num<=3);
    auto ind = y*height*AA*AA + x *AA*AA + num;
    return depth_buf[ind];
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    set_pixel(point.x(), point.y(), color);
}

// clang-format on
