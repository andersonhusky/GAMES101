//
// Created by goksu on 4/6/19.
//

#include <algorithm>
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

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}


// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(int x, int y, const Vector4f* _v){
    Vector3f v[3];
    for(int i=0;i<3;i++)
        v[i] = {_v[i].x(),_v[i].y(), 1.0};
    Vector3f f0,f1,f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);
    Vector3f p(x,y,1.);
    if((p.dot(f0)*f0.dot(v[2])>0) && (p.dot(f1)*f1.dot(v[0])>0) && (p.dot(f2)*f2.dot(v[1])>0))
        return true;
    return false;
}

// 计算任意xy在三角形内有三角形三个点标出时的[alpha, beta, gamma]系数
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList) {

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    // 遍历三角形
    for (const auto& t:TriangleList)
    {
        Triangle newtri = *t;

        // 透视投影前三个顶点坐标
        std::array<Eigen::Vector4f, 3> mm {
                (view * model * t->v[0]),
                (view * model * t->v[1]),
                (view * model * t->v[2])
        };

        // 取mm每个点的前三个给viewspace_pos
        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
            return v.template head<3>();
        });

        // 三角形三个顶点的透视投影
        Eigen::Vector4f v[] = {
                mvp * t->v[0],
                mvp * t->v[1],
                mvp * t->v[2]
        };
        //Homogeneous division
        // 除去第四位得到投影后坐标[x,y,z,1]
        for (auto& vec : v) {
            vec.x()/=vec.w();
            vec.y()/=vec.w();
            vec.z()/=vec.w();
        }

        // 反变换，这里求的n是法向量？
        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[] = {
                inv_trans * to_vec4(t->normal[0], 0.0f),
                inv_trans * to_vec4(t->normal[1], 0.0f),
                inv_trans * to_vec4(t->normal[2], 0.0f)
        };

        //Viewport transformation
        // 转换到像素平面
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        // 设置顶点，转化后在像素平面上
        for (int i = 0; i < 3; ++i)
        {
            //screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        // 设置法向量
        for (int i = 0; i < 3; ++i)
        {
            //view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        // 设置颜色
        newtri.setColor(0, 148,121.0,92.0);
        newtri.setColor(1, 148,121.0,92.0);
        newtri.setColor(2, 148,121.0,92.0);

        // Also pass view space vertice position
        // newtri为经过透视投影后，即缩放空间内的坐标
        // viewspace_pos为未透视投影前，即在为缩放空间内的坐标
        rasterize_triangle(newtri, viewspace_pos);
    }
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos) 
{
    // TODO: From your HW3, get the triangle rasterization code.
    auto v = t.toVector4();
    int boundingBox[4];
    boundingBox[0] = std::floor(std::min(std::min(t.v[0][0], t.v[1][0]), t.v[2][0]));
    boundingBox[1] = std::floor(std::min(std::min(t.v[0][1], t.v[1][1]), t.v[2][1]));
    boundingBox[2] = std::ceil(std::max(std::max(t.v[0][0], t.v[1][0]), t.v[2][0]));
    boundingBox[3] = std::ceil(std::max(std::max(t.v[0][1], t.v[1][1]), t.v[2][1]));
    
    // super-sampling 2*2
    bool MSAA = false;                                                                                                           //Multi-sample Anti-Aliasing（MSAA）
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
                if(num>=2){
                    auto[alpha, beta, gamma] = computeBarycentric2D(float(i)+0.5, float(j)+0.5, t.v);
                    Eigen::Vector3f interpolated_color = interpolate(alpha, beta, gamma, t.getColor(0), t.getColor(1), t.getColor(2), 1);
                    Eigen::Vector3f interpolated_normal = interpolate(alpha, beta, gamma, t.getNormal(0), t.getNormal(1), t.getNormal(2), 1);
                    Eigen::Vector2f interpolated_texcoords = interpolate(alpha, beta, gamma, t.getTexCoord(0), t.getTexCoord(1), t.getTexCoord(2), 1);
                    Eigen::Vector3f interpolated_shadingcoords = interpolate(alpha, beta, gamma, view_pos[0], view_pos[1], view_pos[2], 1);

                    fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
                    payload.view_pos = interpolated_shadingcoords;
                    Eigen::Vector3f pixel_color = fragment_shader(payload);
                    set_pixel(i, j, pixel_color*count/4.0);
                }
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
                    // TODO: Inside your rasterization loop:
                    //    * v[i].w() is the vertex view space depth value z.
                    //    * Z is interpolated view space depth for the current pixel
                    //    * zp is depth between zNear and zFar, used for z-buffer

                    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    // zp *= Z;
                    auto[alpha, beta, gamma] = computeBarycentric2D(float(i)+0.5, float(j)+0.5, t.v);

                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    if(z_interpolated < get_depth(i, j, 0)){
                        // TODO: Interpolate the attributes:
                        // auto interpolated_color
                        // auto interpolated_normal
                        // auto interpolated_texcoords
                        // auto interpolated_shadingcoords
                        Eigen::Vector3f interpolated_color = interpolate(alpha, beta, gamma, t.getColor(0), t.getColor(1), t.getColor(2), 1);
                        Eigen::Vector3f interpolated_normal = interpolate(alpha, beta, gamma, t.getNormal(0), t.getNormal(1), t.getNormal(2), 1);
                        Eigen::Vector2f interpolated_texcoords = interpolate(alpha, beta, gamma, t.getTexCoord(0), t.getTexCoord(1), t.getTexCoord(2), 1);
                        Eigen::Vector3f interpolated_shadingcoords = interpolate(alpha, beta, gamma, view_pos[0], view_pos[1], view_pos[2], 1);

                        fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
                        payload.view_pos = interpolated_shadingcoords;
                        Eigen::Vector3f pixel_color = fragment_shader(payload);
                        set_pixel(i, j, pixel_color);
                        set_depth(i, j, z_interpolated, 0);
                    }
                }
            }
        }
    }
    return;
    // Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
    // Use: payload.view_pos = interpolated_shadingcoords;
    // Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
    // Use: auto pixel_color = fragment_shader(payload);
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
    depth_buf.resize(w * h * AA * AA);

    texture = std::nullopt;
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-y)*width + x;
}

void rst::rasterizer::set_pixel(const Vector2i &point, const Eigen::Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    int ind = (height-point.y())*width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::set_pixel(float x ,float y, const Eigen::Vector3f& color)
{
    auto ind = (height-1-y)*width + (x);
    frame_buf[ind] = color;
}

void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader)
{
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader)
{
    fragment_shader = frag_shader;
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