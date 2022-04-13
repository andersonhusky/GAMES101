#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Vector3f f = -eye_pos;
    f.normalize();
    //左手坐标系begin
//    Eigen::Vector3f u = {0,1,0};
//    Eigen::Vector3f r = u.cross(f);
//    view << r.x(),r.y(),r.z(), 0,
//            u.x(),u.y(),u.z(), 0,
//            f.x(),f.y(),f.z(),0,
//            0, 0, 0, 1;
//    Eigen::Matrix4f translate;
//    translate << 1, 0, 0, -eye_pos[0],
//                0, 1, 0, -eye_pos[1],
//                0, 0, 1,-eye_pos[2],
//                0, 0, 0, 1;
    //左手坐标系end
    
    //右手坐标系begin
    Eigen::Vector3f u = {0,1,0};
    Eigen::Vector3f r = f.cross(u);
    view << r.x(),r.y(),r.z(), 0,
            u.x(),u.y(),u.z(), 0,
            -f.x(),-f.y(),-f.z(),0,
            0, 0, 0, 1;
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                0, 1, 0, -eye_pos[1],
                0, 0, 1,-eye_pos[2],
                0, 0, 0, 1;
    //右手坐标系end
    view =   view * translate;

    return view;
}

Eigen::Matrix4f toMatrix4f(Eigen::Matrix3f m)
{
    Eigen::Matrix4f tRet = Eigen::Matrix4f::Zero();
    tRet.block<3,3>(0,0) = m;
    tRet.row(3) = (Vector4f){0,0,0,1};
    return tRet;
}
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f N;
    float a = angle/180.0f*PI;
    float c = cosf(a),s = sinf(a);
    Vector3f n = axis.normalized();
    N << 0,     -n.z(),  n.y(),
         n.z(),  0,     -n.x(),
        -n.y(), n.x(), 0;
    //wikipedia的方法
    return toMatrix4f(I + (N*N)*(1-c) + N*s);
    //闫老师的方法 n*n^t
    //return toMatrix4f(c*I + (1-c)*(n*n.transpose()) + s*N);
}
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Vector3f z = {0,1,0};
    Eigen::Matrix4f t;
    t << 1, 0, 0, 0,
                0, 1, 0, -0.5,
                0, 0, 1,-2,
                0, 0, 0, 1;
    auto r = get_rotation(z,rotation_angle);
    
    return t * r ;
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float a = rotation_angle/180.0f*PI;
    float c = cosf(a),s = sinf(a);
    model << c,-s,0,0,
             s, c,0,0,
             0, 0,1,0,
             0, 0,0,1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float cota = eye_fov / 180.0f * 3.1415926f * 0.5f ;
    cota = 1.f/ tanf(cota);
    //右手坐标系
    float zD = zNear-zFar;
    projection <<   -cota/aspect_ratio,0,0,0,
                    0,-cota,0,0,
                    0,0,(zFar+zNear)/zD,-2.0f*zFar*zNear/zD,
                    0,0,1,0;
    
    //左手坐标系
//    projection <<   cota/aspect_ratio,0,0,0,
//                    0,cota,0,0,
//                    0,0,(zFar+zNear)/(zFar-zNear),-2.0f*zFar*zNear/(zFar-zNear),
//                    0,0,1,0;
                    
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
//    projection <<   0.2,0,0,0,
//                    0,0.2,0,0,
//                    0,0,-0.5,0.5,
//                    0,0,0,1;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, 0}, {0, 2, 0}, {-2, 0, 0}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
//        Eigen::Vector3f red (255.0f,0.0f,0.0f);
//        Eigen::Vector3f center(100,100,0);
//        for(int i  =0;i<10;++i){
//            for(int j = 0;j<10;++j)
//            {
//                Eigen::Vector3f p = center;
//                p.x() = p.x() + i;
//                p.y() = p.y() + j;
//                r.set_pixel(p,red );
//            }
//        }
        
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        //std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
