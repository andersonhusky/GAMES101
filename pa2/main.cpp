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

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
                    
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it
    Eigen::Matrix4f Mpersp;
    float fovY = eye_fov*MY_PI/180.0;
    float cota = 1.f/tanf(fovY/2);
    float zD = zNear-zFar;
    Mpersp << -cota/aspect_ratio, 0, 0, 0,
                        0, -cota, 0, 0,
                        0, 0, (zNear+zFar)/zD, -2*zNear*zFar/zD,
                        0, 0, 1, 0;

    // Eigen::Matrix4f Mpersp, Mscale, Mtrans, MP2O;
    // float fovY = eye_fov*PI/180.0;
    // float t = abs(zNear)*tanf(fovY/2);
    // float r = t*aspect_ratio;

    // Mscale << 1/r, 0, 0, 0,
    //                         0, 1/t, 0, 0,
    //                         0, 0, 2/(zNear-zFar), 0,
    //                         0, 0, 0, 1;
    // Mtrans << 1, 0, 0, 0,
    //                     0, 1, 0, 0,
    //                     0, 0, 1, -(zNear+zFar)/2,
    //                     0, 0, 0, 1;
    // MP2O << zNear, 0, 0, 0,
    //                 0, zNear, 0, 0,
    //                 0, 0, zNear+zFar, -zNear*zFar,
    //                 0, 0, 1, 0;
    // Mpersp = Mscale*Mtrans*MP2O;

    return Mpersp; 
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

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
                    {3, 4, 5}
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

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on
