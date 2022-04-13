#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "kbhit.cpp"

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

// 角轴转换为旋转矩阵
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    float a = angle*PI/180.0;
    float cosa = cosf(a), sina = sinf(a);
    Eigen::Matrix3f  I = Eigen::Matrix3f::Identity();
    axis = axis.normalized();
    Eigen::Matrix3f nhat;
    nhat << 0, -axis.z(), axis.y(),
                axis.z(), 0, -axis.x(),
                -axis.y(), axis.x(), 0;
    return toMatrix4f(I + (nhat*nhat)*(1-cosa) + sina*nhat);
    // return toMatrix4f(cosa*I + (1-cosa)*(axis*axis.transpose()) + sina*nhat);
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f m;
    Vector3f axis{0, 0, 1};
    Eigen::Matrix4f t;
    t << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    m = get_rotation(axis, rotation_angle);
    return m*t;

    // Eigen::Matrix4f m;
    // float angle = rotation_angle/180.0f * PI;
    // float c = cosf(angle),s = sinf(angle);
    // m << c, -s, 0, 0,
    //     s, c, 0, 0,
    //     0, 0, 1, 0,
    //     0, 0, 0, 1;
    // return m;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
                    
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f Mpersp, MOtrtho, MP2O;

    float fovY = float(eye_fov)*PI/180.0;
    float t = abs(zNear)*tan(fovY/2);
    float r = t*aspect_ratio;

    MOtrtho << 1/r, 0, 0, 0,
                            0, 1/t, 0, 0,
                            0, 0, 2/(zFar-zNear), -(zFar+zNear)/2,
                            0, 0, 0, 1;
    MP2O << zNear, 0, 0, 0,
                    0, zNear, 0, 0,
                    0, 0, zNear+zFar, -zNear*zFar,
                    0, 0, 1, 0;
    Mpersp = MOtrtho*MP2O;
    return Mpersp; 
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    // 命令行输入的第三个为旋转角度，第四个为存储的图片名
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
        
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);
        
        //  AD旋转
        if(kbhit()){
            char action = getchar();
            if(action=='A') angle += 3;
            else if(action=='D')    angle -=3;
        }
    }

    return 0;
}
