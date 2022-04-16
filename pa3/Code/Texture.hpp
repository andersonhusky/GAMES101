//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    // 得到u、v处纹理像素值
    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float u_img = u * float(width);
        float v_img = (1 - v) * float(height);
        int u_u = ceil(u_img), u_d=floor(u_img);
        int v_u = ceil(v_img), v_d = floor(v_img);
        float s = u_img-u_d;
        float t = v_u-v_img;

        auto u0 = (1-s)*image_data.at<cv::Vec3b>(v_u, u_d) + s*image_data.at<cv::Vec3b>(v_u, u_u);
        auto u1 = (1-s)*image_data.at<cv::Vec3b>(v_d, u_d) + s*image_data.at<cv::Vec3b>(v_d, u_u);
        auto color = (1-t)*u0 + t*u1;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
