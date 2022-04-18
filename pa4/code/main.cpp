#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if(control_points.size()==1)    return control_points[0];
    std::vector<cv::Point2f> control_points_next(control_points.size()-1);
    for(int i=0, end=control_points.size()-1; i<end; i++)
    {
        control_points_next[i] = (1-t)*control_points[i] + t*control_points[i+1];
    }
    return recursive_bezier(control_points_next, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for(float t=0; t<1.f; t+=0.001)
    {
        cv::Point2f bt = recursive_bezier(control_points, t);
        std::cout << bt.x << ", " << bt.y << std::endl;
        window.at<cv::Vec3b>(bt.y, bt.x)[2] = 255;
    }
}

void bezier_anti_aliasing(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for(float t=0; t<1.f; t+=0.001)
    {
        cv::Point2f bt = recursive_bezier(control_points, t);
        std::cout << bt.x << ", " << bt.y << std::endl;
        int l = floor(bt.x), r = ceil(bt.x);
        int d = ceil(bt.y), u = floor(bt.y);
        float dist1 = sqrt((l-bt.x)*(l-bt.x)+(d-bt.y)*(d-bt.y));
        float dist2 = sqrt((l-bt.x)*(l-bt.x)+(u-bt.y)*(u-bt.y));
        float dist3 = sqrt((r-bt.x)*(r-bt.x)+(d-bt.y)*(d-bt.y));
        float dist4 = sqrt((r-bt.x)*(r-bt.x)+(u-bt.y)*(u-bt.y));
        window.at<cv::Vec3b>(d, l)[1] = 255*1.5f/(1.5f+dist1);
        window.at<cv::Vec3b>(u, l)[1] = 255*1.5f/(1.5f+dist2);
        window.at<cv::Vec3b>(d, r)[1] = 255*1.5f/(1.5f+dist3);
        window.at<cv::Vec3b>(u, r)[1] = 255*1.5f/(1.5f+dist4);
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
            // bezier(control_points, window);
            bezier_anti_aliasing(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve_anti_aliasing.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
