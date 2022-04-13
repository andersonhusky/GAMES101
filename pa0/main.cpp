//#include<math>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
const float PI = 3.1415926f;
int main(){
    Eigen::Vector3f p(2,1,1);
    Eigen::Matrix3f m;
    float angle = 45.0f/180.0f * PI;
    float c = cosf(angle),s = sinf(angle);
    // Eigen矩阵初始化 可以用<<的形式
    m << c,-s,1,
         s,c,2,
        0,0,1;
    p = m * p;
    std::cout << p;
    return 0;
}
