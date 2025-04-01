#ifndef BEZIER_HPP
#define BEZIER_HPP

#include "bezier_api.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <limits>
#include <algorithm>

namespace bezier {

constexpr double PI = 3.14159265358979323846;

// 优化问题的数据结构
struct OptData {
    const Point2D* p0;
    const Point2D* target_point;
    double radius;
    double theta0;
    double target_length;
    double r_min;
};

// 多弹优化问题的数据结构
struct OverallOptData {
    const std::vector<bezier::Point2D>* input_XYZ;  // 输入点列表
    const bezier::Point2D* target_point;            // 目标点
    double target_radius;                           // 目标圆半径
    const std::vector<double>* headings;            // 航向角列表
    double r_min;                                   // 最小转弯半径
};

const char* nloptResultToString(nlopt::result result);

}

#endif