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
struct MultiMissileOptData {
    std::vector<BezierData> missiles;     // 所有弹的数据
    const Point2D* target_point;           // 共同的目标点
    double radius;                         // 目标点周围的圆半径
    double target_arrival_time;            // 目标到达时间(要求一致)
};

const char* nloptResultToString(nlopt::result result);

double checkBezierCurvesIntersection(
    const Point2D& p0_1, const Point2D& p1_1, const Point2D& p2_1, const Point2D& p3_1,
    const Point2D& p0_2, const Point2D& p1_2, const Point2D& p2_2, const Point2D& p3_2,
    int num_samples = 50);

double multi_missile_intersection_constraint(const std::vector<double> &x, std::vector<double> &grad, void *data);

bool loadMissileDataFromFile(
    const std::string& filename, 
    std::vector<bezier::BezierData>& missiles,
    bezier::Point2D& target_point,
    double& radius,
    double& target_length,
    double& rad_min);
}

#endif