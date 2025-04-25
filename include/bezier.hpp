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
#include <map>
#include <mutex>
#include <omp.h>

namespace bezier {

// 优化问题的数据结构
struct OptData {
    const Point2D* p0;
    const Point2D* target_point;
    double radius;
    double theta0;
    double target_length;
    double r_min;
    double fixed_angle;  // 固定角度
};

struct LayerData {
    const std::vector<NodeData>* nodes;  // 所有节点
    const Point2D* target_point;
    double target_radius;
    int nodes_num;
    nlopt::algorithm algo_first;
    const std::vector<double>* lower_bounds_first;
    const std::vector<double>* upper_bounds_first;
    const std::vector<double>* x_init_first;
    int opt_type;
};

// 先添加新的数据结构用于单层优化
struct UnifiedData {
    const std::vector<NodeData>* nodes;
    int nodes_num;
    const Point2D* target_point;
    double target_radius;
    int params_per_node;
    int opt_type;
};

double calculateBezierLength(
    const Point2D& p0, 
    const Point2D& p1, 
    const Point2D& p2, 
    const Point2D& p3, 
    int num_samples = 100
);

double calculateCurvatureAtPoint(
    double t,
    const Point2D& p0, 
    const Point2D& p1, 
    const Point2D& p2, 
    const Point2D& p3
);

double findMaxCurvature(
    const Point2D& p0, 
    const Point2D& p1, 
    const Point2D& p2, 
    const Point2D& p3, 
    double dt = 0.01
);

double findMaxQuinticCurvature(
    const Point2D& p0,
    const Point2D& p1,
    const Point2D& p2,
    const Point2D& p3,
    const Point2D& p4,
    const Point2D& p5,
    double step);

double calculateQuinticBezierLength(
    const Point2D& p0, 
    const Point2D& p1, 
    const Point2D& p2,
    const Point2D& p3,
    const Point2D& p4,
    const Point2D& p5,
    int steps);

InitData convertGeoToLocal(const InitData& init_geo);

std::vector<std::array<double, 4>> convertLocalToGeo(
    const std::vector<std::array<double, 4>>& path_local, 
    const Point2D& origin_geo);

const char* nloptAlgorithmToString(nlopt::algorithm alg);

const char* nloptResultToString(nlopt::result result);

}

#endif