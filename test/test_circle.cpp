#include "bezier.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#endif

int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    using namespace bezier;
    // 设置起始点
    bezier::Point2D p0 = {0.0, 0.0};
    
    // 设置目标点
    bezier::Point2D target_point = {100.0, 100.0};
    
    // 设置参数
    double radius = 20.0;             // 目标点周围圆的半径
    double theta0 = bezier::PI / 4;         // 起始航向角 (45度)
    double target_length = 150.0;     // 期望的贝塞尔曲线长度
    double r_min = 30.0;              // 最小转弯半径
    
    // 调用优化函数
    std::tuple<Point2D, Point2D, Point2D, double> result = bezier::findNLoptParameters_Circle(
        p0, target_point, radius, theta0, target_length, r_min);
    
    Point2D p1 = std::get<0>(result);
    Point2D p2 = std::get<1>(result);
    Point2D p3 = std::get<2>(result);
    double error = std::get<3>(result);

    // 输出结果
    std::cout << "优化结果:\n";
    std::cout << "===================================\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "起点P0:      (" << p0[0] << ", " << p0[1] << ")\n";
    std::cout << "控制点P1:    (" << p1[0] << ", " << p1[1] << ")\n";
    std::cout << "控制点P2:    (" << p2[0] << ", " << p2[1] << ")\n";
    std::cout << "终点P3:      (" << p3[0] << ", " << p3[1] << ")\n";
    std::cout << "目标点:      (" << target_point[0] << ", " << target_point[1] << ")\n";
    std::cout << "误差:        " << error << "\n";
    
    // 计算和输出贝塞尔曲线长度和最大曲率
    // double curve_length = bezier::calculateBezierLength(p0, p1, p2, p3, 500);
    // double max_curvature = bezier::findMaxCurvature(p0, p1, p2, p3, 0.01);
    
    // std::cout << "曲线长度:    " << curve_length << " (目标: " << target_length << ")\n";
    // std::cout << "最大曲率:    " << max_curvature << " (最小转弯半径: " << 1.0/max_curvature << ")\n";
    
    // 输出贝塞尔曲线点到文件，方便可视化
    bezier::outputBezierCurvePoints(
        p0, p1, p2, p3, target_point, radius, "bezier_curve.txt", 100);
    
    std::cout << "曲线点已输出到 bezier_curve.txt\n";
    
    return 0;
}