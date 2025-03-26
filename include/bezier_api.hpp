#ifndef BEZIER_API_HPP
#define BEZIER_API_HPP

#include <vector>
#include <array>
#include <tuple>
#include <nlopt.hpp>

using Point2D = std::array<double, 2>;

#if defined(_WIN32) || defined(_WIN64)
    #ifdef BEZIER_EXPORTS
        #define BEZIER_API __declspec(dllexport)
    #else
        #define BEZIER_API __declspec(dllimport)
    #endif
#else
    #define BEZIER_API __attribute__((visibility("default")))
#endif

namespace bezier {
    
    // 计算贝塞尔曲线长度
    BEZIER_API double calculateBezierLength(
        const Point2D& p0, 
        const Point2D& p1, 
        const Point2D& p2, 
        const Point2D& p3, 
        int num_samples = 100
    );

    // 计算贝塞尔曲线上某点的曲率
    BEZIER_API double calculateCurvatureAtPoint(
        double t,
        const Point2D& p0, 
        const Point2D& p1, 
        const Point2D& p2, 
        const Point2D& p3
    );
    
    // 查找最大曲率值
    BEZIER_API double findMaxCurvature(
        const Point2D& p0, 
        const Point2D& p1, 
        const Point2D& p2, 
        const Point2D& p3, 
        double dt = 0.01
    );
    
    // 优化函数 - 返回控制点
    BEZIER_API std::tuple<Point2D, Point2D, Point2D> findOptimalParameters_Circle(
        const Point2D& p0,
        const Point2D& target_point,
        double radius,
        double theta0,
        double target_length,
        double r_min
    );
    
    // 输出贝塞尔曲线点到文件
    BEZIER_API bool outputBezierCurvePoints(
        const Point2D& p0, 
        const Point2D& p1, 
        const Point2D& p2, 
        const Point2D& p3,
        const Point2D& target_point, 
        double radius,
        const std::string& filename,
        int num_samples = 100
    );
    
    // 优化函数（基于nlopt） - 返回控制点
    BEZIER_API std::tuple<Point2D, Point2D, Point2D> findNLoptParameters_Circle(
        const Point2D& p0,
        const Point2D& target_point,
        double radius,
        double theta0,
        double target_length,
        double r_min,
        int algorithm = nlopt::LN_COBYLA  // 默认为COBYLA
    );
}

#endif // BEZIER_API_HPP