#ifndef BEZIER_API_HPP
#define BEZIER_API_HPP

#include <vector>
#include <array>
#include <tuple>
#include <nlopt.hpp>

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

    using Point2D = std::array<double, 2>;

    // 单个弹的参数结构
    struct BezierData {
        Point2D start_point;  // 起始位置
        double theta0;        // 起始航向角
        double speed;         // 弹速
        double r_min;         // 最小转弯半径
    };
    
    BEZIER_API std::tuple<Point2D, Point2D, Point2D, double> findNLoptParameters_Circle(
        const Point2D& p0,
        const Point2D& target_point,
        double radius,
        double theta0,
        double target_length,
        double r_min,
        int algorithm = nlopt::LN_COBYLA
    );

    BEZIER_API std::tuple<Point2D, Point2D, Point2D, double> findNLoptParameters_FixedAngle(
        const Point2D& p0,
        const Point2D& target_point,
        double radius,
        double theta0,
        double target_length,
        double r_min,
        double fixed_angle,
        int algorithm = nlopt::LN_COBYLA
    );

    BEZIER_API std::tuple<Point2D, Point2D, Point2D, Point2D, Point2D, double> findNLoptParameters_QuinticFixed(
        const Point2D& p0,
        const Point2D& target_point,
        double radius,
        double theta0,
        double target_length,
        double r_min,
        double fixed_angle,
        const std::vector<double>& lower_bounds = {},
        const std::vector<double>& upper_bounds = {},
        int algorithm = nlopt::LN_COBYLA
    );

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
    
    BEZIER_API bool outputQuinticBezierPoints(
        const Point2D& p0, 
        const Point2D& p1,
        const Point2D& p2, 
        const Point2D& p3,
        const Point2D& p4,
        const Point2D& p5,
        const Point2D& target_point, 
        double r,
        const std::string& filename,
        int num_samples = 100
    );

    BEZIER_API std::vector<std::array<double, 4>> getBezierCurvePoints(
        const Point2D& p0, 
        const Point2D& p1, 
        const Point2D& p2, 
        const Point2D& p3,
        const Point2D& target_point, 
        double radius,
        int num_samples = 100
    );

    BEZIER_API std::vector<std::array<double, 4>> getQuinticBezierPoints(
        const Point2D& p0, 
        const Point2D& p1,
        const Point2D& p2, 
        const Point2D& p3,
        const Point2D& p4,
        const Point2D& p5,
        const Point2D& target_point, 
        double radius,
        int num_samples = 100
    );

    BEZIER_API double optimize_target_length(
        const std::vector<bezier::Point2D>& input_XYZ,
        const bezier::Point2D& target_point,
        double target_radius,
        const std::vector<double>& headings,
        double r_min,
        double min_target_length,
        double max_target_length,
        double initial_target_length
    );
}

#endif // BEZIER_API_HPP