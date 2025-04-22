#ifndef BEZIER_API_HPP
#define BEZIER_API_HPP

#include <vector>
#include <array>
#include <tuple>
#include <chrono>
#include <iostream>
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

#define BEZIER_USE_PARALLEL 1
#define BEIZER_FIRST_MAXEVAL 500
#define BEIZER_SECOND_MAXEVAL 20

namespace bezier {
    
    constexpr double PI = 3.14159265358979323846;

    using Point2D = std::array<double, 2>;

    // 单个弹的参数结构
    struct NodeData {
        Point2D start_point;  // 起始位置
        double heading;  // 起始航向角
        double speed;  // 弹速
        double r_min;  // 最小转弯半径

        friend std::ostream& operator<<(std::ostream& os, const NodeData& node);
    };

    struct InitData {
        std::vector<NodeData> nodes;  // 弹数据
        Point2D target_point;  // 目标点
        int node_num = 0;  // 弹数

        friend std::ostream& operator<<(std::ostream& os, const InitData& init);
    };

    struct OptParms {
        bool layer = false; // 是否为分层优化
        int opt_type = 0; // 优化类型 3 or 5
        int num_samlpes = 100; // 采样点数
        double target_length = 0; // 期望长度
        double target_radius = 0; // 目标半径
        double fixed_angle = 0;   // 固定角度
        nlopt::algorithm algo_first = nlopt::LN_COBYLA;
        nlopt::algorithm algo_second = nlopt::LN_COBYLA;
        std::vector<double> lower_bounds_first = {};
        std::vector<double> upper_bounds_first = {};
        std::vector<double> x_init_first = {};
        std::vector<double> lower_bounds_second = {};
        std::vector<double> upper_bounds_second = {};
        std::vector<double> x_init_second = {};

        friend std::ostream& operator<<(std::ostream& os, const OptParms& opt);
    };
    
    BEZIER_API std::tuple<Point2D, Point2D, Point2D, double> findNLoptParameters_Circle(
        const Point2D& p0,
        const Point2D& target_point,
        double radius,
        double theta0,
        double target_length,
        double r_min,
        nlopt::algorithm algo = nlopt::LN_COBYLA
    );

    BEZIER_API std::tuple<Point2D, Point2D, Point2D, double> findNLoptParameters_FixedAngle(
        const Point2D& p0,
        const Point2D& target_point,
        double radius,
        double theta0,
        double target_length,
        double r_min,
        double fixed_angle,
        const std::vector<double>& lower_bounds = {},
        const std::vector<double>& upper_bounds = {},
        const std::vector<double>& init_x = {},
        nlopt::algorithm algo = nlopt::LN_COBYLA,
        bool cout_flag = true 
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
        const std::vector<double>& init_x = {},
        nlopt::algorithm algo = nlopt::LN_COBYLA,
        bool cout_flag = true
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

    BEZIER_API std::vector<std::array<double, 4>> generateBezierPath(
        const InitData& init,
        const OptParms& opt
    );

    BEZIER_API std::vector<std::array<double, 4>> generateGeoPath(
        const InitData& init_geo,
        const OptParms& opt
    );

    /**
     * @brief output_dir likes "../.." or "." or "../"
     */
    BEZIER_API bool outputMultiPathPoints(
        const std::vector<std::array<double, 4>>& all_path_points,
        const Point2D& target_point,
        double target_radius,
        int points_per_node,
        const std::string& output_dir
    );

    template<typename Func>
    auto measureTime(Func func, std::string func_name) -> decltype(func()) {
        auto start = std::chrono::high_resolution_clock::now();
        auto result = func();
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << func_name << " run: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000 << " ms" << std::endl;
        return result;
    };
}

#endif // BEZIER_API_HPP