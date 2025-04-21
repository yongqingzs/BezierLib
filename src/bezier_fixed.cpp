#include "bezier.hpp"

namespace bezier {

double objective_function_fixed_angle(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
    OptData* opt_data = reinterpret_cast<OptData*>(data);

    // x[0]: d0 - 第一控制点距离
    // x[1]: d3 - 最后控制点距离

    double angle = opt_data->fixed_angle;
    double target_x = opt_data->target_point->at(0);
    double target_y = opt_data->target_point->at(1);

    // 计算终点位置（在固定角度的圆上）
    Point2D p3 = {
        target_x + opt_data->radius * std::cos(angle),
        target_y + opt_data->radius * std::sin(angle)
    };

    // 计算控制点
    Point2D p1 = {
        opt_data->p0->at(0) + x[0] * std::cos(opt_data->theta0),
        opt_data->p0->at(1) + x[0] * std::sin(opt_data->theta0)
    };

    Point2D p2 = {
        p3[0] + x[1] * std::cos(angle),
        p3[1] + x[1] * std::sin(angle)
    };

    // 计算曲线长度
    double curve_length = calculateBezierLength(*(opt_data->p0), p1, p2, p3, 500);

    return std::fabs(curve_length - opt_data->target_length);
}

/** 曲率约束函数：确保最大曲率不超过限制（固定角度版本）
*/
double constraint_function_fixed_angle(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
    OptData* opt_data = reinterpret_cast<OptData*>(data);

    // x[0]: d0 - 第一控制点距离
    // x[1]: d3 - 最后控制点距离

    double angle = opt_data->fixed_angle;
    double target_x = opt_data->target_point->at(0);
    double target_y = opt_data->target_point->at(1);

    // 计算终点位置（在固定角度的圆上）
    Point2D p3 = {
        target_x + opt_data->radius * std::cos(angle),
        target_y + opt_data->radius * std::sin(angle)
    };

    // 计算控制点
    Point2D p1 = {
        opt_data->p0->at(0) + x[0] * std::cos(opt_data->theta0),
        opt_data->p0->at(1) + x[0] * std::sin(opt_data->theta0)
    };

    Point2D p2 = {
        p3[0] + x[1] * std::cos(angle),
        p3[1] + x[1] * std::sin(angle)
    };

    // 计算最大曲率
    double max_curvature = findMaxCurvature(*(opt_data->p0), p1, p2, p3, 0.01);

    return max_curvature - 1.0 / opt_data->r_min;
}

BEZIER_API std::tuple<Point2D, Point2D, Point2D, double> findNLoptParameters_FixedAngle(
    const Point2D& p0,
    const Point2D& target_point,
    double radius,
    double theta0,
    double target_length,
    double r_min,
    double fixed_angle,
    const std::vector<double>& lower_bounds,
    const std::vector<double>& upper_bounds,
    const std::vector<double>& init_x,
    int algorithm,
    bool cout_flag)
{
    // 选择算法
    nlopt::algorithm algo;
    switch (algorithm) {
    case 1: algo = nlopt::LN_BOBYQA; break;
    case 2: algo = nlopt::LN_NELDERMEAD; break;
    case 3: algo = nlopt::LN_SBPLX; break;
    default: algo = nlopt::LN_COBYLA; break;
    }

    OptData opt_data;
    opt_data.p0 = &p0;
    opt_data.target_point = &target_point;
    opt_data.radius = radius;
    opt_data.theta0 = theta0;
    opt_data.target_length = target_length;
    opt_data.r_min = r_min;

    nlopt::opt optimizer(algo, 2);

    std::vector<double> lower_bounds_actual(2);
    std::vector<double> upper_bounds_actual(2);
    if (lower_bounds.size() == 2 && upper_bounds.size() == 2) {
        lower_bounds_actual = lower_bounds;
        upper_bounds_actual = upper_bounds;
    } else {
        // 默认边界设置
        // d0 范围 (0 到 target_length)
        lower_bounds_actual[0] = 10.0;
        upper_bounds_actual[0] = target_length;
        // d3 范围 (0 到 target_length)
        lower_bounds_actual[1] = 10.0;
        upper_bounds_actual[1] = target_length;
    }

    optimizer.set_lower_bounds(lower_bounds_actual);
    optimizer.set_upper_bounds(upper_bounds_actual);

    optimizer.set_min_objective(objective_function_fixed_angle, &opt_data);
    optimizer.add_inequality_constraint(constraint_function_fixed_angle, &opt_data, 1e-8);

    optimizer.set_xtol_rel(1e-4);  // 相对误差 所有参数在连续迭代之间的相对变化都小于 0.01% 时
    optimizer.set_maxeval(BEIZER_FIRST_MAXEVAL);    // 最大评估次数

    // 设置初始猜测值（d0和d3为中等值）
    std::vector<double> x(2);
    x[0] = target_length / 4;  // d0初始值
    x[1] = target_length / 4;  // d3初始值

    // 存储最优函数值
    double min_error;

    // 执行优化
    nlopt::result result = optimizer.optimize(x, min_error);

    // 计算最优解的终点和控制点
    double best_d0 = x[0];
    double best_d3 = x[1];

    double target_x = target_point[0];
    double target_y = target_point[1];

    // 返回结果
    Point2D best_p3 = {
        target_x + radius * std::cos(fixed_angle),
        target_y + radius * std::sin(fixed_angle)
    };

    Point2D p1 = {
        p0[0] + best_d0 * std::cos(theta0),
        p0[1] + best_d0 * std::sin(theta0)
    };

    Point2D p2 = {
        best_p3[0] + best_d3 * std::cos(fixed_angle),
        best_p3[1] + best_d3 * std::sin(fixed_angle)
    };

    if (cout_flag) {
        std::cout << "3 order nlopt result: " << nloptResultToString(result) << std::endl;
        std::cout << "Quintic Bezier min_error = " << min_error << std::endl;
    }

    return std::make_tuple(p1, p2, best_p3, min_error);
}
}
