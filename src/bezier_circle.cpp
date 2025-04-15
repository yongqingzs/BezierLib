#include "bezier.hpp"

namespace bezier {

/** 基于控制点计算贝塞尔曲线长度
 */
double calculateBezierLength(
    const Point2D& p0, 
    const Point2D& p1,
    const Point2D& p2, 
    const Point2D& p3, 
    int num_samples)
{
    double length = 0.0;
    double dt = 1.0 / (num_samples - 1);
    double t = 0.0;
    double x_prev = p0[0];
    double y_prev = p0[1];

    // 对曲线进行采样并计算长度
    for (int i = 1; i < num_samples; i++) {
        t += dt;

        // 三次贝塞尔曲线公式
        double B0 = std::pow(1-t, 3);
        double B1 = 3 * t * std::pow(1-t, 2);
        double B2 = 3 * t * t * (1-t);
        double B3 = std::pow(t, 3);

        double x_curr = B0 * p0[0] + B1 * p1[0] + B2 * p2[0] + B3 * p3[0];
        double y_curr = B0 * p0[1] + B1 * p1[1] + B2 * p2[1] + B3 * p3[1];
        double dx = x_curr - x_prev;
        double dy = y_curr - y_prev;
        length += std::sqrt(dx*dx + dy*dy);
        x_prev = x_curr;
        y_prev = y_curr;
    }

    return length;
}

/** 计算贝塞尔曲线上某点的曲率（根据t找点）
 */
double calculateCurvatureAtPoint(
    double t, 
    const Point2D& p0, 
    const Point2D& p1,
    const Point2D& p2, 
    const Point2D& p3)
{
    // 控制点坐标
    double x0 = p0[0], y0 = p0[1];
    double x1 = p1[0], y1 = p1[1];
    double x2 = p2[0], y2 = p2[1];
    double x3 = p3[0], y3 = p3[1];

    // 一阶导数系数
    double dx0 = 3 * (x1 - x0);
    double dx1 = 3 * (x2 - x1);
    double dx2 = 3 * (x3 - x2);
    double dy0 = 3 * (y1 - y0);
    double dy1 = 3 * (y2 - y1);
    double dy2 = 3 * (y3 - y2);

    // 二阶导数系数
    double ddx0 = 6 * (x2 - 2*x1 + x0);
    double ddx1 = 6 * (x3 - 2*x2 + x1);
    double ddy0 = 6 * (y2 - 2*y1 + y0);
    double ddy1 = 6 * (y3 - 2*y2 + y1);

    // 计算一阶导数
    double x_prime = dx0 * std::pow(1-t, 2) + dx1 * 2*t*(1-t) + dx2 * std::pow(t, 2);
    double y_prime = dy0 * std::pow(1-t, 2) + dy1 * 2*t*(1-t) + dy2 * std::pow(t, 2);

    // 计算二阶导数
    double x_double_prime = ddx0 * (1-t) + ddx1 * t;
    double y_double_prime = ddy0 * (1-t) + ddy1 * t;

    // 计算曲率
    double numerator = std::fabs(x_prime * y_double_prime - y_prime * x_double_prime);
    double denominator = std::pow(x_prime * x_prime + y_prime * y_prime, 1.5);

    if (denominator < 1e-10) {
        return 0.0;
    }

    return numerator / denominator;
}

/** 寻找贝塞尔曲线的最大曲率
 */
double findMaxCurvature(
    const Point2D& p0, 
    const Point2D& p1,
    const Point2D& p2, 
    const Point2D& p3, 
    double dt)
{
    double max_curvature = 0.0;
    double t_at_max = 0.0;
    int num_samples = int(1 / dt);
    
    for (int i = 0; i < num_samples; i++) {
        double t = i * dt;
        double curvature = calculateCurvatureAtPoint(t, p0, p1, p2, p3);

        if (curvature > max_curvature) {
            max_curvature = curvature;
            t_at_max = t;
        }
    }

    return max_curvature;
}

/**
 * 在目标点固定距离圆上寻找最优终点和控制点参数（带自适应步长）
 * 
 * @param p0 起始点
 * @param target_point 目标点
 * @param radius 目标点所在圆半径
 * @param theta0 起始航向角
 * @param target_length 目标曲线长度
 * @param r_min 最小转弯半径
 * @return std::tuple<Matrix, double, double, double> 最优终点和控制点参数
 */
std::tuple<Point2D, Point2D, Point2D> findOptimalParameters_Circle(
    const Point2D& p0,
    const Point2D& target_point,
    double radius,
    double theta0,
    double target_length,
    double r_min)
{
    // 算法参数
    double angle_min = PI / 2;  // 圆上搜索角度范围最小值 (弧度)
    double angle_max = PI * 3 / 2;  // 圆上搜索角度范围最大值 (弧度)
    double d0_min = 0;  // 第一控制点距离范围最小值
    double d0_max = target_length;  // 第一控制点距离范围最大值
    double d3_min = 0;  // 最后控制点距离范围最小值
    double d3_max = target_length;  // 最后控制点距离范围最大值
    double dt = 0.01;  // 采样步长

    // 中间参数
    double max_curvature = 0;
    double min_error = std::numeric_limits<double>::max();
    Point2D best_p3 = {0, 0};
    double best_d0 = d0_min;
    double best_d3 = d3_min;
    double best_angle = angle_min;
    
    // 获取目标点坐标
    double target_x = target_point[0];
    double target_y = target_point[1];
    
    // 粗搜索
    double coarse_angle_step = (angle_max - angle_min) / 24;  // 约15度
    double coarse_d0_step = (d0_max - d0_min) / 10;
    double coarse_d3_step = (d3_max - d3_min) / 10;
    
    for (double angle = angle_min; angle <= angle_max; angle += coarse_angle_step) {
        // 计算当前角度对应的终点位置
        Point2D p3 = {
            target_x + radius * std::cos(angle),
            target_y + radius * std::sin(angle)
        };

        // 计算终点的角度（用于控制点方向）
        double theta3 = angle;
        
        // 对每个终点位置，寻找最优控制点参数
        for (double d0 = d0_min; d0 <= d0_max; d0 += coarse_d0_step) {
            for (double d3 = d3_min; d3 <= d3_max; d3 += coarse_d3_step) {
                // 创建控制点
                Point2D p1 = {
                    p0[0] + d0 * std::cos(theta0),
                    p0[1] + d0 * std::sin(theta0)
                };
                
                Point2D p2 = {
                    p3[0] + d3 * std::cos(theta3),
                    p3[1] + d3 * std::sin(theta3)
                };
                
                max_curvature = findMaxCurvature(p0, p1, p2, p3, dt);  // 计算最大曲率
                if (max_curvature > 1 / r_min) {
                    continue;  // 超过最小转弯半径，跳过
                }

                // 计算曲线长度
                double curve_length = calculateBezierLength(p0, p1, p2, p3, 200); // 粗搜索用较少的采样点
                double error = std::fabs(curve_length - target_length);
                
                // 如果找到更好的解，更新参数
                if (error < min_error) {
                    min_error = error;
                    best_p3 = p3;
                    best_d0 = d0;
                    best_d3 = d3;
                    best_angle = angle;
                }
            }
        }
    }

    // 细搜索（省略具体实现，类似原代码）
    double fine_angle_min = std::max(angle_min, best_angle - coarse_angle_step);
    double fine_angle_max = std::min(angle_max, best_angle + coarse_angle_step);
    double fine_d0_min = std::max(d0_min, best_d0 - coarse_d0_step);
    double fine_d0_max = std::min(d0_max, best_d0 + coarse_d0_step);
    double fine_d3_min = std::max(d3_min, best_d3 - coarse_d3_step);
    double fine_d3_max = std::min(d3_max, best_d3 + coarse_d3_step);
    double fine_angle_step = coarse_angle_step / 5;  // 定义细搜索步长 约3度
    double fine_d0_step = coarse_d0_step / 5;
    double fine_d3_step = coarse_d3_step / 5;
    min_error = std::numeric_limits<double>::max(); // 重置最小误差
    
    // 细搜索过程
    for (double angle = fine_angle_min; angle <= fine_angle_max; angle += fine_angle_step) {
        Point2D p3 = {target_x + radius * std::cos(angle), target_y + radius * std::sin(angle)};

        double theta3 = angle;
        
        for (double d0 = fine_d0_min; d0 <= fine_d0_max; d0 += fine_d0_step) {
            for (double d3 = fine_d3_min; d3 <= fine_d3_max; d3 += fine_d3_step) {
                Point2D p1 = {p0[0] + d0 * std::cos(theta0), p0[1] + d0 * std::sin(theta0)};
                Point2D p2 = {p3[0] + d3 * std::cos(theta3), p3[1] + d3 * std::sin(theta3)};
                
                max_curvature = findMaxCurvature(p0, p1, p2, p3, dt);  // 计算最大曲率
                if (max_curvature > 1 / r_min) {
                    // cout << "more than r_min, continue" << endl;
                    continue;  // 超过最小转弯半径，跳过
                }

                double curve_length = calculateBezierLength(p0, p1, p2, p3, 500); // 细搜索用更多采样点
                double error = fabs(curve_length - target_length);
                
                if (error < min_error) {
                    min_error = error;
                    best_p3 = p3;
                    best_d0 = d0;
                    best_d3 = d3;
                    best_angle = angle;
                }
            }
        }
    }

    // 构建最终解
    Point2D p1 = {
        p0[0] + best_d0 * std::cos(theta0),
        p0[1] + best_d0 * std::sin(theta0)
    };
    
    Point2D p2 = {
        best_p3[0] + best_d3 * std::cos(best_angle),
        best_p3[1] + best_d3 * std::sin(best_angle)
    };

    // std::cout << "最终结果: 终点角度 = " << best_angle * 180 / PI << "度, "
    //           << "终点坐标 = (" << best_p3[0] << ", " << best_p3[1] << "), "
    //           << "d0 = " << best_d0 << ", d3 = " << best_d3 
    //           << ", 误差 = " << min_error << std::endl;
    std::cout << "min_error = " << min_error << std::endl;
    
    return std::make_tuple(p1, p2, best_p3);
}

/**
 * @brief 计算贝塞尔曲线采样点并输出到文件
 * 
 * @param p0 起始控制点
 * @param p1 第一控制点
 * @param p2 第二控制点
 * @param p3 终止控制点
 * @param target_point 目标点
 * @param r 半径
 * @param filename 输出文件名
 * @param num_samples 采样点数量
 * @return bool 是否成功输出
 */
BEZIER_API bool outputBezierCurvePoints(
    const Point2D& p0, 
    const Point2D& p1,
    const Point2D& p2, 
    const Point2D& p3,
    const Point2D& target_point, 
    double r,
    const std::string& filename,
    int num_samples)
{
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open output file " << filename << std::endl;
        return false;
    }
    
    // 写入文件头
    outfile << "# Bezier Curve Points\n";
    outfile << "# t\tx\ty\tcurvature\n";
    
    double dt = 1.0 / (num_samples - 1);
    
    // 计算并输出采样点
    for (int i = 0; i < num_samples; i++) {
        double t = i * dt;
        
        // 贝塞尔公式系数
        double B0 = std::pow(1-t, 3);
        double B1 = 3 * t * std::pow(1-t, 2);
        double B2 = 3 * t * t * (1-t);
        double B3 = std::pow(t, 3);
        
        // 计算点坐标
        double x = B0 * p0[0] + B1 * p1[0] + B2 * p2[0] + B3 * p3[0];
        double y = B0 * p0[1] + B1 * p1[1] + B2 * p2[1] + B3 * p3[1];
        
        double curvature = calculateCurvatureAtPoint(t, p0, p1, p2, p3);
        
        // 输出到文件，使用固定精度
        outfile << std::fixed << std::setprecision(3) << t << "\t"
                << std::setprecision(6) << x << "\t" << y << "\t"
                << curvature << std::endl;
    }
    
    outfile.close();
    
    // 单独输出控制点到文件
    std::string control_points_file = filename.substr(0, filename.find_last_of('.')) + "_control_points.txt";
    std::ofstream cp_outfile(control_points_file);
    if (cp_outfile.is_open()) {
        cp_outfile << "# Control Points\n";
        cp_outfile << "# x\ty\n";
        cp_outfile << std::fixed << std::setprecision(6) 
                  << p0[0] << "\t" << p0[1] << std::endl
                  << p1[0] << "\t" << p1[1] << std::endl
                  << p2[0] << "\t" << p2[1] << std::endl
                  << p3[0] << "\t" << p3[1] << std::endl;

        cp_outfile << "# Target Point\n";
        cp_outfile << "# x\ty\n";
        cp_outfile << target_point[0] << "\t" << target_point[1] << std::endl;
        cp_outfile << "# r\n";
        cp_outfile << r << "\t" << 0 << std::endl;
        cp_outfile.close();
    }

    return true;
}

/** 目标函数：最小化曲线长度与目标长度的差的绝对值
 */
double objective_function(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    OptData *opt_data = reinterpret_cast<OptData*>(data);
    
    // x[0]: 圆上的角度
    // x[1]: d0 - 第一控制点距离
    // x[2]: d3 - 最后控制点距离
    
    // 计算终点位置（在圆上）
    double angle = x[0];
    double target_x = opt_data->target_point->at(0);
    double target_y = opt_data->target_point->at(1);
    
    Point2D p3 = {
        target_x + opt_data->radius * std::cos(angle),
        target_y + opt_data->radius * std::sin(angle)
    };
    
    // 计算控制点
    Point2D p1 = {
        opt_data->p0->at(0) + x[1] * std::cos(opt_data->theta0),
        opt_data->p0->at(1) + x[1] * std::sin(opt_data->theta0)
    };
    
    Point2D p2 = {
        p3[0] + x[2] * std::cos(angle),
        p3[1] + x[2] * std::sin(angle)
    };
    
    // 计算曲线长度
    double curve_length = calculateBezierLength(*(opt_data->p0), p1, p2, p3, 500);
    
    return std::fabs(curve_length - opt_data->target_length);
}

/** 曲率约束函数：确保最大曲率不超过限制
 */
double constraint_function(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    OptData *opt_data = reinterpret_cast<OptData*>(data);
    
    // 计算终点和控制点
    double angle = x[0];
    double target_x = opt_data->target_point->at(0);
    double target_y = opt_data->target_point->at(1);
    
    Point2D p3 = {
        target_x + opt_data->radius * std::cos(angle),
        target_y + opt_data->radius * std::sin(angle)
    };
    
    Point2D p1 = {
        opt_data->p0->at(0) + x[1] * std::cos(opt_data->theta0),
        opt_data->p0->at(1) + x[1] * std::sin(opt_data->theta0)
    };
    
    Point2D p2 = {
        p3[0] + x[2] * std::cos(angle),
        p3[1] + x[2] * std::sin(angle)
    };
    
    // 计算最大曲率
    double max_curvature = findMaxCurvature(*(opt_data->p0), p1, p2, p3, 0.01);
    
    return max_curvature - 1.0/opt_data->r_min;
}

/** NLopt结果转换为字符串
 */
const char* nloptResultToString(nlopt::result result) {
    switch(static_cast<int>(result)) {
        case 1: return "SUCCESS";
        case 2: return "STOPVAL_REACHED";
        case 3: return "FTOL_REACHED";
        case 4: return "XTOL_REACHED";
        case 5: return "MAXEVAL_REACHED";
        case -1: return "FAILURE";
        case -2: return "INVALID_ARGS";
        case -3: return "OUT_OF_MEMORY";
        case -4: return "ROUNDOFF_LIMITED";
        case -5: return "FORCED_STOP";
        default: return "UNKNOWN";
    }
}

/**
 * 在目标点固定距离圆上寻找最优终点和控制点参数（使用NLopt）
 * 
 * @param p0 起始点
 * @param target_point 目标点
 * @param radius 目标点所在圆半径
 * @param theta0 起始航向角
 * @param target_length 目标曲线长度
 * @param r_min 最小转弯半径
 * @return std::tuple<Matrix, Matrix, Matrix> 控制点和终点
 */
BEZIER_API std::tuple<Point2D, Point2D, Point2D, double> findNLoptParameters_Circle(
    const Point2D& p0,
    const Point2D& target_point,
    double radius,
    double theta0,
    double target_length,
    double r_min,
    int algorithm)
{
    // 选择算法
    nlopt::algorithm algo;
    switch(algorithm) {
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
    
    nlopt::opt optimizer(algo, 3);
    
    std::vector<double> lower_bounds(3);
    std::vector<double> upper_bounds(3);
    
    lower_bounds[0] = PI / 2;
    upper_bounds[0] = PI * 3 / 2;
    
    // d0 范围 (0 到 target_length)
    lower_bounds[1] = 0.0;
    upper_bounds[1] = target_length;
    
    // d3 范围 (0 到 target_length)
    lower_bounds[2] = 0.0;
    upper_bounds[2] = target_length;
    
    optimizer.set_lower_bounds(lower_bounds);
    optimizer.set_upper_bounds(upper_bounds);
    
    optimizer.set_min_objective(objective_function, &opt_data);
    optimizer.add_inequality_constraint(constraint_function, &opt_data, 1e-8);
    
    optimizer.set_xtol_rel(1e-4);  // 相对误差 所有参数在连续迭代之间的相对变化都小于 0.01% 时
    optimizer.set_maxeval(500);    // 最大评估次数
    
    // 设置初始猜测值（角度为中间值，d0和d3为中等值）
    std::vector<double> x(3);
    x[0] = PI;  // 中间角度
    x[1] = target_length / 4;  // d0初始值
    x[2] = target_length / 4;  // d3初始值
    
    // 存储最优函数值
    double min_error;
    
    // 执行优化
    nlopt::result result = optimizer.optimize(x, min_error);
    
    // 计算最优解的终点和控制点
    double best_angle = x[0];
    double best_d0 = x[1];
    double best_d3 = x[2];
    
    double target_x = target_point[0];
    double target_y = target_point[1];
    
    // 返回结果
    Point2D best_p3 = {
        target_x + radius * std::cos(best_angle),
        target_y + radius * std::sin(best_angle)
    };

    Point2D p1 = {
        p0[0] + best_d0 * std::cos(theta0),
        p0[1] + best_d0 * std::sin(theta0)
    };
    
    Point2D p2 = {
        best_p3[0] + best_d3 * std::cos(best_angle),
        best_p3[1] + best_d3 * std::sin(best_angle)
    };
    
    return std::make_tuple(p1, p2, best_p3, min_error);
}

} // namespace bezier