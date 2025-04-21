#include "bezier.hpp"

namespace bezier {

// 计算五阶贝塞尔曲线长度
double calculateQuinticBezierLength(
    const Point2D& p0, 
    const Point2D& p1, 
    const Point2D& p2,
    const Point2D& p3,
    const Point2D& p4,
    const Point2D& p5,
    int steps) 
{
    double length = 0.0;
    Point2D prev = p0;
    
    for (int i = 1; i <= steps; i++) {
        double t = static_cast<double>(i) / steps;
        double t1 = 1.0 - t;
        
        // 五阶贝塞尔曲线公式: B(t) = (1-t)⁵P₀ + 5(1-t)⁴tP₁ + 10(1-t)³t²P₂ + 10(1-t)²t³P₃ + 5(1-t)t⁴P₄ + t⁵P₅
        double b0 = t1*t1*t1*t1*t1;
        double b1 = 5*t1*t1*t1*t1*t;
        double b2 = 10*t1*t1*t1*t*t;
        double b3 = 10*t1*t1*t*t*t;
        double b4 = 5*t1*t*t*t*t;
        double b5 = t*t*t*t*t;
        
        Point2D current = {
            b0*p0[0] + b1*p1[0] + b2*p2[0] + b3*p3[0] + b4*p4[0] + b5*p5[0],
            b0*p0[1] + b1*p1[1] + b2*p2[1] + b3*p3[1] + b4*p4[1] + b5*p5[1]
        };
        
        length += std::hypot(current[0] - prev[0], current[1] - prev[1]);
        prev = current;
    }
    
    return length;
}

// 计算五阶贝塞尔曲线在特定参数t处的曲率
double computeQuinticBezierCurvature(
    const Point2D& p0,
    const Point2D& p1,
    const Point2D& p2,
    const Point2D& p3,
    const Point2D& p4,
    const Point2D& p5,
    double t)
{
    // 计算一阶导数
    double t1 = 1.0 - t;
    
    // 一阶导数系数
    double d1_b0 = 5*t1*t1*t1*t1;
    double d1_b1 = 20*t1*t1*t1*t - 5*t1*t1*t1*t1;
    double d1_b2 = 30*t1*t1*t*t - 20*t1*t1*t1*t;
    double d1_b3 = 20*t1*t*t*t - 30*t1*t1*t*t;
    double d1_b4 = 5*t*t*t*t - 20*t1*t*t*t;
    double d1_b5 = -5*t*t*t*t;
    
    Point2D d1 = {
        d1_b0*(p1[0]-p0[0]) + d1_b1*(p2[0]-p1[0]) + d1_b2*(p3[0]-p2[0]) + d1_b3*(p4[0]-p3[0]) + d1_b4*(p5[0]-p4[0]) + d1_b5*(p0[0]-p5[0]),
        d1_b0*(p1[1]-p0[1]) + d1_b1*(p2[1]-p1[1]) + d1_b2*(p3[1]-p2[1]) + d1_b3*(p4[1]-p3[1]) + d1_b4*(p5[1]-p4[1]) + d1_b5*(p0[1]-p5[1])
    };
    
    // 二阶导数系数
    double d2_b0 = 20*t1*t1*t1;
    double d2_b1 = 60*t1*t1*t - 40*t1*t1*t1;
    double d2_b2 = 60*t1*t*t - 120*t1*t1*t;
    double d2_b3 = 20*t*t*t - 120*t1*t*t;
    double d2_b4 = -60*t1*t*t + 20*t*t*t;
    double d2_b5 = -20*t*t*t;
    
    Point2D d2 = {
        d2_b0*(p1[0]-p0[0]) + d2_b1*(p2[0]-p1[0]) + d2_b2*(p3[0]-p2[0]) + d2_b3*(p4[0]-p3[0]) + d2_b4*(p5[0]-p4[0]) + d2_b5*(p0[0]-p5[0]),
        d2_b0*(p1[1]-p0[1]) + d2_b1*(p2[1]-p1[1]) + d2_b2*(p3[1]-p2[1]) + d2_b3*(p4[1]-p3[1]) + d2_b4*(p5[1]-p4[1]) + d2_b5*(p0[1]-p5[1])
    };
    
    double numerator = d1[0] * d2[1] - d1[1] * d2[0];
    double denominator = std::pow(d1[0] * d1[0] + d1[1] * d1[1], 1.5);
    
    if (denominator < 1e-10) {
        return 0.0;  // 避免除以零
    }
    
    return std::fabs(numerator / denominator);
}

// 寻找五阶贝塞尔曲线的最大曲率
double findMaxQuinticCurvature(
    const Point2D& p0,
    const Point2D& p1,
    const Point2D& p2,
    const Point2D& p3,
    const Point2D& p4,
    const Point2D& p5,
    double step)
{
    double max_curvature = 0.0;
    
    for (double t = 0.0; t <= 1.0; t += step) {
        double curvature = computeQuinticBezierCurvature(p0, p1, p2, p3, p4, p5, t);
        max_curvature = std::max(max_curvature, curvature);
    }
    
    return max_curvature;
}

// 目标函数：找到满足长度约束的控制点参数
double objective_function_quintic(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
    OptData* opt_data = reinterpret_cast<OptData*>(data);

    // x[0]: a - 第一控制点距离参数
    // x[1]: b - 第五控制点距离参数
    // x[2]: s - 第二控制点插值参数
    // x[3]: t - 第三控制点插值参数
    // x[4]: c - 第二控制点法向偏移
    // x[5]: d - 第三控制点法向偏移

    double angle = opt_data->fixed_angle; // θ₅
    double theta0 = opt_data->theta0;     // θ₀
    double target_x = opt_data->target_point->at(0);
    double target_y = opt_data->target_point->at(1);

    // 计算终点位置（在固定角度的圆上）
    Point2D p0 = *(opt_data->p0);
    Point2D p5 = {
        target_x + opt_data->radius * std::cos(angle),
        target_y + opt_data->radius * std::sin(angle)
    };
    
    // 使用新的参数化方法计算控制点
    // P₁ = P₀ + a*[cos(θ₀), sin(θ₀)]
    Point2D p1 = {
        p0[0] + x[0] * std::cos(theta0),
        p0[1] + x[0] * std::sin(theta0)
    };
    
    // P₄ = P₅ - b*[cos(θ₅), sin(θ₅)]
    Point2D p4 = {
        p5[0] + x[1] * std::cos(angle),
        p5[1] + x[1] * std::sin(angle)
    };
    
    // P₂ = (1-s)*P₀ + s*P₅ + c*[-sin(θ₀), cos(θ₀)]
    Point2D p2 = {
        (1-x[2])*p0[0] + x[2]*p5[0] + x[4]*(-std::sin(theta0)),
        (1-x[2])*p0[1] + x[2]*p5[1] + x[4]*(std::cos(theta0))
    };
    
    // P₃ = (1-t)*P₀ + t*P₅ + d*[-sin(θ₅), cos(θ₅)]
    Point2D p3 = {
        (1-x[3])*p0[0] + x[3]*p5[0] + x[5]*(-std::sin(angle)),
        (1-x[3])*p0[1] + x[3]*p5[1] + x[5]*(std::cos(angle))
    };

    // 计算曲线长度
    double curve_length = calculateQuinticBezierLength(p0, p1, p2, p3, p4, p5, 500);

    return std::fabs(curve_length - opt_data->target_length);
}

// 修改后的曲率约束函数
double constraint_function_quintic(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
    OptData* opt_data = reinterpret_cast<OptData*>(data);

    // x[0]: a - 第一控制点距离参数
    // x[1]: b - 第五控制点距离参数
    // x[2]: s - 第二控制点插值参数
    // x[3]: t - 第三控制点插值参数
    // x[4]: c - 第二控制点法向偏移
    // x[5]: d - 第三控制点法向偏移

    double angle = opt_data->fixed_angle; // θ₅
    double theta0 = opt_data->theta0;     // θ₀
    double target_x = opt_data->target_point->at(0);
    double target_y = opt_data->target_point->at(1);

    // 计算终点位置（在固定角度的圆上）
    Point2D p0 = *(opt_data->p0);
    Point2D p5 = {
        target_x + opt_data->radius * std::cos(angle),
        target_y + opt_data->radius * std::sin(angle)
    };
    
    // 使用新的参数化方法计算控制点
    // P₁ = P₀ + a*[cos(θ₀), sin(θ₀)]
    Point2D p1 = {
        p0[0] + x[0] * std::cos(theta0),
        p0[1] + x[0] * std::sin(theta0)
    };
    
    // P₄ = P₅ - b*[cos(θ₅), sin(θ₅)]
    Point2D p4 = {
        p5[0] + x[1] * std::cos(angle),
        p5[1] + x[1] * std::sin(angle)
    };
    
    // P₂ = (1-s)*P₀ + s*P₅ + c*[-sin(θ₀), cos(θ₀)]
    Point2D p2 = {
        (1-x[2])*p0[0] + x[2]*p5[0] + x[4]*(-std::sin(theta0)),
        (1-x[2])*p0[1] + x[2]*p5[1] + x[4]*(std::cos(theta0))
    };
    
    // P₃ = (1-t)*P₀ + t*P₅ + d*[-sin(θ₅), cos(θ₅)]
    Point2D p3 = {
        (1-x[3])*p0[0] + x[3]*p5[0] + x[5]*(-std::sin(angle)),
        (1-x[3])*p0[1] + x[3]*p5[1] + x[5]*(std::cos(angle))
    };

    // 计算最大曲率
    double max_curvature = findMaxQuinticCurvature(p0, p1, p2, p3, p4, p5, 0.01);

    return max_curvature - 1.0 / opt_data->r_min;
}

BEZIER_API std::tuple<Point2D, Point2D, Point2D, Point2D, Point2D, double> findNLoptParameters_QuinticFixed(
    const Point2D& p0,
    const Point2D& target_point,
    double radius,
    double theta0,
    double target_length,
    double r_min,
    double fixed_angle,
    const std::vector<double>& lower_bounds,  // 默认为空向量
    const std::vector<double>& upper_bounds,  // 默认为空向量
    const std::vector<double>& init_x,
    nlopt::algorithm algo,
    bool cout_flag)
{
    OptData opt_data;
    opt_data.p0 = &p0;
    opt_data.target_point = &target_point;
    opt_data.radius = radius;
    opt_data.theta0 = theta0;
    opt_data.target_length = target_length;
    opt_data.r_min = r_min;
    opt_data.fixed_angle = fixed_angle;

    nlopt::opt optimizer(algo, 6);  // 现在有6个优化参数：a, b, s, t, c, d

    // 设置参数边界
    std::vector<double> lower_bounds_actual(6);
    std::vector<double> upper_bounds_actual(6);
    if (lower_bounds.size() == 6 && upper_bounds.size() == 6) {
        lower_bounds_actual = lower_bounds;
        upper_bounds_actual = upper_bounds;
    } else {
        // 默认边界设置
        // a, b 参数（控制点距离）
        lower_bounds_actual[0] = 10.0;
        lower_bounds_actual[1] = 10.0;
        upper_bounds_actual[0] = target_length;
        upper_bounds_actual[1] = target_length;
        
        // s, t 参数（插值比例，范围是0到1）
        lower_bounds_actual[2] = 0.1;
        lower_bounds_actual[3] = 0.1;
        upper_bounds_actual[2] = 0.9;
        upper_bounds_actual[3] = 0.9;
        
        // c, d 参数（法向偏移）
        double max_offset = target_length * 5;
        lower_bounds_actual[4] = -max_offset;
        lower_bounds_actual[5] = -max_offset;
        upper_bounds_actual[4] = max_offset;
        upper_bounds_actual[5] = max_offset;
    }

    optimizer.set_lower_bounds(lower_bounds_actual);
    optimizer.set_upper_bounds(upper_bounds_actual);

    optimizer.set_min_objective(objective_function_quintic, &opt_data);
    optimizer.add_inequality_constraint(constraint_function_quintic, &opt_data, 1e-8);

    optimizer.set_xtol_rel(1e-4);  // 相对误差
    optimizer.set_maxeval(BEIZER_FIRST_MAXEVAL);   // 最大评估次数

    // 设置初始猜测值
    std::vector<double> x(6);
    if (init_x.size() == 6)
    {
        x = init_x;  // 使用用户提供的初始值
    }
    else
    {
        x[0] = target_length / 5;  // a 初始值
        x[1] = target_length / 5;  // b 初始值
        x[2] = 0.25;               // s 初始值
        x[3] = 0.75;               // t 初始值
        x[4] = 0.0;                // c 初始值
        x[5] = 0.0;                // d 初始值
    }

    // 存储最优函数值
    double min_error;

    // 执行优化
    nlopt::result result = optimizer.optimize(x, min_error);

    // 获取最优参数
    double best_a = x[0];
    double best_b = x[1];
    double best_s = x[2];
    double best_t = x[3];
    double best_c = x[4];
    double best_d = x[5];

    // 计算终点位置
    double target_x = target_point[0];
    double target_y = target_point[1];

    Point2D best_p5 = {
        target_x + radius * std::cos(fixed_angle),
        target_y + radius * std::sin(fixed_angle)
    };

    // 计算最终控制点
    Point2D p1 = {
        p0[0] + best_a * std::cos(theta0),
        p0[1] + best_a * std::sin(theta0)
    };
    
    Point2D p4 = {
        best_p5[0] + best_b * std::cos(fixed_angle),
        best_p5[1] + best_b * std::sin(fixed_angle)
    };
    
    Point2D p2 = {
        (1-best_s)*p0[0] + best_s*best_p5[0] + best_c*(-std::sin(theta0)),
        (1-best_s)*p0[1] + best_s*best_p5[1] + best_c*(std::cos(theta0))
    };
    
    Point2D p3 = {
        (1-best_t)*p0[0] + best_t*best_p5[0] + best_d*(-std::sin(fixed_angle)),
        (1-best_t)*p0[1] + best_t*best_p5[1] + best_d*(std::cos(fixed_angle))
    };

    if (cout_flag)
    {
        std::cout << "5 ordernlopt result: " << nloptResultToString(result) << std::endl;
        std::cout << "Quintic Bezier min_error = " << min_error << std::endl;
    }

    return std::make_tuple(p1, p2, p3, p4, best_p5, min_error);
}

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
    int num_samples)
{
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open output file " << filename << std::endl;
        return false;
    }
    
    // 写入文件头
    outfile << "# Quintic Bezier Curve Points\n";
    outfile << "# t\tx\ty\tcurvature\n";
    
    double dt = 1.0 / (num_samples - 1);
    
    // 计算并输出采样点
    for (int i = 0; i < num_samples; i++) {
        double t = i * dt;
        double t1 = 1.0 - t;
        
        // 五阶贝塞尔曲线公式系数
        double B0 = t1*t1*t1*t1*t1;
        double B1 = 5*t1*t1*t1*t1*t;
        double B2 = 10*t1*t1*t1*t*t;
        double B3 = 10*t1*t1*t*t*t;
        double B4 = 5*t1*t*t*t*t;
        double B5 = t*t*t*t*t;
        
        // 计算点坐标
        double x = B0*p0[0] + B1*p1[0] + B2*p2[0] + B3*p3[0] + B4*p4[0] + B5*p5[0];
        double y = B0*p0[1] + B1*p1[1] + B2*p2[1] + B3*p3[1] + B4*p4[1] + B5*p5[1];
        
        // 计算该点的曲率
        double curvature = computeQuinticBezierCurvature(p0, p1, p2, p3, p4, p5, t);
        
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
        cp_outfile << "# Quintic Bezier Control Points\n";
        cp_outfile << "# x\ty\n";
        cp_outfile << std::fixed << std::setprecision(6) 
                  << p0[0] << "\t" << p0[1] << std::endl
                  << p1[0] << "\t" << p1[1] << std::endl
                  << p2[0] << "\t" << p2[1] << std::endl
                  << p3[0] << "\t" << p3[1] << std::endl
                  << p4[0] << "\t" << p4[1] << std::endl
                  << p5[0] << "\t" << p5[1] << std::endl;

        cp_outfile << "# Target Point\n";
        cp_outfile << "# x\ty\n";
        cp_outfile << target_point[0] << "\t" << target_point[1] << std::endl;
        cp_outfile << "# r\n";
        cp_outfile << r << "\t" << 0 << std::endl;
        cp_outfile.close();
    }

    return true;
}

BEZIER_API std::vector<std::array<double, 4>> getQuinticBezierPoints(
    const bezier::Point2D& p0,
    const bezier::Point2D& p1,
    const bezier::Point2D& p2,
    const bezier::Point2D& p3,
    const bezier::Point2D& p4,
    const bezier::Point2D& p5,
    const bezier::Point2D& target_point, 
    double r,
    int num_samples)
{
    std::vector<std::array<double, 4>> output_;
    output_.reserve(num_samples);
    double dt = 1.0 / (num_samples - 1);

    // 计算并输出采样点
    for (int i = 0; i < num_samples; i++) {
        double t = i * dt;

        // 五阶贝塞尔公式系数
        double t1 = 1.0 - t;
        double B0 = t1*t1*t1*t1*t1;
        double B1 = 5*t1*t1*t1*t1*t;
        double B2 = 10*t1*t1*t1*t*t;
        double B3 = 10*t1*t1*t*t*t;
        double B4 = 5*t1*t*t*t*t;
        double B5 = t*t*t*t*t;

        // 计算点坐标
        double x = B0*p0[0] + B1*p1[0] + B2*p2[0] + B3*p3[0] + B4*p4[0] + B5*p5[0];
        double y = B0*p0[1] + B1*p1[1] + B2*p2[1] + B3*p3[1] + B4*p4[1] + B5*p5[1];

        // 计算一阶导数（速度向量）
        // 五阶贝塞尔曲线一阶导数系数
        double dB0 = -5*t1*t1*t1*t1;
        double dB1 = 5*t1*t1*t1*t1 - 20*t1*t1*t1*t;
        double dB2 = 20*t1*t1*t1*t - 30*t1*t1*t*t;
        double dB3 = 30*t1*t1*t*t - 20*t1*t*t*t;
        double dB4 = 20*t1*t*t*t - 5*t*t*t*t;
        double dB5 = 5*t*t*t*t;
        
        double x_prime = dB0*p0[0] + dB1*p1[0] + dB2*p2[0] + dB3*p3[0] + dB4*p4[0] + dB5*p5[0];
        double y_prime = dB0*p0[1] + dB1*p1[1] + dB2*p2[1] + dB3*p3[1] + dB4*p4[1] + dB5*p5[1];

        // 计算朝向角度（以北向为0度，顺时针）
        double angle_rad = atan2(x_prime, y_prime);
        if (angle_rad < 0) angle_rad += 2 * PI;
        double heading = angle_rad * (180.0 / PI);

        // 将点坐标、高度(默认为0)和航向角添加到输出向量
        output_.push_back({ x, y, 0, heading });
    }

    return output_;
}

} // namespace bezier