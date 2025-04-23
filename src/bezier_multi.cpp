#include "bezier.hpp"

namespace bezier {

// 计算三阶贝塞尔曲线控制点
inline std::array<Point2D, 4> calculateCubicBezierControlPoints(
    const Point2D& start_point,      // 起始点
    const Point2D& target_point,     // 目标圆心
    double target_radius,            // 目标圆半径
    double theta0,                   // 起始航向角
    double fixed_angle,              // 终止航向角
    double d0,                       // 第一控制点距离参数
    double d3                        // 最后控制点距离参数
) {
    std::array<Point2D, 4> points;
    
    // P0: 起始点
    points[0] = start_point;
    
    // P1: 从起始点沿起始航向前进
    points[1] = {
        points[0][0] + d0 * std::cos(theta0),
        points[0][1] + d0 * std::sin(theta0)
    };
    
    // P3: 目标圆上的终点
    points[3] = {
        target_point[0] + target_radius * std::cos(fixed_angle),
        target_point[1] + target_radius * std::sin(fixed_angle)
    };
    
    // P2: 从终点沿终止航向反方向前进
    points[2] = {
        points[3][0] + d3 * std::cos(fixed_angle),
        points[3][1] + d3 * std::sin(fixed_angle)
    };
    
    return points;
}

// 计算五阶贝塞尔曲线控制点
inline std::array<Point2D, 6> calculateQuinticBezierControlPoints(
    const Point2D& start_point,      // 起始点
    const Point2D& target_point,     // 目标圆心
    double target_radius,            // 目标圆半径
    double theta0,                   // 起始航向角
    double fixed_angle,              // 终止航向角
    double a,                        // 第一控制点距离参数
    double b,                        // 第五控制点距离参数
    double s,                        // 第二控制点插值参数
    double t,                        // 第三控制点插值参数
    double c,                        // 第二控制点法向偏移
    double d                         // 第三控制点法向偏移
) {
    std::array<Point2D, 6> points;
    
    // P0: 起始点
    points[0] = start_point;
    
    // P5: 目标圆上的终点
    points[5] = {
        target_point[0] + target_radius * std::cos(fixed_angle),
        target_point[1] + target_radius * std::sin(fixed_angle)
    };
    
    // P1: 从起始点沿起始航向前进
    points[1] = {
        points[0][0] + a * std::cos(theta0),
        points[0][1] + a * std::sin(theta0)
    };
    
    // P4: 从终点沿终止航向反方向前进
    points[4] = {
        points[5][0] + b * std::cos(fixed_angle),
        points[5][1] + b * std::sin(fixed_angle)
    };
    
    // P2: 插值+法向偏移
    points[2] = {
        (1-s) * points[0][0] + s * points[5][0] + c * (-std::sin(theta0)),
        (1-s) * points[0][1] + s * points[5][1] + c * (std::cos(theta0))
    };
    
    // P3: 插值+法向偏移
    points[3] = {
        (1-t) * points[0][0] + t * points[5][0] + d * (-std::sin(fixed_angle)),
        (1-t) * points[0][1] + t * points[5][1] + d * (std::cos(fixed_angle))
    };
    
    return points;
}

// 单层优化的目标函数
double unified_objective(unsigned n, const double *x, double *grad, void *data) {
    UnifiedData* user_data = static_cast<UnifiedData*>(data);
    
    // 提取全局参数
    double target_length = x[0];
    double fixed_angle = x[1];
    
    double total_error = 0.0;
    
#if BEZIER_USE_PARALLEL == 1
    #pragma omp parallel for reduction(+:total_error)
#endif
    for (int i = 0; i < user_data->nodes_num; ++i) {
        const NodeData& node = (*(user_data->nodes))[i];
        double theta0 = node.heading;
        double r_min = node.r_min;
        
        // 计算该节点参数的起始索引
        int param_offset = 2 + i * user_data->params_per_node;
        
        // 根据优化类型计算误差
        double current_error = 0.0;
        
        if (user_data->opt_type == 0) {  // 三次贝塞尔曲线
            double d0 = x[param_offset];     // 第一个控制点距离
            double d3 = x[param_offset + 1]; // 最后一个控制点距离
            
            // 计算控制点
            auto points = calculateCubicBezierControlPoints(
                node.start_point,
                *user_data->target_point,
                user_data->target_radius,
                theta0,
                fixed_angle,
                d0,
                d3
            );
            
            Point2D p0 = points[0];
            Point2D p1 = points[1];
            Point2D p2 = points[2];
            Point2D p3 = points[3];
            
            // 计算曲线长度和最大曲率
            double curve_length = calculateBezierLength(p0, p1, p2, p3, 100);
            double max_curvature = findMaxCurvature(p0, p1, p2, p3, 0.01);
            double min_radius = 1.0 / max_curvature;
            
            // 计算误差：长度误差 + 曲率约束违反惩罚
            double length_error = std::fabs(curve_length - target_length);
            double curvature_penalty = max_curvature > 1.0/r_min ? 
                                      1000 * (max_curvature - 1.0/r_min) : 0.0;
            
            current_error = length_error + curvature_penalty;
        } 
        else if (user_data->opt_type == 1) {  // 五次贝塞尔曲线
            // 从参数中提取五次贝塞尔曲线的参数
            double a = x[param_offset];
            double b = x[param_offset + 1];
            double s = x[param_offset + 2];
            double t = x[param_offset + 3];
            double c = x[param_offset + 4];
            double d = x[param_offset + 5];
            
            // 计算控制点
            auto points = calculateQuinticBezierControlPoints(
                node.start_point,
                *user_data->target_point,
                user_data->target_radius,
                theta0,
                fixed_angle,
                a, b, s, t, c, d
            );
            
            Point2D p0 = points[0];
            Point2D p1 = points[1];
            Point2D p2 = points[2];
            Point2D p3 = points[3];
            Point2D p4 = points[4];
            Point2D p5 = points[5];
            
            // 计算曲线长度和最大曲率
            double curve_length = calculateQuinticBezierLength(p0, p1, p2, p3, p4, p5, 100);
            double max_curvature = findMaxQuinticCurvature(p0, p1, p2, p3, p4, p5, 0.01);
            
            // 计算误差
            double length_error = std::fabs(curve_length - target_length);
            double curvature_penalty = max_curvature > 1.0/r_min ? 
                                      1000 * (max_curvature - 1.0/r_min) : 0.0;
            
            current_error = length_error + curvature_penalty;
        }
        
        total_error += current_error;
    }
    
    return total_error;
}

double unified_curvature_constraint(unsigned n, const double *x, double *grad, void *data) {
    UnifiedData* user_data = static_cast<UnifiedData*>(data);
    
    // 提取全局参数
    double fixed_angle = x[1];
    
    // 最大违反值 - 负值表示满足约束，正值表示违反约束
    double max_violation = -std::numeric_limits<double>::max();
    
    for (int i = 0; i < user_data->nodes_num; ++i) {
        const NodeData& node = (*(user_data->nodes))[i];
        double theta0 = node.heading;
        double r_min = node.r_min;
        
        // 计算该节点参数的起始索引
        int param_offset = 2 + i * user_data->params_per_node;
        
        double max_curvature = 0.0;
        
        if (user_data->opt_type == 0) {  // 三次贝塞尔曲线
            double d0 = x[param_offset];     // 第一个控制点距离
            double d3 = x[param_offset + 1]; // 最后一个控制点距离
            
            // 计算控制点
            auto points = calculateCubicBezierControlPoints(
                node.start_point,
                *user_data->target_point,
                user_data->target_radius,
                theta0,
                fixed_angle,
                d0,
                d3
            );
            
            Point2D p0 = points[0];
            Point2D p1 = points[1];
            Point2D p2 = points[2];
            Point2D p3 = points[3];
            
            // 计算最大曲率
            max_curvature = findMaxCurvature(p0, p1, p2, p3, 0.01);
        } 
        else if (user_data->opt_type == 1) {  // 五次贝塞尔曲线
            double a = x[param_offset];
            double b = x[param_offset + 1];
            double s = x[param_offset + 2];
            double t = x[param_offset + 3];
            double c = x[param_offset + 4];
            double d = x[param_offset + 5];
            
            // 计算控制点
            auto points = calculateQuinticBezierControlPoints(
                node.start_point,
                *user_data->target_point,
                user_data->target_radius,
                theta0,
                fixed_angle,
                a, b, s, t, c, d
            );
            
            Point2D p0 = points[0];
            Point2D p1 = points[1];
            Point2D p2 = points[2];
            Point2D p3 = points[3];
            Point2D p4 = points[4];
            Point2D p5 = points[5];
            
            // 计算最大曲率
            max_curvature = findMaxQuinticCurvature(p0, p1, p2, p3, p4, p5, 0.01);
        }
        
        // 计算违反值: (最大曲率 - 允许的最大曲率)
        // 负值表示满足约束，正值表示违反约束
        double violation = max_curvature - 1.0/r_min;
        max_violation = std::max(max_violation, violation);
    }
    
    return max_violation;
}

// 目标函数，计算所有节点的误差之和
double second_layer_objective(unsigned n, const double *x, double *grad, void *data) {
    LayerData* user_data = static_cast<LayerData*>(data);
    
    double total_error = 0.0;
    bool first_cout = false;

    // 使用OpenMP并行计算每个节点的误差
#if BEZIER_USE_PARALLEL == 1
    #pragma omp parallel for reduction(+:total_error)
#endif
    for (int i = 0; i < user_data->nodes_num; ++i) {
        const NodeData& node = (*user_data->nodes)[i];
        double theta0 = node.heading;
        double r_min = node.r_min;
        
        double current_error = 0.0;
        
        if (user_data->opt_type == 0) {  // 固定角度优化
            std::tuple<Point2D, Point2D, Point2D, double> result = 
                findNLoptParameters_FixedAngle(
                    node.start_point, 
                    *user_data->target_point, 
                    user_data->target_radius, 
                    theta0, 
                    x[0],  // target_length 
                    r_min, 
                    x[1],  // fixed_angle
                    *user_data->lower_bounds_first, 
                    *user_data->upper_bounds_first, 
                    *user_data->x_init_first,
                    user_data->algo_first,
                    first_cout
                );
            current_error = std::get<3>(result);
        } 
        else if (user_data->opt_type == 1) {  // 五次贝塞尔曲线优化
            std::tuple<Point2D, Point2D, Point2D, Point2D, Point2D, double> result = 
                findNLoptParameters_QuinticFixed(
                    node.start_point, 
                    *user_data->target_point, 
                    user_data->target_radius, 
                    theta0, 
                    x[0],  // target_length 
                    r_min, 
                    x[1],  // fixed_angle
                    *user_data->lower_bounds_first, 
                    *user_data->upper_bounds_first, 
                    *user_data->x_init_first,
                    user_data->algo_first,
                    first_cout
                );
            current_error = std::get<5>(result);
        }
        
        total_error += current_error;
    }
    
    return total_error;
}

BEZIER_API std::vector<std::array<double, 4>> generateBezierPath(
    const InitData& init,
    const OptParms& opt
) {
    if (init.nodes.empty() || init.node_num <= 0 || init.node_num > init.nodes.size()) {
        return {};
    }

    std::vector<std::array<double, 4>> all_path_points;
    
    // 距离计算
    double min_distance = std::numeric_limits<double>::max();
    double max_distance = 0.0;
    for (int j = 0; j < init.node_num; ++j) {
        const NodeData& node_j = init.nodes[j];
        double dx = node_j.start_point[0] - init.target_point[0];
        double dy = node_j.start_point[1] - init.target_point[1];
        double distance = std::sqrt(dx*dx + dy*dy);
        
        min_distance = std::min(min_distance, distance);
        max_distance = std::max(max_distance, distance);
    }

    OptParms optimized_opt = opt;
    
    // 选择优化方法 - 如果use_unified_opt为true则使用集合优化，否则使用双层优化
    if (opt.use_unified_opt) {
        std::cout << "Running unified optimization...";
        std::cout << nloptAlgorithmToString(opt.algo_first);
        if (opt.opt_type == 0) {
            std::cout << " (Cubic Bezier) ";
        } else if (opt.opt_type == 1) {
            std::cout << " (Quintic Bezier) ";
        }
        std::cout << std::endl;
        
        // 确定每个节点需要的参数数量
        int params_per_node = (opt.opt_type == 0) ? 2 : 6;  // 三次贝塞尔用2个参数，五次用6个参数
        
        // 总参数数量 = 全局参数(2) + 节点数 * 每节点参数数
        int total_params = 2 + init.node_num * params_per_node;
        
        // 创建优化器
        nlopt::opt unified_opt(opt.algo_first, total_params);
        
        // 设置参数边界
        std::vector<double> lower_bounds(total_params);
        std::vector<double> upper_bounds(total_params);
        
        // 设置全局参数边界
        if (opt.lower_bounds_second.size() == 2 && opt.upper_bounds_second.size() == 2) {
            lower_bounds[0] = opt.lower_bounds_second[0];  // target_length下限
            lower_bounds[1] = opt.lower_bounds_second[1];  // fixed_angle下限
            upper_bounds[0] = opt.upper_bounds_second[0];  // target_length上限
            upper_bounds[1] = opt.upper_bounds_second[1];  // fixed_angle上限
        } else {
            lower_bounds[0] = opt.target_length - 2000;  // target_length下限
            lower_bounds[1] = opt.fixed_angle - 0.5;     // fixed_angle下限
            upper_bounds[0] = opt.target_length + 8000;  // target_length上限
            upper_bounds[1] = opt.fixed_angle + 1;       // fixed_angle上限
        }
        
        // 设置每个节点参数的边界
        for (int i = 0; i < init.node_num; ++i) {
            int offset = 2 + i * params_per_node;
            
            if (opt.opt_type == 0) {  // 三次贝塞尔曲线
                // 设置d0和d3的边界
                if (opt.lower_bounds_first.size() >= 2 && opt.upper_bounds_first.size() >= 2) {
                    lower_bounds[offset] = opt.lower_bounds_first[0];    // d0下限
                    lower_bounds[offset+1] = opt.lower_bounds_first[1];  // d3下限
                    upper_bounds[offset] = opt.upper_bounds_first[0];    // d0上限
                    upper_bounds[offset+1] = opt.upper_bounds_first[1];  // d3上限
                } else {
                    lower_bounds[offset] = 10.0;         // d0下限
                    lower_bounds[offset+1] = 10.0;       // d3下限
                    upper_bounds[offset] = opt.target_length;       // d0上限
                    upper_bounds[offset+1] = opt.target_length;     // d3上限
                }
            }
            else if (opt.opt_type == 1) {  // 五次贝塞尔曲线
                // 设置a,b,s,t,c,d的边界
                if (opt.lower_bounds_first.size() >= 6 && opt.upper_bounds_first.size() >= 6) {
                    for (int j = 0; j < 6; ++j) {
                        lower_bounds[offset+j] = opt.lower_bounds_first[j];
                        upper_bounds[offset+j] = opt.upper_bounds_first[j];
                    }
                } else {
                    lower_bounds[offset] = 10.0;         // a下限
                    lower_bounds[offset+1] = 10.0;       // b下限
                    upper_bounds[offset] = opt.target_length;       // a上限
                    upper_bounds[offset+1] = opt.target_length;     // b上限

                    lower_bounds[offset+2] = 0.1;         // s下限
                    lower_bounds[offset+3] = 0.1;         // t下限
                    upper_bounds[offset+2] = 0.9;         // s上限
                    upper_bounds[offset+3] = 0.9;         // t上限

                    lower_bounds[offset+4] = -opt.target_length * 5;  // c下限
                    lower_bounds[offset+5] = -opt.target_length * 5;  // d下限
                    upper_bounds[offset+4] = opt.target_length * 5;   // c上限
                    upper_bounds[offset+5] = opt.target_length * 5;   // d上限
                }
            }
        }
        
        
        unified_opt.set_lower_bounds(lower_bounds);
        unified_opt.set_upper_bounds(upper_bounds);
        
        // 初始化参数
        std::vector<double> x(total_params);
        
        // 设置全局参数初值
        if (opt.x_init_second.size() == 2) {
            x[0] = opt.x_init_second[0];  // 全局target_length初值
            x[1] = opt.x_init_second[1];  // 全局fixed_angle初值
        } else {
            x[0] = opt.target_length;  // 全局target_length初值
            x[1] = opt.fixed_angle;    // 全局fixed_angle初值
        }
        
        // 设置每个节点的初始参数
        for (int i = 0; i < init.node_num; ++i) {
            int offset = 2 + i * params_per_node;
            
            if (opt.opt_type == 0) {  // 三次贝塞尔曲线
                if (opt.x_init_first.size() >= 2) {
                    x[offset] = opt.x_init_first[0];    // d0初值
                    x[offset+1] = opt.x_init_first[1];  // d3初值
                } else {
                    x[offset] = opt.target_length / 3;    // d0初值
                    x[offset+1] = opt.target_length / 3;  // d3初值
                }
            }
            else if (opt.opt_type == 1) {  // 五次贝塞尔曲线
                if (opt.x_init_first.size() >= 6) {
                    for (int j = 0; j < 6; ++j) {
                        x[offset+j] = opt.x_init_first[j];
                    }
                } else {
                    // 默认初值设置
                    x[offset] = opt.target_length / 5;    // a初值
                    x[offset+1] = opt.target_length / 5;  // b初值
                    x[offset+2] = 0.25;                      // s初值
                    x[offset+3] = 0.75;                      // t初值
                    x[offset+4] = 0.0;  // c初值
                    x[offset+5] = 0.0;  // d初值
                }
            }
        }
        
        // 设置用户数据
        UnifiedData user_data;
        user_data.nodes = &init.nodes;
        user_data.nodes_num = init.node_num;
        user_data.target_point = &init.target_point;
        user_data.target_radius = opt.target_radius;
        user_data.params_per_node = params_per_node;
        user_data.opt_type = opt.opt_type;
        
        // 设置目标函数
        unified_opt.set_min_objective(unified_objective, &user_data);

        // 设置曲率约束
        unified_opt.add_inequality_constraint(unified_curvature_constraint, &user_data, 1e-4);
        
        // 设置优化器参数
        unified_opt.set_xtol_rel(1e-4);
        unified_opt.set_maxeval(BEIZER_FIRST_MAXEVAL);
        
        // 执行优化
        double min_error;
        nlopt::result result = unified_opt.optimize(x, min_error);
        std::cout << "Unified optimization success! Result: " << nloptResultToString(result) << " Error: " << min_error << std::endl;
        std::cout << "new target_length: " << x[0] << ", new fixed_angle: " << x[1] << std::endl;

        // 更新优化后的参数
        optimized_opt.target_length = x[0];
        optimized_opt.fixed_angle = x[1];
        
        // 保存最优解供后续使用
        std::vector<std::vector<double>> node_params(init.node_num);
        for (int i = 0; i < init.node_num; ++i) {
            int offset = 2 + i * params_per_node;
            node_params[i].resize(params_per_node);
            for (int j = 0; j < params_per_node; ++j) {
                node_params[i][j] = x[offset + j];
            }
        }
        
        // 分配路径点数组
        all_path_points.resize(init.node_num * optimized_opt.num_samlpes);
        
        // 用于收集节点计算结果的结构
        struct NodeResult {
            int node_idx;
            double error;
            double r_true;
            double r_min;
        };
        std::vector<NodeResult> node_results(init.node_num);
        
        // 根据优化结果生成各节点的路径
#if BEZIER_USE_PARALLEL == 1
        #pragma omp parallel for
#endif
        for (int i = 0; i < init.node_num; ++i) {
            const NodeData& node = init.nodes[i];
            Point2D p0 = node.start_point;
            double theta0 = node.heading;
            double r_min = node.r_min;
            Point2D target_point = init.target_point;
            
            int num_samples = optimized_opt.num_samlpes;
            
            // 当前节点的路径点
            std::vector<std::array<double, 4>> node_path;
            
            // 创建结果对象
            NodeResult result;
            result.node_idx = i;
            result.r_min = r_min;
            
            // 使用优化得到的参数计算路径
            if (opt.opt_type == 0) {  // 三次贝塞尔曲线
                double d0 = node_params[i][0];
                double d3 = node_params[i][1];

                auto points = calculateCubicBezierControlPoints(
                    node.start_point,
                    target_point,
                    optimized_opt.target_radius,
                    theta0,
                    optimized_opt.fixed_angle,
                    d0,
                    d3
                );
                
                Point2D p0 = points[0];
                Point2D p1 = points[1];
                Point2D p2 = points[2];
                Point2D p3 = points[3];
                
                // 计算误差和实际最小转弯半径
                double curve_length = calculateBezierLength(p0, p1, p2, p3, 100);
                result.error = std::fabs(curve_length - optimized_opt.target_length);
                result.r_true = 1 / findMaxCurvature(p0, p1, p2, p3, 0.01);
                
                // 获取贝塞尔曲线上的点
                node_path = getBezierCurvePoints(
                    p0, p1, p2, p3, target_point, optimized_opt.target_radius, num_samples
                );
            } 
            else if (opt.opt_type == 1) {  // 五次贝塞尔曲线
                double a = node_params[i][0];
                double b = node_params[i][1];
                double s = node_params[i][2];
                double t = node_params[i][3];
                double c = node_params[i][4];
                double d = node_params[i][5];
                
                auto points = calculateQuinticBezierControlPoints(
                    node.start_point,
                    target_point,
                    optimized_opt.target_radius,
                    theta0,
                    optimized_opt.fixed_angle,
                    a, b, s, t, c, d
                );
                
                Point2D p0 = points[0];
                Point2D p1 = points[1];
                Point2D p2 = points[2];
                Point2D p3 = points[3];
                Point2D p4 = points[4];
                Point2D p5 = points[5];

                // 计算误差和实际最小转弯半径
                double curve_length = calculateQuinticBezierLength(p0, p1, p2, p3, p4, p5, 100);
                result.error = std::fabs(curve_length - optimized_opt.target_length);
                result.r_true = 1 / findMaxQuinticCurvature(p0, p1, p2, p3, p4, p5, 0.01);
                
                // 获取五次贝塞尔曲线上的点
                node_path = getQuinticBezierPoints(
                    p0, p1, p2, p3, p4, p5, target_point, optimized_opt.target_radius, num_samples
                );
            }
            
            // 将结果存储到节点结果数组中
            node_results[i] = result;
            
            // 保存路径点
            int start_idx = i * num_samples;
            for (int j = 0; j < node_path.size() && j < num_samples; ++j) {
                all_path_points[start_idx + j] = node_path[j];
            }
        }
        
        // 输出结果信息
        std::cout << "\n========== Unified Optimization Results ==========\n";
        for (const auto& result : node_results) {
            std::cout << "Node " << result.node_idx << " error: " << result.error << std::endl;
            std::cout << "Node " << result.node_idx
                    << " r_ratio: " << result.r_true / result.r_min << std::endl;
        }
        std::cout << "===========================================\n";
    }
    else {
        // 原来的双层优化代码 - 保持不变
        if (opt.layer) {
            std::cout << "Layer-2 opt run" << std::endl;
            nlopt::opt second_layer_opt(opt.algo_second, 2); // 优化两个参数
            
            // 设置参数边界
            if (opt.lower_bounds_second.size() == 2 && opt.upper_bounds_second.size() == 2) {
                second_layer_opt.set_lower_bounds(opt.lower_bounds_second);
                second_layer_opt.set_upper_bounds(opt.upper_bounds_second);
            } else {
                // 默认边界设置
                std::vector<double> lower_bounds = {opt.target_length - 2000, opt.fixed_angle - 0.5}; // 最小长度和角度下限
                std::vector<double> upper_bounds = {opt.target_length + 8000, opt.fixed_angle + 1}; // 最大长度和角度上限
                second_layer_opt.set_lower_bounds(lower_bounds);
                second_layer_opt.set_upper_bounds(upper_bounds);
            }
            
            // 初始参数值 目前初值以外部指定
            std::vector<double> params;
            if (opt.x_init_second.size() == 2) {
                params = opt.x_init_second;  // 使用用户提供的初始值
            } else {
                params = {opt.target_length, opt.fixed_angle};  // 默认初始值
            }
            
            // 设置最大评估次数
            second_layer_opt.set_maxeval(BEIZER_SECOND_MAXEVAL);
            
            // 传递所有节点信息
            LayerData user_data;
            user_data.nodes = &init.nodes;
            user_data.nodes_num = init.node_num;
            user_data.target_point = &init.target_point;
            user_data.target_radius = opt.target_radius;
            user_data.algo_first = opt.algo_first;
            user_data.lower_bounds_first = &opt.lower_bounds_first;
            user_data.upper_bounds_first = &opt.upper_bounds_first;
            user_data.x_init_first = &opt.x_init_first;
            user_data.opt_type = opt.opt_type;
            
            // 设置目标函数
            second_layer_opt.set_min_objective(second_layer_objective, &user_data);
            
            // 执行优化
            double min_error = 0.0;
            try {
                nlopt::result result = second_layer_opt.optimize(params, min_error);
                // 更新优化后的参数
                optimized_opt.target_length = params[0];
                optimized_opt.fixed_angle = params[1];
                std::cout << "Layer-2 opt success! New target_length: " << params[0] 
                          << ", fixed_angle: " << params[1] 
                          << ", total error: " << min_error << std::endl;
            } 
            catch (const std::exception& e) {
                // 优化失败时使用原始参数
                std::cout << "Layer-2 opt fail: " << e.what() << ", using original parameters!" << std::endl;
            }
        }

        all_path_points.resize(init.node_num * optimized_opt.num_samlpes);

        // 用于收集节点计算结果的结构
        struct NodeResult {
            int node_idx;
            double error;
            double r_true;
            double r_min;
        };
        std::vector<NodeResult> node_results(init.node_num);

    #if BEZIER_USE_PARALLEL == 1
        #pragma omp parallel for
    #endif
        for (int i = 0; i < init.node_num; ++i) {
            const NodeData& node = init.nodes[i];
            Point2D p0 = node.start_point;
            double theta0 = node.heading;
            double r_min = node.r_min;
            Point2D target_point = init.target_point;

            double target_length = optimized_opt.target_length;
            double radius = optimized_opt.target_radius;
            double fixed_angle = optimized_opt.fixed_angle;
            nlopt::algorithm algo_first = optimized_opt.algo_first;
            int num_samples = optimized_opt.num_samlpes;

            // 当前节点的路径点
            std::vector<std::array<double, 4>> node_path;
            
            // 创建结果对象
            NodeResult result;
            result.node_idx = i;
            result.r_min = r_min;

            // 根据优化类型选择不同的算法
            if (opt.opt_type == 0) {  // 固定角度优化
                std::tuple<Point2D, Point2D, Point2D, double> opt_result = findNLoptParameters_FixedAngle(
                    p0, target_point, radius, theta0, target_length, r_min, fixed_angle, 
                    optimized_opt.lower_bounds_first, 
                    optimized_opt.upper_bounds_first, 
                    optimized_opt.x_init_first,
                    algo_first, 
                    false
                );
                Point2D p1 = std::get<0>(opt_result);
                Point2D p2 = std::get<1>(opt_result);
                Point2D p3 = std::get<2>(opt_result);
                
                // 存储结果而不是立即打印
                result.error = std::get<3>(opt_result);
                result.r_true = 1 / findMaxCurvature(p0, p1, p2, p3, 0.01);
                
                // 获取贝塞尔曲线上的点
                node_path = getBezierCurvePoints(
                    p0, p1, p2, p3, target_point, radius, num_samples
                );
            } 
            else if (opt.opt_type == 1) {  // 五次贝塞尔曲线优化
                std::tuple<Point2D, Point2D, Point2D, Point2D, Point2D, double> opt_result = findNLoptParameters_QuinticFixed(
                    p0, target_point, radius, theta0, target_length, r_min, fixed_angle, 
                    optimized_opt.lower_bounds_first, 
                    optimized_opt.upper_bounds_first, 
                    optimized_opt.x_init_first,
                    algo_first,
                    false
                );
                Point2D p1 = std::get<0>(opt_result);
                Point2D p2 = std::get<1>(opt_result);
                Point2D p3 = std::get<2>(opt_result);
                Point2D p4 = std::get<3>(opt_result);
                Point2D p5 = std::get<4>(opt_result);
                
                // 存储结果而不是立即打印
                result.error = std::get<5>(opt_result);
                result.r_true = 1 / findMaxQuinticCurvature(p0, p1, p2, p3, p4, p5, 0.01);
                
                // 获取五次贝塞尔曲线上的点
                node_path = getQuinticBezierPoints(
                    p0, p1, p2, p3, p4, p5, target_point, radius, num_samples
                );
            }
            
            // 将结果存储到节点结果数组中
            node_results[i] = result;

            int start_idx = i * num_samples;
            for (int j = 0; j < node_path.size() && j < num_samples; ++j) {
                all_path_points[start_idx + j] = node_path[j];
            }
        }

        std::cout << "\n========== Path Generation Results ==========\n";
        for (const auto& result : node_results) {
            std::cout << "Node " << result.node_idx << " error: " << result.error << std::endl;
            
            std::cout << "Node " << result.node_idx
                      << " r_ratio: " << result.r_true / result.r_min << std::endl;
        }
        std::cout << "===========================================\n";
    }

    return all_path_points;  // 返回所有节点的路径点
}

BEZIER_API std::vector<std::array<double, 4>> generateGeoPath(
    const InitData& init_geo,
    const OptParms& opt
) {
    const double a = 6378137.0;  // WGS84椭球体长半轴(m)
    const double f = 1/298.257223563;  // WGS84扁率
    const double e2 = 2*f - f*f;  // 第一偏心率平方
    
    // 使用目标点作为坐标原点(输入经纬度已经是弧度)
    double lat0 = init_geo.target_point[1];  // 纬度(弧度)
    double lon0 = init_geo.target_point[0];  // 经度(弧度)
    
    // 计算该纬度下的子午圈半径
    double N = a / sqrt(1 - e2 * sin(lat0) * sin(lat0));
    double M = a * (1 - e2) / pow(1 - e2 * sin(lat0) * sin(lat0), 1.5);
    
    // 转换为局部坐标系的初始数据
    InitData init_local = init_geo;
    init_local.target_point = {0.0, 0.0};  // 局部坐标系中目标点为原点
    
    // 转换各个节点的坐标
    for (auto& node : init_local.nodes) {
        // 输入经纬度已经是弧度
        double lat = node.start_point[1];  // 纬度(弧度)
        double lon = node.start_point[0];  // 经度(弧度)
        
        // 计算经纬度差值(弧度)
        double dLat = lat - lat0;
        double dLon = lon - lon0;
        
        // 转换为局部坐标系(考虑地球椭球)
        double y = M * dLat;  // 纬度差对应的距离(m)
        double x = N * cos(lat0) * dLon;  // 经度差对应的距离(m)
        
        node.start_point = {x, y};
        
        // 航向角修正(假设输入航向角也是弧度)
        double heading_correction = atan2(sin(dLon) * cos(lat), 
                                       cos(lat0) * sin(lat) - sin(lat0) * cos(lat) * cos(dLon));
        node.heading = node.heading - heading_correction;
    }
    
    // 调用平面坐标系下的路径生成函数
    auto path_local = generateBezierPath(init_local, opt);
    
    // 将局部坐标转换回经纬度
    std::vector<std::array<double, 4>> path_geo;
    for (const auto& point : path_local) {
        double x = point[0];
        double y = point[1];
        
        // 转回经纬度差值(弧度)
        double dLat = y / M;
        double dLon = x / (N * cos(lat0));
        
        // 计算实际经纬度(弧度)
        double lat = lat0 + dLat;
        double lon = lon0 + dLon;
        
        // 航向角也需要进行修正(返回弧度)
        double heading_correction = atan2(sin(dLon) * cos(lat), 
                                       cos(lat0) * sin(lat) - sin(lat0) * cos(lat) * cos(dLon));
        double heading = point[2] + heading_correction;
        
        // 输出格式: 经度(弧度),纬度(弧度),高度(默认0),航向角(弧度)
        path_geo.push_back({lon, lat, 0.0, heading});
    }
    
    return path_geo;
}

BEZIER_API bool outputMultiPathPoints(
    const std::vector<std::array<double, 4>>& all_path_points,
    const Point2D& target_point,
    double target_radius,
    int points_per_node,
    const std::string& output_dir
) {
    if (all_path_points.empty() || points_per_node <= 0 || output_dir.empty()) {
        return false;
    }
    
    // 计算节点数量
    int node_num = int(all_path_points.size() / points_per_node);
    if (node_num * points_per_node != all_path_points.size()) {
        std::cout << "Warning: Path points count (" << all_path_points.size() 
                  << ") is not a multiple of points_per_node (" << points_per_node 
                  << "). Some points may be ignored." << std::endl;
    }
    
    // 创建输出目录（如果不存在）
    #if defined(_WIN32) || defined(_WIN64)
        std::string cmd = "if not exist \"" + output_dir + "\" mkdir \"" + output_dir + "\"";
        system(cmd.c_str());
    #else
        std::string cmd = "mkdir -p \"" + output_dir + "\"";
        if (system(cmd.c_str())) {};
    #endif
    
    // 为每个节点输出路径点
    for (int i = 0; i < node_num; ++i) {
        std::string path_file = output_dir + "/node_" + std::to_string(i+1) + "_path.txt";
        std::ofstream path_out(path_file);
        
        if (path_out.is_open()) {
            // 计算该节点的路径点在all_path_points中的索引范围
            int start_idx = i * points_per_node;
            int end_idx = std::min(start_idx + points_per_node, static_cast<int>(all_path_points.size()));
            
            path_out << "# Path points for Node " << (i+1) << std::endl;
            path_out << "# x y z heading" << std::endl;
            
            // 写入该节点的路径点 - 使用空格分隔而非逗号
            for (int j = start_idx; j < end_idx; ++j) {
                path_out << all_path_points[j][0] << " " << all_path_points[j][1] << " " 
                        << all_path_points[j][2] << " " << all_path_points[j][3] << std::endl;
            }
            path_out.close();
            // std::cout << "Path points for Node " << (i+1) << " saved to: " << path_file << std::endl;
        }
    }
    
    // 输出目标点
    std::string target_file = output_dir + "/target.txt";
    std::ofstream target_out(target_file);
    
    if (target_out.is_open()) {
        target_out << "# Target information" << std::endl;
        target_out << "# x y radius" << std::endl;
        
        // 使用空格分隔而非逗号
        target_out << target_point[0] << " " << target_point[1] << " " << target_radius << std::endl;
        
        target_out.close();
        // std::cout << "Target point and circle saved to: " << target_file << std::endl;
    }
    
    return true;
}

}