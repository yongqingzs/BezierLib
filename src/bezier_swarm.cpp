#include "bezier.hpp"

namespace bezier {
    
/**
 * 多弹同时到达优化的目标函数
 */
double multi_missile_objective_function(const std::vector<double> &x, std::vector<double> &grad, void *data) {
    MultiMissileOptData *opt_data = reinterpret_cast<MultiMissileOptData*>(data);
    
    // 计算总体误差
    double total_error = 0.0;
    int num_missiles = opt_data->missiles.size();
    
    for (int i = 0; i < num_missiles; i++) {
        // 每个弹有3个优化变量: 角度、d0、d3
        int base_idx = i * 3;
        double angle = x[base_idx];      // 终点角度
        double d0 = x[base_idx + 1];     // 第一控制点距离
        double d3 = x[base_idx + 2];     // 最后控制点距离
        
        const BezierData& missile = opt_data->missiles[i];
        
        // 计算终点位置
        double target_x = opt_data->target_point->at(0);
        double target_y = opt_data->target_point->at(1);
        
        Point2D p3 = {
            target_x + opt_data->radius * std::cos(angle),
            target_y + opt_data->radius * std::sin(angle)
        };
        
        // 计算控制点
        Point2D p0 = missile.start_point;
        Point2D p1 = {
            p0[0] + d0 * std::cos(missile.theta0),
            p0[1] + d0 * std::sin(missile.theta0)
        };
        
        Point2D p2 = {
            p3[0] + d3 * std::cos(angle),
            p3[1] + d3 * std::sin(angle)
        };
        
        // 计算曲线长度
        double curve_length = calculateBezierLength(p0, p1, p2, p3, 500);
        
        // 计算到达时间
        double arrival_time = curve_length / missile.speed;
        
        // 累加误差
        total_error += std::pow(arrival_time - opt_data->target_arrival_time, 2);
    }
    
    return total_error;
}
    
/**
 * 多弹曲率约束函数
 */
double multi_missile_constraint_function(const std::vector<double> &x, std::vector<double> &grad, void *data) {
    MultiMissileOptData *opt_data = reinterpret_cast<MultiMissileOptData*>(data);
    
    int num_missiles = opt_data->missiles.size();
    std::vector<double> constraints(num_missiles);
    
    for (int i = 0; i < num_missiles; i++) {
        // 每个弹有3个优化变量
        int base_idx = i * 3;
        double angle = x[base_idx];
        double d0 = x[base_idx + 1];
        double d3 = x[base_idx + 2];
        
        const BezierData& missile = opt_data->missiles[i];
        
        // 计算终点位置
        double target_x = opt_data->target_point->at(0);
        double target_y = opt_data->target_point->at(1);
        
        Point2D p3 = {
            target_x + opt_data->radius * std::cos(angle),
            target_y + opt_data->radius * std::sin(angle)
        };
        
        // 计算控制点
        Point2D p0 = missile.start_point;
        Point2D p1 = {
            p0[0] + d0 * std::cos(missile.theta0),
            p0[1] + d0 * std::sin(missile.theta0)
        };
        
        Point2D p2 = {
            p3[0] + d3 * std::cos(angle),
            p3[1] + d3 * std::sin(angle)
        };
        
        // 计算最大曲率
        double max_curvature = findMaxCurvature(p0, p1, p2, p3, 0.01);
        
        // 计算曲率约束违反程度
        constraints[i] = max_curvature - 1.0/missile.r_min;
    }
    
    // 返回最严重的违反约束(最大值)
    return *std::max_element(constraints.begin(), constraints.end());
}
    
/**
 * 多弹同时到达的贝塞尔曲线优化
 * 
 * @param missiles 所有弹的参数
 * @param target_point 目标点
 * @param radius 目标点周围圆的半径
 * @param target_arrival_time 期望到达时间(若为0则自动计算)
 * @return 每个弹的控制点和终点
 */
std::vector<std::tuple<Point2D, Point2D, Point2D, Point2D>> 
optimizeMultiMissilePaths(
    const std::vector<BezierData>& missiles,
    const Point2D& target_point,
    double radius,
    double target_arrival_time)
{
    int num_missiles = missiles.size();
    
    if (target_arrival_time <= 0.0) {
        double total_time = 0.0;
        
        double max_distance = 0;
        for (const auto& missile : missiles) {
            double dx = target_point[0] - missile.start_point[0];
            double dy = target_point[1] - missile.start_point[1];
            double direct_distance = std::sqrt(dx*dx + dy*dy);
            
            if (direct_distance > max_distance) {
                max_distance = direct_distance;
            }
            // total_time += (direct_distance * 1.3) / missile.speed;
            total_time += (max_distance * 1.0 - radius + 100) / missile.speed;
        }
        
        // 取平均值作为目标时间
        target_arrival_time = total_time / num_missiles;
    }
    
    MultiMissileOptData opt_data;
    opt_data.missiles = missiles;
    opt_data.target_point = &target_point;
    opt_data.radius = radius;
    opt_data.target_arrival_time = target_arrival_time;
    
    // 3个优化变量(角度、d0、d3)
    nlopt::opt optimizer(nlopt::LN_COBYLA, num_missiles * 3);
    
    std::vector<double> lower_bounds(num_missiles * 3);
    std::vector<double> upper_bounds(num_missiles * 3);
    
    for (int i = 0; i < num_missiles; i++) {
        int base_idx = i * 3;
        
        lower_bounds[base_idx] = 0;
        upper_bounds[base_idx] = 2 * PI;
        
        double dx = target_point[0] - missiles[i].start_point[0];
        double dy = target_point[1] - missiles[i].start_point[1];
        double max_path_length = 2 * std::sqrt(dx*dx + dy*dy);
        
        // d0和d3范围
        lower_bounds[base_idx + 1] = 0.0;
        upper_bounds[base_idx + 1] = max_path_length;
        lower_bounds[base_idx + 2] = 0.0;
        upper_bounds[base_idx + 2] = max_path_length;
    }
    
    optimizer.set_lower_bounds(lower_bounds);
    optimizer.set_upper_bounds(upper_bounds);
    
    optimizer.set_min_objective(multi_missile_objective_function, &opt_data);
    
    optimizer.add_inequality_constraint(multi_missile_constraint_function, &opt_data, 1e-8);

    // 添加航迹交叉约束
    // optimizer.add_inequality_constraint(multi_missile_intersection_constraint, &opt_data, 1e-8);
    
    optimizer.set_xtol_rel(1e-4);
    optimizer.set_maxeval(1000);
    
    // 设置初始猜测值
    std::vector<double> x(num_missiles * 3);
    
    for (int i = 0; i < num_missiles; i++) {
        int base_idx = i * 3;
        
        // 初始角度均匀分布在圆周上
        // x[base_idx] = 2 * PI * i / num_missiles;
        x[base_idx] = PI;
        
        // 估计合适的控制点距离
        double dx = target_point[0] - missiles[i].start_point[0];
        double dy = target_point[1] - missiles[i].start_point[1];
        double distance = std::sqrt(dx*dx + dy*dy);
        
        x[base_idx + 1] = distance * 0.3; // d0初始值
        x[base_idx + 2] = distance * 0.3; // d3初始值
    }
    
    // 存储最优函数值
    double min_error;
    
    try {
        nlopt::result result = optimizer.optimize(x, min_error);
        std::cout << "优化结果: " << nloptResultToString(result) << ", 误差: " << min_error << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "优化过程中出错: " << e.what() << std::endl;
    }
    
    // 计算最优解对应的控制点和路径
    std::vector<std::tuple<Point2D, Point2D, Point2D, Point2D>> paths;
    
    for (int i = 0; i < num_missiles; i++) {
        int base_idx = i * 3;
        double angle = x[base_idx];
        double d0 = x[base_idx + 1];
        double d3 = x[base_idx + 2];
        
        const BezierData& missile = missiles[i];
        
        Point2D p0 = missile.start_point;
        Point2D p3 = {
            target_point[0] + radius * std::cos(angle),
            target_point[1] + radius * std::sin(angle)
        };
        
        Point2D p1 = {
            p0[0] + d0 * std::cos(missile.theta0),
            p0[1] + d0 * std::sin(missile.theta0)
        };
        
        Point2D p2 = {
            p3[0] + d3 * std::cos(angle),
            p3[1] + d3 * std::sin(angle)
        };
        
        // 计算并输出实际曲线长度和到达时间
        double curve_length = calculateBezierLength(p0, p1, p2, p3, 500);
        double arrival_time = curve_length / missile.speed;
        
        std::cout << "弹道 " << i+1 << " 路径长度: " << curve_length 
                    << ", 到达时间: " << arrival_time
                    << ", 终点角度: " << angle * 180 / PI << "度"
                    << std::endl;
        
        paths.push_back(std::make_tuple(p0, p1, p2, p3));
    }
    
    return paths;
}

/**
 * 从文件解析多弹数据
 * @param filename 数据文件名
 * @param missiles 输出弹的数据
 * @param target_point 输出目标点
 * @param radius 输出目标圆半径
 * @param rad_min 输出最小转弯半径
 * @return 是否成功读取
 */
bool loadMissileDataFromFile(
    const std::string& filename, 
    std::vector<bezier::BezierData>& missiles,
    bezier::Point2D& target_point,
    double& radius,
    double& target_length,
    double& rad_min)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }
    
    std::string line;
    std::string token;
    
    // 清空输入的弹数据向量
    missiles.clear();
    
    // 逐行读取文件
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        iss >> token;
        
        if (token == "Target:") {
            // 读取目标点
            iss >> target_point[0] >> target_point[1];
        }
        else if (token == "RADIUS:") {
            // 读取半径
            iss >> radius;
        }
        else if (token == "LENGTH:") {
            // 读取目标长度
            iss >> target_length;
        }
        else if (token == "RAD_MIN:") {
            // 读取最小转弯半径
            iss >> rad_min;
        }
        else if (token == "MISSILE") {
            // 读取弹数据
            bezier::BezierData missile;
            std::string name;
            iss >> name >> missile.start_point[0] >> missile.start_point[1] 
                >> missile.theta0 >> missile.speed;
            
            // 设置最小转弯半径(使用全局值)
            missile.r_min = rad_min;
            
            // 将弹数据添加到向量中
            missiles.push_back(missile);
        }
        else if (token.length() >= 2 && token[0] == 'c' && std::isdigit(token[1])) {
            // 读取弹数据（使用c1, c2等作为标识符）
            bezier::BezierData missile;
            
            // 直接读取后续的x, y, 航向角, 速度
            iss >> missile.start_point[0] >> missile.start_point[1] 
                >> missile.theta0;
            
            missile.speed = 236;
            
            // 设置最小转弯半径(使用全局值)
            missile.r_min = rad_min;
            
            // 将弹数据添加到向量中
            missiles.push_back(missile);
            
            std::cout << "读取到导弹 " << token << ": 位置=(" 
                      << missile.start_point[0] << ", " << missile.start_point[1] 
                      << "), 航向=" << missile.theta0 
                      << ", 速度=" << missile.speed << std::endl;
        }
    }
    
    file.close();
    
    // 验证是否成功读取数据
    if (missiles.empty()) {
        std::cerr << "警告: 未从文件中读取到弹数据" << std::endl;
        return false;
    }
    
    std::cout << "成功从文件读取 " << missiles.size() << " 个弹数据:" << std::endl;
    std::cout << "目标点: (" << target_point[0] << ", " << target_point[1] << ")" << std::endl;
    std::cout << "目标圆半径: " << radius << std::endl;
    std::cout << "目标曲线长度: " << target_length << std::endl;
    std::cout << "最小转弯半径: " << rad_min << std::endl;
    
    for (size_t i = 0; i < missiles.size(); ++i) {
        std::cout << "弹 " << (i+1) << ": 位置=(" << missiles[i].start_point[0] << ", " 
                 << missiles[i].start_point[1] << "), 航向=" << missiles[i].theta0 
                 << ", 速度=" << missiles[i].speed << std::endl;
    }
    
    return true;
}

}