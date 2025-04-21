#include "bezier.hpp"

namespace bezier {

struct LayerData {
    const std::vector<NodeData>* nodes;  // 所有节点
    const Point2D* target_point;
    double target_radius;
    int nodes_num;
    int algorithm;
    const std::vector<double>* lower_bounds;
    const std::vector<double>* upper_bounds;
    const std::vector<double>* x_init;
    int opt_type;
};

// 目标函数，计算所有节点的误差之和
double second_layer_objective(unsigned n, const double *x, double *grad, void *data) {
    LayerData* user_data = static_cast<LayerData*>(data);
    
    double total_error = 0.0;
    bool first_cout = false;

    // 遍历所有节点，累加误差
    for (int i = 0; i < user_data->nodes_num; ++i) {
        const NodeData& node = (*user_data->nodes)[i];
        double theta0 = node.heading;
        double r_min = node.r_min;
        
        // 根据优化类型计算当前节点的误差
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
                    *user_data->lower_bounds, 
                    *user_data->upper_bounds, 
                    *user_data->x_init,
                    user_data->algorithm,
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
                    *user_data->lower_bounds, 
                    *user_data->upper_bounds, 
                    *user_data->x_init,
                    user_data->algorithm,
                    first_cout
                );
            current_error = std::get<5>(result);
        }
        
        // 累加误差
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
    if (opt.layer) {
        std::cout << "Layer-2 opt run" << std::endl;
        nlopt::opt second_layer_opt(nlopt::LN_COBYLA, 2); // 优化两个参数
        
        // 设置参数边界
        std::vector<double> lower_bounds = {min_distance - opt.target_radius, 0}; // 最小长度和角度下限
        std::vector<double> upper_bounds = {2 * max_distance, PI}; // 最大长度和角度上限
        second_layer_opt.set_lower_bounds(lower_bounds);
        second_layer_opt.set_upper_bounds(upper_bounds);
        
        // 初始参数值
        std::vector<double> params = {opt.target_length, opt.fixed_angle};
        
        // 设置最大评估次数
        second_layer_opt.set_maxeval(BEIZER_SECOND_MAXEVAL);
        
        // 传递所有节点信息
        LayerData user_data;
        user_data.nodes = &init.nodes;
        user_data.nodes_num = init.node_num;
        user_data.target_point = &init.target_point;
        user_data.target_radius = opt.target_radius;
        user_data.algorithm = opt.algorithm;
        user_data.lower_bounds = &opt.lower_bounds;
        user_data.upper_bounds = &opt.upper_bounds;
        user_data.x_init = &opt.init_x;
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

    // 为每个节点生成路径
    for (int i = 0; i < init.node_num; ++i) {
        const NodeData& node = init.nodes[i];
        Point2D p0 = node.start_point;
        double theta0 = node.heading;
        double r_min = node.r_min;
        Point2D target_point = init.target_point;

        double target_length = optimized_opt.target_length;
        double radius = optimized_opt.target_radius;
        double fixed_angle = optimized_opt.fixed_angle;
        int algorithm = optimized_opt.algorithm;
        int num_samples = optimized_opt.num_samlpes;

        // 当前节点的路径点
        std::vector<std::array<double, 4>> node_path;

        // 根据优化类型选择不同的算法
        if (opt.opt_type == 0) {  // 固定角度优化
            std::tuple<Point2D, Point2D, Point2D, double> result = findNLoptParameters_FixedAngle(
                p0, target_point, radius, theta0, target_length, r_min, fixed_angle, 
                optimized_opt.lower_bounds, 
                optimized_opt.upper_bounds, 
                optimized_opt.init_x,
                algorithm, 
                false
            );
            Point2D p1 = std::get<0>(result);
            Point2D p2 = std::get<1>(result);
            Point2D p3 = std::get<2>(result);
            double min_error = std::get<3>(result);

            std::cout << "Node " << i << " error: " << min_error << std::endl;

            // 获取贝塞尔曲线上的点
            node_path = getBezierCurvePoints(
                p0, p1, p2, p3, target_point, radius, num_samples
            );
        } 
        else if (opt.opt_type == 1) {  // 五次贝塞尔曲线优化
            std::tuple<Point2D, Point2D, Point2D, Point2D, Point2D, double> result = findNLoptParameters_QuinticFixed(
                p0, target_point, radius, theta0, target_length, r_min, fixed_angle, 
                optimized_opt.lower_bounds, 
                optimized_opt.upper_bounds, 
                optimized_opt.init_x,
                algorithm, 
                false
            );
            Point2D p1 = std::get<0>(result);
            Point2D p2 = std::get<1>(result);
            Point2D p3 = std::get<2>(result);
            Point2D p4 = std::get<3>(result);
            Point2D p5 = std::get<4>(result);
            double min_error = std::get<5>(result);

            std::cout << "Node " << i << " error: " << min_error << std::endl;

            // 获取五次贝塞尔曲线上的点
            node_path = getQuinticBezierPoints(
                p0, p1, p2, p3, p4, p5, target_point, radius, num_samples
            );
        }

        // 将当前节点的路径添加到总路径中
        for (auto& point : node_path) {
            all_path_points.push_back(point);
        }
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

}