#include "bezier.hpp"

namespace bezier {

double compute_min_distance(const std::vector<Point2D>& traj1, const std::vector<Point2D>& traj2) {
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& p1 : traj1) {
        for (const auto& p2 : traj2) {
            double dist = std::sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]));
            min_dist = std::min(min_dist, dist);
        }
    }
    return min_dist;
}

std::vector<Point2D> generate_trajectory(Point2D p0, Point2D p1, Point2D p2, Point2D p3) {
    std::vector<Point2D> trajectory;
    int num_samples = 100; // 采样点数量
    for (int i = 0; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        // 三次贝塞尔曲线公式
        double x = (1 - t) * (1 - t) * (1 - t) * p0[0] + 3 * (1 - t) * (1 - t) * t * p1[0] + 3 * (1 - t) * t * t * p2[0] + t * t * t * p3[0];
        double y = (1 - t) * (1 - t) * (1 - t) * p0[1] + 3 * (1 - t) * (1 - t) * t * p1[1] + 3 * (1 - t) * t * t * p2[1] + t * t * t * p3[1];
        trajectory.push_back({ x, y });
    }
    return trajectory;
}

// 整体误差函数，计算所有个体的误差总和
double overall_objective_function(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    OverallOptData* opt_data = static_cast<OverallOptData*>(data);
    double target_length = x[0]; // 单一的 target_length 值
    double total_error = 0.0;
    int n = opt_data->input_XYZ->size();

    std::vector<std::vector<Point2D>> trajectories; // 存储所有轨迹的采样点
    // 对每个个体计算误差并累加
    for (int i = 0; i < n; ++i) {
        auto optimal_parameters = bezier::findNLoptParameters_Circle(
            (*opt_data->input_XYZ)[i],              // 起始点
            *(opt_data->target_point),              // 目标点
            opt_data->target_radius,                // 目标圆半径
            (*opt_data->headings)[i], // 起始航向角
            target_length,                          // 当前的 target_length
            opt_data->r_min,                        // 最小转弯半径
            25                                       // 使用 BOBYQA 算法
        );
        double min_error = std::get<3>(optimal_parameters);
        std::vector<Point2D> trajectory = generate_trajectory((*opt_data->input_XYZ)[i], 
                std::get<0>(optimal_parameters), std::get<1>(optimal_parameters), std::get<2>(optimal_parameters));
        trajectories.push_back(trajectory);
        total_error += min_error;
    }

    // Step 2: 计算相邻轨迹之间的最小距离并添加惩罚项
    double penalty = 0.0;              // 惩罚项初始值为 0
    double min_distance_threshold = 100.0; // 最小距离阈值，可根据实际情况调整
    for (int i = 0; i < n - 1; ++i) {
        // 计算相邻两条轨迹的最小距离
        double min_dist = compute_min_distance(trajectories[i], trajectories[i + 1]);
        if (min_dist < min_distance_threshold) {
            // 当距离小于阈值时，施加惩罚
            penalty += (min_distance_threshold - min_dist) * 111.1; // 惩罚力度可调，自己调整吧，相交的概率挺高的
        }
    }
    // Step 3: 返回总误差（原始误差 + 惩罚项）
    return total_error +penalty;
}

// 优化函数，返回单一的最优 target_length
double optimize_target_length(
    const std::vector<bezier::Point2D>& input_XYZ,
    const bezier::Point2D& target_point,
    double target_radius,
    const std::vector<double>& headings,
    double r_min,
    double min_target_length,
    double max_target_length,
    double initial_target_length
) {
    // 创建单变量优化器
    nlopt::opt optimizer(nlopt::LN_BOBYQA, 1); // 变量数量为 1

    // 设置上下界
    optimizer.set_lower_bounds({ min_target_length });
    optimizer.set_upper_bounds({ max_target_length });

    // 设置优化数据
    OverallOptData opt_data;
    opt_data.input_XYZ = &input_XYZ;
    opt_data.target_point = &target_point;
    opt_data.target_radius = target_radius;
    opt_data.headings = &headings;
    opt_data.r_min = r_min;

    // 设置目标函数
    optimizer.set_min_objective(overall_objective_function, &opt_data);

    // 设置优化参数
    optimizer.set_xtol_rel(1e-4); // 相对误差容忍度
    optimizer.set_maxeval(500);   // 最大评估次数

    // 初始化优化变量
    std::vector<double> x = { initial_target_length };
    double min_error;

    // 执行优化
    nlopt::result result = optimizer.optimize(x, min_error);

    double optimal_target_length = x[0];
    std::cout << "全局最优 target_length = " << optimal_target_length
        << ", 最小整体误差 = " << min_error << std::endl;

    return optimal_target_length;
}

// 示例调用
void run_optimization() {
    // 示例输入数据
    std::vector<bezier::Point2D> input_XYZ = { /* 您的输入点 */ };
    bezier::Point2D target_point = { /* 目标点坐标 */ };
    double target_radius = 10.0;
    std::vector<double> headings = { /* 航向角列表 */ };
    double r_min = 5.0;
    double min_target_length = 10.0;
    double max_target_length = 50.0;
    double initial_target_length = 20.0;

    // 获取全局最优的 target_length
    double optimal_target_length = optimize_target_length(
        input_XYZ, target_point, target_radius, headings, r_min,
        min_target_length, max_target_length, initial_target_length
    );

    // 使用最优值计算所有个体的控制点和终点
    for (int i = 0; i < input_XYZ.size(); ++i) {
        auto optimal_parameters = bezier::findNLoptParameters_Circle(
            input_XYZ[i], target_point, target_radius,
            PI - (headings[i] + 90) * PI / 180.0,
            optimal_target_length, r_min, 1
        );
        bezier::Point2D p1 = std::get<0>(optimal_parameters);
        bezier::Point2D p2 = std::get<1>(optimal_parameters);
        bezier::Point2D p3 = std::get<2>(optimal_parameters);
        double min_error = std::get<3>(optimal_parameters);

        std::cout << "个体 " << i << ": p1 = (" << p1[0] << ", " << p1[1] << "), "
            << "p2 = (" << p2[0] << ", " << p2[1] << "), "
            << "p3 = (" << p3[0] << ", " << p3[1] << "), 误差 = " << min_error << std::endl;
    }
}
}