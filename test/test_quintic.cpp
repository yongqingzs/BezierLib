#include <locale>
#include "bezier.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

using namespace bezier;

int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    // 创建输出目录
    std::string output_dir = "../output";
    
    // 设置起点、方向和目标点
    Point2D start_point = {-18408.05, -6982.94};       // 起点坐标
    double start_angle = 0.643083;                // 起点方向（弧度）
    Point2D target_point = {0, 0};    // 目标点坐标
    
    // 设置参数
    double radius = 10000;              // 终点圆半径
    double target_length = 10000;      // 期望的曲线长度
    double min_radius = 3500;          // 最小转弯半径（曲率约束）
    double end_angle = PI + PI / 8;       // 终点方向（弧度）
    
    std::cout << "=== 五阶贝塞尔曲线优化示例 ===" << std::endl;
    std::cout << "起点: (" << start_point[0] << ", " << start_point[1] << "), 角度: " 
              << start_angle * 180 / PI << "°" << std::endl;
    std::cout << "目标点: (" << target_point[0] << ", " << target_point[1] 
              << "), 终点角度: " << end_angle * 180 / PI << "°" << std::endl;
    std::cout << "目标长度: " << target_length << ", 最小转弯半径: " << min_radius << std::endl;
    
    std::cout << "\n开始优化..." << std::endl;
    
    // 调用优化函数
    auto result = findNLoptParameters_QuinticFixed(
        start_point, target_point, radius, start_angle, 
        target_length, min_radius, end_angle);
    
    // 提取结果
    Point2D p1 = std::get<0>(result);
    Point2D p2 = std::get<1>(result);
    Point2D p3 = std::get<2>(result);
    Point2D p4 = std::get<3>(result);
    Point2D p5 = std::get<4>(result);
    double error = std::get<5>(result);
    
    // 输出结果
    std::cout << "\n优化结果:" << std::endl;
    std::cout << "误差: " << error << std::endl;
    
    // 计算并输出其他指标
    // double curve_length = calculateQuinticBezierLength(
    //     start_point, p1, p2, p3, p4, p5, 500);
    // double max_curv = findMaxQuinticCurvature(
    //     start_point, p1, p2, p3, p4, p5, 0.01);
        
    // std::cout << "曲线长度: " << curve_length 
    //           << " (目标: " << target_length << ")" << std::endl;
    // std::cout << "最大曲率: " << max_curv 
    //           << " (最小转弯半径: " << (max_curv > 0 ? 1.0/max_curv : 0) << ")" << std::endl;
    
    // 输出控制点
    std::cout << "\n控制点坐标:" << std::endl;
    std::cout << "P0: (" << start_point[0] << ", " << start_point[1] << ")" << std::endl;
    std::cout << "P1: (" << p1[0] << ", " << p1[1] << ")" << std::endl;
    std::cout << "P2: (" << p2[0] << ", " << p2[1] << ")" << std::endl;
    std::cout << "P3: (" << p3[0] << ", " << p3[1] << ")" << std::endl;
    std::cout << "P4: (" << p4[0] << ", " << p4[1] << ")" << std::endl;
    std::cout << "P5: (" << p5[0] << ", " << p5[1] << ")" << std::endl;
    
    // 将曲线点输出到文件
    // std::string filename = output_dir + "/quintic_bezier_curve.txt";
    std::string filename = "../../output/quintic_curve.txt";
    bool success = outputQuinticBezierPoints(
        start_point, p1, p2, p3, p4, p5, target_point, radius, 
        filename, 200);
    
    if (success) {
        std::cout << "\n曲线数据已写入: " << filename << std::endl;
    } else {
        std::cerr << "\n无法写入曲线数据到文件" << std::endl;
    }
    
    return 0;
}