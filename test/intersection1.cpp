/**
 * 贝塞尔曲线相交检测演示程序
 * 文件名: bezier_intersection_demo.cpp
 */
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <random>
#include <tuple>
#include "bezier.hpp"

using namespace bezier;

// 生成随机贝塞尔曲线
std::tuple<Point2D, Point2D, Point2D, Point2D> generateRandomBezierCurve(
    std::mt19937& rng, double min_x, double max_x, double min_y, double max_y)
{
    std::uniform_real_distribution<double> dist_x(min_x, max_x);
    std::uniform_real_distribution<double> dist_y(min_y, max_y);

    Point2D p0 = {dist_x(rng), dist_y(rng)};
    Point2D p1 = {dist_x(rng), dist_y(rng)};
    Point2D p2 = {dist_x(rng), dist_y(rng)};
    Point2D p3 = {dist_x(rng), dist_y(rng)};

    return {p0, p1, p2, p3};
}

bool outputCurvePair(
    const Point2D& p0_1, const Point2D& p1_1, const Point2D& p2_1, const Point2D& p3_1,
    const Point2D& p0_2, const Point2D& p1_2, const Point2D& p2_2, const Point2D& p3_2,
    const std::string& filename,
    double safety_distance,
    int num_samples = 100)
{    
    // 3. 检测相交情况
    double intersection_value = checkBezierIntersectionBySampling(
        p0_1, p1_1, p2_1, p3_1,
        p0_2, p1_2, p2_2, p3_2,
        safety_distance, num_samples
    );
    bool is_intersecting = (intersection_value > 0);
    
    // 4. 生成SVG可视化
    std::string svg_file = filename + ".svg";
    std::ofstream svg(svg_file);
    if (!svg.is_open()) {
        std::cerr << "Error: Could not create SVG file " << svg_file << std::endl;
        return false;
    }
    
    // 计算画布范围
    double min_x = std::min({p0_1[0], p1_1[0], p2_1[0], p3_1[0], p0_2[0], p1_2[0], p2_2[0], p3_2[0]});
    double min_y = std::min({p0_1[1], p1_1[1], p2_1[1], p3_1[1], p0_2[1], p1_2[1], p2_2[1], p3_2[1]});
    double max_x = std::max({p0_1[0], p1_1[0], p2_1[0], p3_1[0], p0_2[0], p1_2[0], p2_2[0], p3_2[0]});
    double max_y = std::max({p0_1[1], p1_1[1], p2_1[1], p3_1[1], p0_2[1], p1_2[1], p2_2[1], p3_2[1]});
    
    // 添加边距
    double padding = 50.0;
    min_x -= padding;
    min_y -= padding;
    max_x += padding;
    max_y += padding;
    
    // SVG头部
    svg << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n";
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"800\" height=\"600\" ";
    svg << "viewBox=\"" << min_x << " " << min_y << " " 
        << (max_x - min_x) << " " << (max_y - min_y) << "\">\n";
    
    // 添加网格
    svg << "  <!-- 网格线 -->\n";
    svg << "  <g stroke=\"#dddddd\" stroke-width=\"0.5\">\n";
    for (double x = std::floor(min_x/50)*50; x <= max_x; x += 50) {
        svg << "    <line x1=\"" << x << "\" y1=\"" << min_y << "\" x2=\"" << x << "\" y2=\"" << max_y << "\" />\n";
        svg << "    <text x=\"" << x << "\" y=\"" << (min_y + 15) << "\" font-size=\"10\" fill=\"#999\">" 
             << x << "</text>\n";
    }
    for (double y = std::floor(min_y/50)*50; y <= max_y; y += 50) {
        svg << "    <line x1=\"" << min_x << "\" y1=\"" << y << "\" x2=\"" << max_x << "\" y2=\"" << y << "\" />\n";
        svg << "    <text x=\"" << (min_x + 5) << "\" y=\"" << y << "\" font-size=\"10\" fill=\"#999\">" 
             << y << "</text>\n";
    }
    svg << "  </g>\n\n";
    
    // 绘制控制多边形（虚线）
    svg << "  <!-- 控制多边形 -->\n";
    svg << "  <g stroke-dasharray=\"5,5\" stroke-width=\"1\">\n";
    svg << "    <polyline points=\""
        << p0_1[0] << "," << p0_1[1] << " "
        << p1_1[0] << "," << p1_1[1] << " "
        << p2_1[0] << "," << p2_1[1] << " "
        << p3_1[0] << "," << p3_1[1] 
        << "\" stroke=\"#9999ff\" fill=\"none\" />\n";
    
    svg << "    <polyline points=\""
        << p0_2[0] << "," << p0_2[1] << " "
        << p1_2[0] << "," << p1_2[1] << " "
        << p2_2[0] << "," << p2_2[1] << " "
        << p3_2[0] << "," << p3_2[1] 
        << "\" stroke=\"#99ff99\" fill=\"none\" />\n";
    svg << "  </g>\n\n";
    
    // 生成曲线点集
    std::vector<Point2D> curve1_points;
    std::vector<Point2D> curve2_points;
    double dt = 1.0 / (num_samples - 1);
    
    for (int i = 0; i < num_samples; i++) {
        double t = i * dt;
        double B0 = std::pow(1-t, 3);
        double B1 = 3 * t * std::pow(1-t, 2);
        double B2 = 3 * t * t * (1-t);
        double B3 = std::pow(t, 3);
        
        // 第一条曲线
        double x1 = B0 * p0_1[0] + B1 * p1_1[0] + B2 * p2_1[0] + B3 * p3_1[0];
        double y1 = B0 * p0_1[1] + B1 * p1_1[1] + B2 * p2_1[1] + B3 * p3_1[1];
        curve1_points.push_back({x1, y1});
        
        // 第二条曲线
        double x2 = B0 * p0_2[0] + B1 * p1_2[0] + B2 * p2_2[0] + B3 * p3_2[0];
        double y2 = B0 * p0_2[1] + B1 * p1_2[1] + B2 * p2_2[1] + B3 * p3_2[1];
        curve2_points.push_back({x2, y2});
    }
    
    // 绘制实际曲线
    svg << "  <!-- 实际贝塞尔曲线 -->\n";
    
    // 绘制第一条曲线
    svg << "  <path d=\"M ";
    for (size_t i = 0; i < curve1_points.size(); i++) {
        svg << curve1_points[i][0] << "," << curve1_points[i][1];
        if (i < curve1_points.size() - 1) svg << " L ";
    }
    svg << "\" stroke=\"#0000cc\" stroke-width=\"2\" fill=\"none\" />\n";
    
    // 绘制第二条曲线
    svg << "  <path d=\"M ";
    for (size_t i = 0; i < curve2_points.size(); i++) {
        svg << curve2_points[i][0] << "," << curve2_points[i][1];
        if (i < curve2_points.size() - 1) svg << " L ";
    }
    svg << "\" stroke=\"#009900\" stroke-width=\"2\" fill=\"none\" />\n\n";
    
    // 查找相交点或最近点
    double min_distance = std::numeric_limits<double>::max();
    Point2D closest_point1, closest_point2;
    
    for (size_t i = 0; i < curve1_points.size(); i++) {
        for (size_t j = 0; j < curve2_points.size(); j++) {
            double dx = curve1_points[i][0] - curve2_points[j][0];
            double dy = curve1_points[i][1] - curve2_points[j][1];
            double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist < min_distance) {
                min_distance = dist;
                closest_point1 = curve1_points[i];
                closest_point2 = curve2_points[j];
            }
        }
    }
    
    // 标记控制点
    svg << "  <!-- 控制点标记 -->\n";
    svg << "  <g>\n";
    svg << "    <circle cx=\"" << p0_1[0] << "\" cy=\"" << p0_1[1] << "\" r=\"4\" fill=\"#0000cc\" />\n";
    svg << "    <circle cx=\"" << p1_1[0] << "\" cy=\"" << p1_1[1] << "\" r=\"3\" fill=\"#0000cc\" fill-opacity=\"0.6\" />\n";
    svg << "    <circle cx=\"" << p2_1[0] << "\" cy=\"" << p2_1[1] << "\" r=\"3\" fill=\"#0000cc\" fill-opacity=\"0.6\" />\n";
    svg << "    <circle cx=\"" << p3_1[0] << "\" cy=\"" << p3_1[1] << "\" r=\"4\" fill=\"#0000cc\" />\n";
    
    svg << "    <circle cx=\"" << p0_2[0] << "\" cy=\"" << p0_2[1] << "\" r=\"4\" fill=\"#009900\" />\n";
    svg << "    <circle cx=\"" << p1_2[0] << "\" cy=\"" << p1_2[1] << "\" r=\"3\" fill=\"#009900\" fill-opacity=\"0.6\" />\n";
    svg << "    <circle cx=\"" << p2_2[0] << "\" cy=\"" << p2_2[1] << "\" r=\"3\" fill=\"#009900\" fill-opacity=\"0.6\" />\n";
    svg << "    <circle cx=\"" << p3_2[0] << "\" cy=\"" << p3_2[1] << "\" r=\"4\" fill=\"#009900\" />\n";
    svg << "  </g>\n\n";
    
    // 标记最近点
    svg << "  <!-- 最近点标记 -->\n";
    svg << "  <g>\n";
    svg << "    <circle cx=\"" << closest_point1[0] << "\" cy=\"" << closest_point1[1] 
         << "\" r=\"5\" fill=\"" << (is_intersecting ? "#ff0000" : "#0000cc") << "\" stroke=\"#ffffff\" stroke-width=\"1\" />\n";
    svg << "    <circle cx=\"" << closest_point2[0] << "\" cy=\"" << closest_point2[1] 
         << "\" r=\"5\" fill=\"" << (is_intersecting ? "#ff0000" : "#009900") << "\" stroke=\"#ffffff\" stroke-width=\"1\" />\n";
    
    // 连接最近点
    svg << "    <line x1=\"" << closest_point1[0] << "\" y1=\"" << closest_point1[1]
        << "\" x2=\"" << closest_point2[0] << "\" y2=\"" << closest_point2[1]
        << "\" stroke=\"" << (is_intersecting ? "#ff0000" : "#999999") 
        << "\" stroke-width=\"1\" stroke-dasharray=\"3,3\" />\n";
    svg << "  </g>\n\n";
    
    // 添加信息文本
    svg << "  <!-- 信息标签 -->\n";
    svg << "  <g font-family=\"Arial\" font-size=\"12\">\n";
    svg << "    <text x=\"" << min_x + 10 << "\" y=\"" << min_y + 20 << "\" font-weight=\"bold\">"
        << "贝塞尔曲线对相交检测</text>\n";
    
    svg << "    <text x=\"" << min_x + 10 << "\" y=\"" << min_y + 40 << "\">"
        << "状态: " << (is_intersecting ? "相交" : "不相交") << "</text>\n";
    
    // svg << "    <text x=\"" << min_x + 10 << "\" y=\"" << min_y + 60 << "\">"
    //     << "最小距离: " << std::fixed << std::setprecision(2) << min_distance << "</text>\n";
    
    svg << "    <text x=\"" << min_x + 10 << "\" y=\"" << min_y + 80 << "\">"
        << "安全距离: " << safety_distance << "</text>\n";
    
    svg << "    <text x=\"" << min_x + 10 << "\" y=\"" << min_y + 100 << "\">"
        << "相交检测值: " << std::fixed << std::setprecision(4) << intersection_value << "</text>\n";
    
    // 添加图例
    svg << "    <g transform=\"translate(" << min_x + 10 << "," << min_y + 130 << ")\">\n";
    svg << "      <line x1=\"0\" y1=\"0\" x2=\"20\" y2=\"0\" stroke=\"#0000cc\" stroke-width=\"2\" />\n";
    svg << "      <text x=\"25\" y=\"5\">曲线1</text>\n";
    svg << "    </g>\n";
    
    svg << "    <g transform=\"translate(" << min_x + 10 << "," << min_y + 150 << ")\">\n";
    svg << "      <line x1=\"0\" y1=\"0\" x2=\"20\" y2=\"0\" stroke=\"#009900\" stroke-width=\"2\" />\n";
    svg << "      <text x=\"25\" y=\"5\">曲线2</text>\n";
    svg << "    </g>\n";
    
    svg << "    <g transform=\"translate(" << min_x + 10 << "," << min_y + 170 << ")\">\n";
    svg << "      <line x1=\"0\" y1=\"0\" x2=\"20\" y2=\"0\" stroke=\"#9999ff\" stroke-width=\"1\" stroke-dasharray=\"5,5\" />\n";
    svg << "      <text x=\"25\" y=\"5\">控制多边形1</text>\n";
    svg << "    </g>\n";
    
    svg << "    <g transform=\"translate(" << min_x + 10 << "," << min_y + 190 << ")\">\n";
    svg << "      <line x1=\"0\" y1=\"0\" x2=\"20\" y2=\"0\" stroke=\"#99ff99\" stroke-width=\"1\" stroke-dasharray=\"5,5\" />\n";
    svg << "      <text x=\"25\" y=\"5\">控制多边形2</text>\n";
    svg << "    </g>\n";
    svg << "  </g>\n";
    
    // 结束SVG
    svg << "</svg>\n";
    svg.close();
    
    std::cout << "SVG可视化已保存至：" << svg_file << std::endl;
    // std::cout << "两曲线间最小距离: " << min_distance << std::endl;
    std::cout << "相交情况: " << (is_intersecting ? "相交" : "不相交") 
              << " (值: " << intersection_value << ")" << std::endl;

    // 5. 输出相交检测细节到文本文件
    // std::string details_file = filename + "_details.txt";
    // std::ofstream details(details_file);
    // if (details.is_open()) {
    //     details << "# 贝塞尔曲线相交检测结果\n\n";
        
    //     details << "## 控制点\n";
    //     details << "曲线1: (" << p0_1[0] << "," << p0_1[1] << "), ("
    //             << p1_1[0] << "," << p1_1[1] << "), ("
    //             << p2_1[0] << "," << p2_1[1] << "), ("
    //             << p3_1[0] << "," << p3_1[1] << ")\n";
                
    //     details << "曲线2: (" << p0_2[0] << "," << p0_2[1] << "), ("
    //             << p1_2[0] << "," << p1_2[1] << "), ("
    //             << p2_2[0] << "," << p2_2[1] << "), ("
    //             << p3_2[0] << "," << p3_2[1] << ")\n\n";
                
    //     details << "## 检测结果\n";
    //     details << "最小距离: " << min_distance << "\n";
    //     details << "安全距离: " << safety_distance << "\n";
    //     details << "检测值: " << intersection_value << "\n";
    //     details << "结论: " << (is_intersecting ? "相交" : "不相交") << "\n\n";
        
    //     details << "## 最近点\n";
    //     details << "曲线1上最近点: (" << closest_point1[0] << "," << closest_point1[1] << ")\n";
    //     details << "曲线2上最近点: (" << closest_point2[0] << "," << closest_point2[1] << ")\n";
        
    //     details.close();
    //     std::cout << "详细检测结果已保存至：" << details_file << std::endl;
    // }
    
    return true;
}

int demo_0()
{
    // 设置随机数生成器
    std::random_device rd;
    std::mt19937 rng(rd());

    // 测试参数
    const double safety_distance = 20.0;
    const int num_curves = 10; // 生成10对曲线进行测试
    const double canvas_size = 500.0;

    // 生成曲线进行演示
    std::vector<std::tuple<Point2D, Point2D, Point2D, Point2D>> curves1;
    std::vector<std::tuple<Point2D, Point2D, Point2D, Point2D>> curves2;

    // 1. 创建一些肯定会相交的曲线对
    curves1.push_back({{100, 100}, {200, 300}, {300, 100}, {400, 300}}); // 波浪形
    curves2.push_back({{100, 300}, {200, 100}, {300, 300}, {400, 100}}); // 交叉波浪形

    curves1.push_back({{100, 150}, {150, 150}, {200, 150}, {250, 150}}); // 水平线
    curves2.push_back({{175, 100}, {175, 150}, {175, 200}, {175, 250}}); // 垂直线

    // 2. 创建一些接近但不相交的曲线对
    curves1.push_back({{100, 400}, {150, 300}, {250, 300}, {300, 400}});
    curves2.push_back({{100, 380}, {150, 280}, {250, 280}, {300, 380}}); // 平行，距离20

    // 3. 创建一些肯定不会相交的曲线对
    curves1.push_back({{50, 450}, {100, 450}, {150, 450}, {200, 450}});
    curves2.push_back({{50, 500}, {100, 500}, {150, 500}, {200, 500}});

    // 4. 添加一些随机生成的曲线对
    for (int i = 0; i < num_curves; i++) {
        curves1.push_back(generateRandomBezierCurve(rng, 0.0, canvas_size, 0.0, canvas_size));
        curves2.push_back(generateRandomBezierCurve(rng, 0.0, canvas_size, 0.0, canvas_size));
    }

    // 使用不同的采样点数进行测试
    // std::vector<int> sample_counts = {5, 10, 20, 50, 100};
    std::vector<int> sample_counts = {100};
    
    auto testIntersectionBatch = [](const std::vector<std::tuple<Point2D, Point2D, Point2D, Point2D>>& curves1,
        const std::vector<std::tuple<Point2D, Point2D, Point2D, Point2D>>& curves2,
        double safety_distance, int num_samples, 
        std::vector<double>& results) {
        results.resize(std::min(curves1.size(), curves2.size()));

        auto start = std::chrono::high_resolution_clock::now();

        for (size_t i = 0; i < results.size(); i++) {
            Point2D p0_1 = std::get<0>(curves1[i]);
            Point2D p1_1 = std::get<1>(curves1[i]);
            Point2D p2_1 = std::get<2>(curves1[i]);
            Point2D p3_1 = std::get<3>(curves1[i]);
            Point2D p0_2 = std::get<0>(curves2[i]);
            Point2D p1_2 = std::get<1>(curves2[i]);
            Point2D p2_2 = std::get<2>(curves2[i]);
            Point2D p3_2 = std::get<3>(curves2[i]);

            results[i] = checkBezierIntersectionBySampling(
                p0_1, p1_1, p2_1, p3_1,
                p0_2, p1_2, p2_2, p3_2,
                safety_distance, num_samples
            );
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;

        std::cout << "检测 " << results.size() << " 对曲线 (采样点数: " << num_samples 
                << "): " << elapsed.count() << " ms" << std::endl;
        std::cout << "每对曲线平均用时: " << elapsed.count() / results.size() << " ms" << std::endl;
        return 0;
    };

    for (int samples : sample_counts) {
        std::vector<double> results;
        testIntersectionBatch(curves1, curves2, safety_distance, samples, results);
        
        // 将结果转换为是否相交的布尔值
        std::vector<bool> intersections;
        for (double result : results) {
            intersections.push_back(result > 0);
        }
        
        // 输出相交检测结果
        std::cout << "采样点数: " << samples << std::endl;
        for (size_t i = 0; i < results.size(); i++) {
            std::cout << "曲线对 " << i+1 << ": " 
                        << (results[i] > 0 ? "相交" : "不相交") 
                        << " (值: " << results[i] << ")" << std::endl;
        }
        std::cout << std::endl;
    }
    
    // 生成SVG可视化
    for (size_t i = 0; i < curves1.size(); i++) {
        std::string filename = "./output/curve_pair_" + std::to_string(i);
        const auto& curve1 = curves1[i];
        const auto& curve2 = curves2[i];
        
        outputCurvePair(
            std::get<0>(curve1), std::get<1>(curve1), std::get<2>(curve1), std::get<3>(curve1),
            std::get<0>(curve2), std::get<1>(curve2), std::get<2>(curve2), std::get<3>(curve2),
            filename, safety_distance
        );
    }
    return 0;    
}

int demo_1()
{
    struct TestCase {
        Point2D p1, p2;  // 第一条线段
        Point2D q1, q2;  // 第二条线段
        bool expected;   // 期望结果
        std::string description;  // 描述
    };
    std::vector<TestCase> testCases = {
        // 基本相交情况
        {
            {100, 100}, {300, 300},  // 线段1: (100,100)-(300,300)
            {100, 300}, {300, 100},   // 线段2: (100,300)-(300,100)
            true,                     // 期望结果：相交
            "X型交叉"                 // 描述
        },
        // 端点相交
        {
            {100, 100}, {300, 300},
            {300, 300}, {500, 100},
            true,
            "端点相交"
        },
        // T型相交
        {
            {100, 200}, {500, 200},
            {300, 100}, {300, 300},
            true,
            "T型相交"
        },
        // 完全不相交
        {
            {100, 100}, {200, 200},
            {400, 400}, {500, 500},
            false,
            "完全不相交"
        },
        // 平行不相交
        {
            {100, 100}, {300, 100},
            {100, 200}, {300, 200},
            false,
            "平行不相交"
        },
        // 共线但不重叠
        {
            {100, 100}, {200, 100},
            {300, 100}, {400, 100},
            false,
            "共线但不重叠"
        },
        // 共线且部分重叠
        {
            {100, 100}, {300, 100},
            {200, 100}, {400, 100},
            true,
            "共线且部分重叠"
        },
        // 共线且完全包含
        {
            {100, 100}, {500, 100},
            {200, 100}, {400, 100},
            true,
            "共线且一个线段包含另一个"
        },
        // 端点刚好触碰
        {
            {100, 100}, {300, 300},
            {300, 300}, {500, 100},
            true,
            "端点刚好触碰"
        },
        // 一个线段垂直于x轴，一个平行于x轴
        {
            {300, 100}, {300, 300},
            {100, 200}, {500, 200},
            true,
            "垂直相交"
        },
        // 一个点在另一条线段上
        {
            {100, 100}, {500, 100},
            {300, 100}, {300, 300},
            true,
            "一个端点在另一线段上"
        },
        // 特殊情况：非常接近但不相交
        {
            {100, 100}, {300, 101},
            {200, 102}, {200, 300},
            false,
            "非常接近但不相交"
        },
        // 退化情况：点和线段（不相交）
        {
            {300, 300}, {300, 300},
            {100, 100}, {500, 500},
            true,
            "点和线段（相交）"
        },
        // 退化情况：点和线段（相交）
        {
            {300, 300}, {300, 300},
            {100, 300}, {500, 300},
            true,
            "点和线段（相交）"
        },
        // 退化情况：点和点（重合）
        {
            {300, 300}, {300, 300},
            {300, 300}, {300, 300},
            true,
            "点和点（重合）"
        },
        // 退化情况：点和点（不重合）
        {
            {100, 100}, {100, 100},
            {200, 200}, {200, 200},
            false,
            "点和点（不重合）"
        }
    };
    
    std::cout << "======== 线段相交检测测试 ========\n\n";
    
    std::vector<bool> results;
    int passCount = 0;
    
    for (size_t i = 0; i < testCases.size(); i++) {
        const auto& test = testCases[i];
        
        bool result = doSegmentsIntersect(test.p1, test.p2, test.q1, test.q2);
        results.push_back(result);
        
        bool passed = (result == test.expected);
        if (passed) passCount++;
        
        std::cout << "测试 " << std::setw(2) << (i+1) << " ["
                  << std::setw(30) << std::left << test.description << "]: "
                  << (passed ? "通过" : "失败")
                  << " (期望: " << (test.expected ? "相交" : "不相交")
                  << ", 实际: " << (result ? "相交" : "不相交") << ")" << std::endl;
        
        // 打印线段坐标
        std::cout << "    线段1: (" << test.p1[0] << "," << test.p1[1] << ")-("
                  << test.p2[0] << "," << test.p2[1] << ")" << std::endl;
        std::cout << "    线段2: (" << test.q1[0] << "," << test.q1[1] << ")-("
                  << test.q2[0] << "," << test.q2[1] << ")" << std::endl;
        
        if (!passed) {
            std::cout << "    错误：" << (test.expected ? "线段应该相交，但函数返回不相交"
                                                    : "线段不应该相交，但函数返回相交") << std::endl;
        }
        
        std::cout << std::endl;
    }
    
    // 测试对称性（交换参数顺序）
    std::cout << "\n正在检查函数对称性...\n";
    bool symmetricCorrect = true;
    
    for (const auto& test : testCases) {
        bool result1 = doSegmentsIntersect(test.p1, test.p2, test.q1, test.q2);
        bool result2 = doSegmentsIntersect(test.q1, test.q2, test.p1, test.p2);
        
        if (result1 != result2) {
            symmetricCorrect = false;
            std::cout << "对称性错误: " << test.description << std::endl;
            std::cout << "  doSegmentsIntersect(p1,p2,q1,q2) = " << result1 << std::endl;
            std::cout << "  doSegmentsIntersect(q1,q2,p1,p2) = " << result2 << std::endl;
        }
    }
    
    if (symmetricCorrect) {
        std::cout << "对称性检查通过！\n";
    }
    
    // 测试鲁棒性（处理极端情况）
    std::cout << "\n正在测试极端情况...\n";
    
    // 相同点
    Point2D same_point = {0, 0};
    bool extreme1 = doSegmentsIntersect(same_point, same_point, same_point, same_point);
    std::cout << "四点相同: " << (extreme1 ? "相交" : "不相交") 
              << " (应为相交)" << std::endl;
    
    // 非常接近的平行线
    Point2D p1 = {0, 0}, p2 = {100, 0.0001};
    Point2D q1 = {0, 0.0002}, q2 = {100, 0.0003};
    bool extreme2 = doSegmentsIntersect(p1, p2, q1, q2);
    std::cout << "非常接近的平行线: " << (extreme2 ? "相交" : "不相交")
              << " (应为不相交)" << std::endl;
    
    // 大数值测试
    Point2D big1 = {1e9, 1e9}, big2 = {2e9, 2e9};
    Point2D big3 = {1e9, 2e9}, big4 = {2e9, 1e9};
    bool extreme3 = doSegmentsIntersect(big1, big2, big3, big4);
    std::cout << "大数值测试 (相交线): " << (extreme3 ? "相交" : "不相交")
              << " (应为相交)" << std::endl;
    
    // 总结
    std::cout << "\n======== 测试结果总结 ========\n";
    std::cout << "通过: " << passCount << "/" << testCases.size() 
              << " 测试用例 (" << (100.0 * passCount / testCases.size()) << "%)" << std::endl;
    return 0;
}

int main() {
    demo_0();
    // demo_1();
    return 0;
}