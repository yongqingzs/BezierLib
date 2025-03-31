// #include "bezier_api.hpp"
#include "bezier.hpp"
#include <chrono>

using namespace std;
using namespace bezier;

// 初始化数据结构
struct InitData {
    // 使用Point2D替代Matrix
    InitData(
        double point_0_0,
        double point_0_1,
        double target_point_0,
        double target_point_1,
        double r_target,
        double theata_0,
        double target_length,
        double r_min
    ) {
        point_0 = {point_0_0, point_0_1};
        target_point = {target_point_0, target_point_1};
        this->r_target = r_target;
        this->theata_0 = theata_0;
        this->target_length = target_length;
        this->r_min = r_min;
    };

    Point2D point_0;
    Point2D target_point;
    double r_target;
    double theata_0;
    double target_length;
    double r_min;
};

// 使用网格搜索方法测试贝塞尔曲线优化
int demo_circle(
    const Point2D& point_0,
    const Point2D& target_point,
    double r_target,
    double theata_0,
    double target_length,
    double r_min,
    const string& path
)
{
    cout << "使用网格搜索优化贝塞尔曲线..." << endl;
    cout << "起始点: (" << point_0[0] << ", " << point_0[1] << "), 起始角: " << theata_0 << " rad" << endl;
    cout << "目标点: (" << target_point[0] << ", " << target_point[1] << "), 半径: " << r_target << " m" << endl;
    cout << "目标长度: " << target_length << " m, 最小转弯半径: " << r_min << " m" << endl;
    
    /* 测试圆上的最优参数搜索 */
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::tuple<Point2D, Point2D, Point2D> optimal_parameters = findOptimalParameters_Circle(
        point_0, target_point, r_target, theata_0, target_length, r_min
    );

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    
    Point2D p1 = std::get<0>(optimal_parameters);
    Point2D p2 = std::get<1>(optimal_parameters);
    Point2D p3 = std::get<2>(optimal_parameters);

    // 计算并输出曲线特性
    double curve_length = calculateBezierLength(point_0, p1, p2, p3, 1000);
    double max_curvature = findMaxCurvature(point_0, p1, p2, p3, 0.001);
    double min_radius = 1.0 / max_curvature;
    
    cout << "网格搜索结果:" << endl;
    // cout << "  控制点1: (" << p1[0] << ", " << p1[1] << ")" << endl;
    // cout << "  控制点2: (" << p2[0] << ", " << p2[1] << ")" << endl;
    // cout << "  终点: (" << p3[0] << ", " << p3[1] << ")" << endl;
    cout << "  曲线长度: " << curve_length << " m (目标: " << target_length << " m)" << endl;
    cout << "  长度误差: " << fabs(curve_length - target_length) << " m" << endl;
    cout << "  最大曲率: " << max_curvature << ", 最小半径: " << min_radius << " m" << endl;
    cout << "  计算时间: " << duration.count() << " 秒" << endl << endl;

    // 输出到文件
    outputBezierCurvePoints(point_0, p1, p2, p3, target_point, r_target, path, 1000);
    // cout << "曲线点已输出到文件: " << path << endl;

    return 0;
}

// 使用NLopt优化方法测试贝塞尔曲线优化
int demo_nlopt_circle(
    const Point2D& point_0,
    const Point2D& target_point,
    double r_target,
    double theata_0,
    double target_length,
    double r_min,
    const string& path
)
{
    cout << "使用NLopt优化贝塞尔曲线..." << endl;
    cout << "起始点: (" << point_0[0] << ", " << point_0[1] << "), 起始角: " << theata_0 << " rad" << endl;
    
    /* 测试NLopt优化方法 */
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 尝试不同的算法
    // 0: COBYLA, 1: BOBYQA, 2: NELDERMEAD, 3: SBPLX
    std::tuple<Point2D, Point2D, Point2D> optimal_parameters = findNLoptParameters_Circle(
        point_0, target_point, r_target, theata_0, target_length, r_min
    );
    
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    
    Point2D p1 = std::get<0>(optimal_parameters);
    Point2D p2 = std::get<1>(optimal_parameters);
    Point2D p3 = std::get<2>(optimal_parameters);

    // 计算并输出曲线特性
    double curve_length = calculateBezierLength(point_0, p1, p2, p3, 1000);
    double max_curvature = findMaxCurvature(point_0, p1, p2, p3, 0.001);
    double min_radius = 1.0 / max_curvature;
    
    cout << "NLopt优化结果:" << endl;
    // cout << "  控制点1: (" << p1[0] << ", " << p1[1] << ")" << endl;
    // cout << "  控制点2: (" << p2[0] << ", " << p2[1] << ")" << endl;
    // cout << "  终点: (" << p3[0] << ", " << p3[1] << ")" << endl;
    cout << "  曲线长度: " << curve_length << " m (目标: " << target_length << " m)" << endl;
    cout << "  长度误差: " << fabs(curve_length - target_length) << " m" << endl;
    cout << "  最大曲率: " << max_curvature << ", 最小半径: " << min_radius << " m" << endl;
    cout << "  计算时间: " << duration.count() << " 秒" << endl << endl;

    // 输出到文件
    outputBezierCurvePoints(point_0, p1, p2, p3, target_point, r_target, path, 1000);
    // cout << "曲线点已输出到文件: " << path << endl;

    return 0;
}

int demo_multi()
{
    // 创建多个弹的数据
    std::vector<bezier::BezierData> missiles;
    
    bezier::BezierData m1;
    m1.start_point = {-14922.974681, -13310.281983};     // 起始点
    m1.theta0 = 0.640792;                 // 起始航向 (弧度，0表示向东)
    m1.speed = 200;                 // 速度
    m1.r_min = 2373.839956;                  // 最小转弯半径
    missiles.push_back(m1);
    
    bezier::BezierData m2;
    m2.start_point = {-14920.476178, -9972.019593};    // 不同的起始点
    m2.theta0 = 0.641107;             // 向西
    m2.speed = 200;                  // 不同的速度
    m2.r_min = 2373.839956;                  // 不同的最小转弯半径
    missiles.push_back(m2);
    
    bezier::BezierData m3;
    m3.start_point = {-14919.097693, -6631.636326};  // 不同的起始点
    m3.theta0 = 0.641872;              // 向北
    m3.speed = 200;                 // 不同的速度
    m3.r_min = 2373.839956;                  // 不同的最小转弯半径
    missiles.push_back(m3);
    
    bezier::Point2D target = {0, 0};
    double radius = 8000;
    
    // 计算最优路径
    auto paths = bezier::optimizeMultiMissilePaths(missiles, target, radius);
    
    // 输出路径点到文件，方便可视化
    for (int i = 0; i < paths.size(); i++) {
        // auto [p0, p1, p2, p3] = paths[i];
        bezier::Point2D p0 = std::get<0>(paths[i]);
        bezier::Point2D p1 = std::get<1>(paths[i]);
        bezier::Point2D p2 = std::get<2>(paths[i]);
        bezier::Point2D p3 = std::get<3>(paths[i]);
        
        std::string filename = "../output/missile_" + std::to_string(i+1) + "_path.txt";
        bezier::outputBezierCurvePoints(p0, p1, p2, p3, target, radius, filename, 100);
    }
    return 0;
}

int demo_multi_txt()
{
    std::vector<bezier::BezierData> missiles;
    bezier::Point2D target;
    double radius = 0.0;
    double target_length = 0.0;
    double rad_min = 0.0;

    loadMissileDataFromFile("../data/multi_missile.txt", missiles, target, radius, target_length, rad_min);
    
    // 计算最优路径
    auto paths = bezier::optimizeMultiMissilePaths(missiles, target, radius);
    
    // 输出路径点到文件，方便可视化
    for (int i = 0; i < paths.size(); i++) {
        // auto [p0, p1, p2, p3] = paths[i];
        bezier::Point2D p0 = std::get<0>(paths[i]);
        bezier::Point2D p1 = std::get<1>(paths[i]);
        bezier::Point2D p2 = std::get<2>(paths[i]);
        bezier::Point2D p3 = std::get<3>(paths[i]);
        
        std::string filename = "../output/missile_" + std::to_string(i+1) + "_path.txt";
        bezier::outputBezierCurvePoints(p0, p1, p2, p3, target, radius, filename, 100);

        // 检查每个曲线的最大曲率
        double max_curvature = bezier::findMaxCurvature(p0, p1, p2, p3, 0.01);
        double tmp_radius = 1.0 / max_curvature;
        std::cout << "弹道 " << (i+1) <<  " R_min: " << rad_min << ", R_true: " << tmp_radius << std::endl;
    }
    return 0;
}

int main()
{
    /* 
            x               y               heading(rad)
    c0:     -14922.974681   -13310.281983   0.640792
    c1:     -14920.476178   -9972.019593    0.641107
    c2:     -14919.097693   -6631.636326    0.641872
    tar:    0               0
    RADIUS:8000.000000
    LENGTH:13996.469183
    RAD_MIN:2000.000000
     */
#if 0  // 单DD测试用例
    // 使用新的InitData结构创建测试案例
    InitData c0(-14922.974681, -13310.281983, 0, 0, 8000, 0.640792, 13996.469183, 2000);
    InitData c1(-14920.476178, -9972.019593, 0, 0, 8000, 0.641107, 13996.469183, 2000);
    InitData c2(-14919.097693, -6631.636326, 0, 0, 8000, 0.641872, 13996.469183, 2000);

    // 输出文件路径
    string point_0 = "../output/bezier_curve_c0.txt";
    string point_1 = "../output/bezier_curve_c1.txt";
    string point_2 = "../output/bezier_curve_c2.txt";

    // 执行网格搜索优化
    cout << "==========================网格搜索优化==========================" << endl;
    demo_circle(c0.point_0, c0.target_point, c0.r_target, c0.theata_0, c0.target_length, c0.r_min, point_0);
    demo_circle(c1.point_0, c1.target_point, c1.r_target, c1.theata_0, c1.target_length, c1.r_min, point_1);
    demo_circle(c2.point_0, c2.target_point, c2.r_target, c2.theata_0, c2.target_length, c2.r_min, point_2);
    
    // 执行NLopt优化
    cout << "==========================NLopt优化==========================" << endl;
    demo_nlopt_circle(c0.point_0, c0.target_point, c0.r_target, c0.theata_0, c0.target_length, c0.r_min, point_0);
    demo_nlopt_circle(c1.point_0, c1.target_point, c1.r_target, c1.theata_0, c1.target_length, c1.r_min, point_1);
    demo_nlopt_circle(c2.point_0, c2.target_point, c2.r_target, c2.theata_0, c2.target_length, c2.r_min, point_2);
#endif

#if 1  // 多DD测试用例
    // demo_multi();
    demo_multi_txt();
#endif

    return 0;
}