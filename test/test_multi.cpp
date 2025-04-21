#include "bezier.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

/*
        x               y               heading(rad)
Target: 0.000000        0.000000        -
c1:     -13854.162565   -13642.381203   0.650036
c2:     -13767.754324   -13491.681586   0.649824
c3:     -13517.080820   -13215.950545   0.649948
c4:     -13395.398156   -13038.395554   0.649807
c5:     -13197.423769   -12802.778147   0.649848
c6:     -12996.951312   -12565.258044   0.649890
RADIUS:10000.000000
LENGTH:9443.569252
RAD_MIN:3500.000000
*/

int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    double c[6][3] = {
        {-13854.162565, -13642.381203, 0.650036},
        {-13767.754324, -13491.681586, 0.649824},
        {-13517.080820, -13215.950545, 0.649948},
        {-13395.398156, -13038.395554, 0.649807},
        {-13197.423769, -12802.778147, 0.649848},
        {-12996.951312, -12565.258044, 0.649890}
    };
    double target[3] = {0.0, 0.0, 0.0};
    double target_r = 10000.0;
    double target_l = 9443.569252;
    double r_min = 3500.0;

    // 创建初始数据结构
    bezier::InitData init;
    init.nodes.resize(6);
    init.node_num = 6;
    for (int i = 0; i < 6; i++)
    {
        init.nodes[i].start_point = {c[i][0], c[i][1]};
        init.nodes[i].heading = c[i][2];  // 航向角(弧度)，0表示正北方向
        init.nodes[i].r_min = r_min;  // 最小转弯半径
    }
    init.target_point = {target[0], target[1]};
    
    // 设置优化参数
    bezier::OptParms opt;
    opt.layer = true;                // 是否为分层优化
    opt.opt_type = 0;                    // 0表示使用固定角度优化
    opt.num_samlpes = 10;                // 采样点数量(注意拼写错误)
    opt.target_length = target_l;           // 期望路径长度
    opt.target_radius = target_r;            // 目标半径
    // opt.fixed_angle = 3.14159 / 1;       // 固定角度(弧度)
    opt.algorithm = nlopt::LN_COBYLA;    // 优化算法
    
    std::cout << "\n3 order" << std::endl;
    std::vector<std::array<double, 4>> paths = bezier::measureTime(
        [&]() {return bezier::generateBezierPath(init, opt);},
         "3 order");
    
    std::cout << "\n5 order" << std::endl;
    opt.opt_type = 1;
    paths = bezier::measureTime(
        [&]() {return bezier::generateBezierPath(init, opt);},
         "5 order");
    
    return 0;
}