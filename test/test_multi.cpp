#include "bezier.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

/* 
1.
double c[6][3] = {
    {-13854.162565, -13642.381203, 0.650036},
    {-13767.754324, -13491.681586, 0.649824},
    {-13517.080820, -13215.950545, 0.649948},
    {-13395.398156, -13038.395554, 0.649807},
    {-13197.423769, -12802.778147, 0.649848},
    {-12996.951312, -12565.258044, 0.649890}
};
double target[3] = {0.0, 0.0, 0.0};
double target_r = 5000.0;
double target_l = 9443.569252 + target_r;
double r_min = 3500.0;

2.
double c[6][3] = {
    {-15283.902929, -47606.746489, 1.260146},
    {-15116.767973, -47660.081054, 1.263655},
    {-14949.446907, -47712.828853, 1.267163},
    {-14781.941791, -47764.989238, 1.270672},
    {-14614.254687, -47816.561565, 1.274181},
    {-14446.387661, -47867.545201, 1.277690}
};
double target[3] = {0.0, 0.0, 0.0};
double target_r = 5000.0;
double target_l = 50000;
double r_min = 8000;

*/

int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    double c[6][3] = {
        {-15283.902929, -47606.746489, 1.260146},
        {-15116.767973, -47660.081054, 1.263655},
        {-14949.446907, -47712.828853, 1.267163},
        {-14781.941791, -47764.989238, 1.270672},
        {-14614.254687, -47816.561565, 1.274181},
        {-14446.387661, -47867.545201, 1.277690}
    };
    double target[3] = {0.0, 0.0, 0.0};
    double target_r = 10000.0;
    double target_l = 50000;
    double r_min = 6000;

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
    opt.num_samlpes = 10;                // 采样点数量(注意拼写错误)
    opt.target_length = target_l;           // 期望路径长度
    opt.target_radius = target_r;            // 目标半径
    opt.fixed_angle = bezier::PI * 0.85;       // 固定角度(弧度)
    opt.lower_bounds_first = {};
    opt.upper_bounds_first = {};
    opt.x_init_first = {};
    opt.lower_bounds_second = {opt.target_length - 2000, opt.fixed_angle - 0.5};
    opt.upper_bounds_second = {opt.target_length + 20000, opt.fixed_angle + 2};
    opt.x_init_second = {};

    /* 
    无梯度算法（不需要提供导数）
    局部优化算法
    nlopt::LN_COBYLA: 通过线性近似的约束优化（您目前使用的）
    nlopt::LN_AUGLAG: 增广拉格朗日方法，使用其他无梯度算法作为子算法
    nlopt::LN_AUGLAG_EQ: 增广拉格朗日方法（只处理等式约束）

    全局优化算法
    nlopt::GN_ISRES: 改进的随机排序进化策略，支持非线性约束
    nlopt::GN_ORIG_DIRECT: 原始DIRECT算法，支持约束
    nlopt::GN_ORIG_DIRECT_L: 局部偏好的DIRECT算法，支持约束
    nlopt::GN_AGS: 自适应全局搜索算法，支持约束

    梯度算法（需要提供导数）
    局部优化算法
    nlopt::LD_SLSQP: 序列最小二乘二次规划，支持非线性约束
    nlopt::LD_MMA: 移动渐近线法，支持约束
    nlopt::LD_CCSAQ: 保守凸可分离近似法，支持约束
    nlopt::LD_AUGLAG: 增广拉格朗日方法（使用导数），需要其他局部梯度算法作为子算法

    全局优化算法
    nlopt::GD_MLSL: 多层次单链接算法，可以处理约束
    nlopt::GD_MLSL_LDS: 带低差异序列的MLSL，更强的全局搜索能力
    */

    /* 
    不支持联合优化这种较多参数
    GN_ORIG_DIRECT
    GN_ORIG_DIRECT_L
    GN_AGS
    不支持梯度算法
    */
     
    /** 较好
     * LN_COBYLA
     * LN_AUGLAG_EQ
     */
    opt.layer = true;                // 是否为分层优化
    opt.use_unified_opt = true;  // 使用集合优化
    opt.opt_type = 0;  // 0: 3 order
    opt.algo_first = nlopt::LN_AUGLAG_EQ;    // 优化算法
    opt.algo_second = nlopt::GN_ISRES;

    std::vector<std::array<double, 4>> paths = bezier::measureTime(
        [&]() {return bezier::generateBezierPath(init, opt);},
         "");
    bezier::outputMultiPathPoints(paths, init.target_point, opt.target_radius, opt.num_samlpes, "../../out");
    
    return 0;
}