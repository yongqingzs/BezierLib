#include "bezier.hpp"

/* 
Waypoints: {
       {14.8504, 35.5802, 52.7557},
       {14.8513, 35.5816, 52.7678},
       {14.8541, 35.5841, 52.7607},
       {14.8555, 35.5857, 52.7688},
       {14.8577, 35.5878, 52.7665},
       {14.8599, 35.5899, 52.7640},
};
*/

int test_base()
{
    using namespace bezier;
    double c[6][3] = {
        {14.8504, 35.5802, 52.7557},
        {14.8513, 35.5816, 52.7678},
        {14.8541, 35.5841, 52.7607},
        {14.8555, 35.5857, 52.7688},
        {14.8577, 35.5878, 52.7665},
        {14.8599, 35.5899, 52.7640},
    };
    double target[3] = {15.0038, 35.7029, 0.0};
    double target_r = 10000.0;
    double target_l = 50000;
    double r_min = 6000;

    double distance = 0;
    double azimuth = 0;
    auto vin_result = bezier::inverseVincenty(c[0][0] * RAD, c[0][1] * RAD, c[1][0] * RAD, c[1][1] * RAD);
    std::cout << "distance: " << std::get<0>(vin_result) << std::endl;
    std::cout << "azimuth1: " << std::get<1>(vin_result) * DEG << std::endl;
    std::cout << "azimuth2: " << std::get<1>(vin_result) * DEG << std::endl;
    std::cout << std::endl;
    vin_result = bezier::inverseVincenty(c[0][0] * RAD, c[0][1] * RAD, target[0] * RAD, target[1] * RAD);
    std::cout << "distance: " << std::get<0>(vin_result) << std::endl;
    std::cout << "azimuth1: " << std::get<1>(vin_result) * DEG << std::endl;
    std::cout << "azimuth2: " << std::get<1>(vin_result) * DEG << std::endl;

    double con_input[3] = {c[0][0]* RAD, c[0][1]* RAD, 0.0};
    auto con_result = convertGeoToENU(
        con_input[0], con_input[1], 0.0,
        target[0] * RAD, target[1] * RAD, 0.0
    );
    std::cout << "\nGEO: " << con_input[0] * DEG << ", " << con_input[1] * DEG << ", " << con_input[2] * DEG << std::endl;
    std::cout << "ENU: " << con_result[0] << ", " << con_result[1] << ", " << con_result[2] << std::endl;
    con_result = convertENUToGeo(
        con_result[0], con_result[1], con_result[2],
        target[0] * RAD, target[1] * RAD, 0.0
    );
    std::cout << "GEO: " << con_result[0] * DEG << ", " << con_result[1] * DEG << ", " << con_result[2] * DEG<< std::endl;

    con_input[0] = c[2][0] * RAD;
    con_input[1] = c[2][1] * RAD;
    con_result = convertGeoToENU(
        con_input[0], con_input[1], 0.0,
        target[0] * RAD, target[1] * RAD, 0.0
    );
    std::cout << "\nGEO: " << con_input[0] * DEG << ", " << con_input[1] * DEG << ", " << con_input[2] * DEG << std::endl;
    std::cout << "ENU: " << con_result[0] << ", " << con_result[1] << ", " << con_result[2] << std::endl;
    con_result = convertENUToGeo(
        con_result[0], con_result[1], 0,
        target[0] * RAD, target[1] * RAD, 0.0
    );
    std::cout << "GEO: " << con_result[0] * DEG << ", " << con_result[1] * DEG << ", " << con_result[2] * DEG << std::endl;

    return 0;
}

int test_dense()
{
    using namespace bezier;
    double c[6][3] = {
        {14.8504, 35.5802, 52.7557},
        {14.8513, 35.5816, 52.7678},
        {14.8541, 35.5841, 52.7607},
        {14.8555, 35.5857, 52.7688},
        {14.8577, 35.5878, 52.7665},
        {14.8599, 35.5899, 52.7640},
    };
    double target[3] = {15.0038, 35.7029, 0.0};

    for (int i = 0; i < 6; i++)
    {
        c[i][0] = c[i][0] * RAD;
        c[i][1] = c[i][1] * RAD;
        c[i][2] = bezier::PI/2 - c[i][2] * RAD;
    }
    target[0] = target[0] * RAD;
    target[1] = target[1] * RAD;
    
    double target_r = 5000.0;
    auto c0_result = inverseVincenty(
        c[0][0], c[0][1], target[0], target[1]
    );
    double c0_dis = std::get<0>(c0_result);
    double c0_azimuth = std::get<1>(c0_result);
    double target_l = c0_dis - target_r;
    double r_min = 3500;

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
    opt.num_samlpes = 10;                // 采样点数量
    opt.target_length = target_l;           // 期望路径长度
    opt.target_radius = target_r;            // 目标半径
    opt.fixed_angle = bezier::PI + c0_azimuth;       // 固定角度(弧度)
    opt.lower_bounds_first = {};
    opt.upper_bounds_first = {};
    opt.x_init_first = {};
    opt.lower_bounds_second = {opt.target_length - 2000, opt.fixed_angle - 0.5};
    opt.upper_bounds_second = {opt.target_length + 8000, opt.fixed_angle + 2};
    opt.x_init_second = {};
    opt.layer = true;                // 是否为分层优化
    opt.use_unified_opt = true;  // 使用集合优化
    opt.opt_type = 0;  // 0: 3 order
    opt.algo_first = nlopt::LN_AUGLAG_EQ;    // 优化算法
    opt.algo_second = nlopt::GN_ISRES;

    std::vector<std::array<double, 4>> paths = bezier::measureTime(
        [&]() {return bezier::generateGeoPath(init, opt);},
         "bezier_opt");
    
    // out路径需要根据实际执行路径修改
    bezier::outputMultiPathPoints(paths, init.target_point, opt.target_radius, opt.num_samlpes, "out");

    return 0;
}

int main()
{
    test_dense();
    return 0;
}