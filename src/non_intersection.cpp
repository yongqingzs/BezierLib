#include "bezier.hpp"

namespace bezier {

/**
 * 判断两条线段是否相交
 * 使用跨立实验和快速排斥实验
 * 
 * @param p1 第一条线段的起点
 * @param p2 第一条线段的终点
 * @param q1 第二条线段的起点
 * @param q2 第二条线段的终点
 * @return 线段是否相交
 */
bool doSegmentsIntersect(const Point2D& p1, const Point2D& p2, const Point2D& q1, const Point2D& q2)
{
    // 快速排斥实验 (Fast Rejection Test)
    // 如果两条线段的包围盒不相交，则线段一定不相交
    if (std::max(p1[0], p2[0]) < std::min(q1[0], q2[0]) ||
        std::max(q1[0], q2[0]) < std::min(p1[0], p2[0]) ||
        std::max(p1[1], p2[1]) < std::min(q1[1], q2[1]) ||
        std::max(q1[1], q2[1]) < std::min(p1[1], p2[1])) {
        return false;
    }
    
    // 跨立实验 (Cross Test)
    // 两线段相交当且仅当：
    // 1. (p1, p2) 跨立 (q1, q2)
    // 2. (q1, q2) 跨立 (p1, p2)
    
    // 计算叉积
    auto cross = [](const Point2D& a, const Point2D& b, const Point2D& c) -> double {
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    };
    
    // 判断 (q1, q2) 是否跨立 (p1, p2)
    double d1 = cross(p1, p2, q1);
    double d2 = cross(p1, p2, q2);
    
    // 如果 d1 和 d2 同号，或者其中一个为0（线段与直线重合），则不相交
    if (d1 * d2 > 0) {
        return false;
    }
    
    // 判断 (p1, p2) 是否跨立 (q1, q2)
    double d3 = cross(q1, q2, p1);
    double d4 = cross(q1, q2, p2);
    
    // 如果 d3 和 d4 同号，或者其中一个为0（线段与直线重合），则不相交
    if (d3 * d4 > 0) {
        return false;
    }
    
    // 特殊情况：共线但重叠
    // 如果四个叉积都为0，则两线段共线，需要判断是否重叠
    if (d1 == 0 && d2 == 0 && d3 == 0 && d4 == 0) {
        // 确定两条线段在x轴和y轴上的投影是否重叠
        if (std::max(p1[0], p2[0]) < std::min(q1[0], q2[0]) ||
            std::max(q1[0], q2[0]) < std::min(p1[0], p2[0]) ||
            std::max(p1[1], p2[1]) < std::min(q1[1], q2[1]) ||
            std::max(q1[1], q2[1]) < std::min(p1[1], p2[1])) {
            return false; // 投影不重叠
        }
        return true; // 投影重叠，线段重叠
    }
    
    // 如果两线段互相跨立，则相交
    return true;
}

/**
 * 贝塞尔曲线相交检测
 * @brief 使用层次化方法：先检查控制多边形的轴对齐包围盒(AABB)，再检查精确相交
 */
double checkBezierIntersectionEfficient(
    const Point2D& p0_1, const Point2D& p1_1, const Point2D& p2_1, const Point2D& p3_1,
    const Point2D& p0_2, const Point2D& p1_2, const Point2D& p2_2, const Point2D& p3_2,
    double safety_distance)
{
    // 第一级快速检测：计算AABB
    double min_x1 = std::min({p0_1[0], p1_1[0], p2_1[0], p3_1[0]});
    double max_x1 = std::max({p0_1[0], p1_1[0], p2_1[0], p3_1[0]});
    double min_y1 = std::min({p0_1[1], p1_1[1], p2_1[1], p3_1[1]});
    double max_y1 = std::max({p0_1[1], p1_1[1], p2_1[1], p3_1[1]});
    
    double min_x2 = std::min({p0_2[0], p1_2[0], p2_2[0], p3_2[0]});
    double max_x2 = std::max({p0_2[0], p1_2[0], p2_2[0], p3_2[0]});
    double min_y2 = std::min({p0_2[1], p1_2[1], p2_2[1], p3_2[1]});
    double max_y2 = std::max({p0_2[1], p1_2[1], p2_2[1], p3_2[1]});
    
    // 考虑安全距离 - 扩展包围盒
    min_x1 -= safety_distance; 
    min_y1 -= safety_distance;
    max_x1 += safety_distance;
    max_y1 += safety_distance;
    
    // 如果包围盒不相交，曲线肯定不相交
    if (max_x1 < min_x2 || min_x1 > max_x2 || max_y1 < min_y2 || min_y1 > max_y2) {
        // // 计算包围盒间的最小距离
        // double dx = std::max(0.0, std::max(min_x2 - max_x1, min_x1 - max_x2));
        // double dy = std::max(0.0, std::max(min_y2 - max_y1, min_y1 - max_y2));
        // double box_distance = std::sqrt(dx*dx + dy*dy);
        
        // // 返回负值表示不相交(安全距离减去包围盒间距)
        // return safety_distance - (box_distance + safety_distance);
        return -safety_distance;  // 不相交，返回负值表示不相交
    }
    
    return checkBezierIntersectionBySampling(
        p0_1, p1_1, p2_1, p3_1,
        p0_2, p1_2, p2_2, p3_2,
        safety_distance, 10  // 每条曲线只需少量采样点
    );
}

/**
 * 检查两个线段之间的最小距离
 */
double checkSegmentDistance(
    const Point2D& p0, const Point2D& p1,
    const Point2D& q0, const Point2D& q1,
    double safety_distance)
{
    // 线段相交检测
    if (doSegmentsIntersect(p0, p1, q0, q1)) {
        return safety_distance;  // 相交，返回最大违反值
    }
    
    // // 计算线段间最小距离
    // auto pointToSegmentDistance = [](const Point2D& p, const Point2D& s1, const Point2D& s2) -> double {
    //     double l2 = std::pow(s2[0] - s1[0], 2) + std::pow(s2[1] - s1[1], 2);
    //     if (l2 < 1e-10) return std::sqrt(std::pow(p[0] - s1[0], 2) + std::pow(p[1] - s1[1], 2));
        
    //     double t = std::max(0.0, std::min(1.0, 
    //         ((p[0] - s1[0]) * (s2[0] - s1[0]) + (p[1] - s1[1]) * (s2[1] - s1[1])) / l2));
        
    //     double px = s1[0] + t * (s2[0] - s1[0]);
    //     double py = s1[1] + t * (s2[1] - s1[1]);
        
    //     return std::sqrt(std::pow(p[0] - px, 2) + std::pow(p[1] - py, 2));
    // };
    
    // double d1 = pointToSegmentDistance(p0, q0, q1);
    // double d2 = pointToSegmentDistance(p1, q0, q1);
    // double d3 = pointToSegmentDistance(q0, p0, p1);
    // double d4 = pointToSegmentDistance(q1, p0, p1);
    
    // double min_distance = std::min({d1, d2, d3, d4});
    
    // return min_distance - safety_distance;
    return -safety_distance;  // 不相交，返回负值表示不相交
}

/**
 * 使用有限采样点检测两条贝塞尔曲线是否相交
 * 仅在递归细分后的小曲线段上使用，采样点数可以很少
 */
double checkBezierIntersectionBySampling(
    const Point2D& p0_1, const Point2D& p1_1, const Point2D& p2_1, const Point2D& p3_1,
    const Point2D& p0_2, const Point2D& p1_2, const Point2D& p2_2, const Point2D& p3_2,
    double safety_distance, int num_samples)
{
    // 为两条短曲线生成少量采样点
    std::vector<Point2D> curve1_points;
    std::vector<Point2D> curve2_points;
    
    for (int i = 0; i <= num_samples; i++) {
        double t = static_cast<double>(i) / num_samples;
        
        // 计算曲线1上的点
        double x1 = std::pow(1-t, 3) * p0_1[0] + 3 * std::pow(1-t, 2) * t * p1_1[0] + 
                  3 * (1-t) * std::pow(t, 2) * p2_1[0] + std::pow(t, 3) * p3_1[0];
        double y1 = std::pow(1-t, 3) * p0_1[1] + 3 * std::pow(1-t, 2) * t * p1_1[1] + 
                  3 * (1-t) * std::pow(t, 2) * p2_1[1] + std::pow(t, 3) * p3_1[1];
        
        curve1_points.push_back({x1, y1});
        
        // 计算曲线2上的点
        double x2 = std::pow(1-t, 3) * p0_2[0] + 3 * std::pow(1-t, 2) * t * p1_2[0] + 
                  3 * (1-t) * std::pow(t, 2) * p2_2[0] + std::pow(t, 3) * p3_2[0];
        double y2 = std::pow(1-t, 3) * p0_2[1] + 3 * std::pow(1-t, 2) * t * p1_2[1] + 
                  3 * (1-t) * std::pow(t, 2) * p2_2[1] + std::pow(t, 3) * p3_2[1];
        
        curve2_points.push_back({x2, y2});
    }
    
    // 在采样点之间近似为线段，检查每对线段
    double min_distance = -safety_distance;
    
    for (size_t i = 0; i < curve1_points.size() - 1; i++) {
        for (size_t j = 0; j < curve2_points.size() - 1; j++) {
            double seg_dist = checkSegmentDistance(
                curve1_points[i], curve1_points[i+1],
                curve2_points[j], curve2_points[j+1],
                safety_distance
            );
            
            min_distance = std::max(min_distance, seg_dist);
        }
    }
    
    return min_distance;
}

/**
 * 多弹不相交约束函数
 */
double multi_missile_intersection_constraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    MultiMissileOptData *opt_data = reinterpret_cast<MultiMissileOptData*>(data);
    int num_missiles = opt_data->missiles.size();
    
    // 如果只有一条弹道，不存在交叉问题
    if (num_missiles <= 1) return -1.0;
    
    // 计算所有弹道的控制点
    std::vector<Point2D> p0s(num_missiles);
    std::vector<Point2D> p1s(num_missiles);
    std::vector<Point2D> p2s(num_missiles);
    std::vector<Point2D> p3s(num_missiles);
    
    for (int i = 0; i < num_missiles; i++) {
        int base_idx = i * 3;
        double angle = x[base_idx];
        double d0 = x[base_idx + 1];
        double d3 = x[base_idx + 2];
        
        const BezierData& missile = opt_data->missiles[i];
        
        p0s[i] = missile.start_point;
        
        p3s[i] = {
            opt_data->target_point->at(0) + opt_data->radius * std::cos(angle),
            opt_data->target_point->at(1) + opt_data->radius * std::sin(angle)
        };
        
        p1s[i] = {
            p0s[i][0] + d0 * std::cos(missile.theta0),
            p0s[i][1] + d0 * std::sin(missile.theta0)
        };
        
        p2s[i] = {
            p3s[i][0] + d3 * std::cos(angle),
            p3s[i][1] + d3 * std::sin(angle)
        };
    }
    
    // 检查所有弹道对之间的交叉情况
    // double max_violation = -std::numeric_limits<double>::max();
    double sum_violation = 0;
    double safety_distance = 500.0;  // 最小安全距离，根据实际需要调整
    
    for (int i = 0; i < num_missiles - 1; i++) {
        for (int j = i + 1; j < num_missiles; j++) {
            double intersection = checkBezierIntersectionEfficient(
                p0s[i], p1s[i], p2s[i], p3s[i],
                p0s[j], p1s[j], p2s[j], p3s[j],
                safety_distance
            );

            // max_violation = std::max(max_violation, intersection);
            sum_violation += intersection;
        }
    }
    
    return sum_violation;
}

}