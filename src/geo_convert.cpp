#include "bezier.hpp"

namespace bezier {

constexpr double a = 6378137.0;  // WGS84椭球体长半轴(m)
constexpr double f = 1/298.257223563;  // WGS84扁率
constexpr double e2 = 2*f - f*f;  // 第一偏心率平方
constexpr double b = a * (1 - f);

// 将经纬度坐标直接转换为ECEF坐标
std::array<double, 3> convertGeoToECEF(double lon, double lat, double h = 0.0) {
    double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));
    double X = (N + h) * cos(lat) * cos(lon);
    double Y = (N + h) * cos(lat) * sin(lon);
    double Z = (N * (1 - e2) + h) * sin(lat);
    
    return {X, Y, Z};
}

// 将ECEF坐标转换为经纬度坐标
std::array<double, 3> convertECEFToGeo(double X, double Y, double Z) {
    double p = sqrt(X*X + Y*Y);
    double theta = atan2(Z*a, p*(1-e2)*a);
    
    double lat = atan2(Z + e2*(1-e2)*a*sin(theta)*sin(theta)*sin(theta)/(1-e2), 
                        p - e2*a*cos(theta)*cos(theta)*cos(theta));
    double lon = atan2(Y, X);
    double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));
    double h = p/cos(lat) - N;  // 高程
    
    return {lon, lat, h};
}

// 将ECEF坐标转换为局部ENU坐标
std::array<double, 3> convertECEFToENU(
    double X, double Y, double Z,
    double X0, double Y0, double Z0,
    double lat0, double lon0
) {
    // 计算坐标差
    double dX = X - X0;
    double dY = Y - Y0;
    double dZ = Z - Z0;
    
    // 转换到ENU坐标系
    double E = -sin(lon0) * dX + cos(lon0) * dY;
    double N = -sin(lat0) * cos(lon0) * dX - sin(lat0) * sin(lon0) * dY + cos(lat0) * dZ;
    double U = cos(lat0) * cos(lon0) * dX + cos(lat0) * sin(lon0) * dY + sin(lat0) * dZ;
    
    return {E, N, U};
}

// 将局部ENU坐标转换为ECEF坐标
std::array<double, 3> convertENUToECEF(
    double E, double N, double U,
    double X0, double Y0, double Z0,
    double lat0, double lon0
) {
    // 构建转换矩阵
    double dX = -sin(lon0) * E - sin(lat0) * cos(lon0) * N + cos(lat0) * cos(lon0) * U;
    double dY = cos(lon0) * E - sin(lat0) * sin(lon0) * N + cos(lat0) * sin(lon0) * U;
    double dZ = cos(lat0) * N + sin(lat0) * U;
    
    // 计算ECEF坐标
    double X = X0 + dX;
    double Y = Y0 + dY;
    double Z = Z0 + dZ;
    
    return {X, Y, Z};
}

// 将经纬度坐标转换为局部ENU坐标
BEZIER_API std::array<double, 3> convertGeoToENU(
    double lon, double lat, double h,
    double lon0, double lat0, double h0
) {
    // 将参考点(lon0, lat0)转换为ECEF坐标
    auto origin_ecef = convertGeoToECEF(lon0, lat0, h0);
    double X0 = origin_ecef[0];
    double Y0 = origin_ecef[1];
    double Z0 = origin_ecef[2];
    
    // 将目标点(lon, lat)转换为ECEF坐标
    auto target_ecef = convertGeoToECEF(lon, lat, h);
    
    // 将ECEF坐标转换为ENU坐标
    return convertECEFToENU(
        target_ecef[0], target_ecef[1], target_ecef[2],
        X0, Y0, Z0,
        lat0, lon0
    );
}

// 将局部ENU坐标转换为经纬度坐标
BEZIER_API std::array<double, 3> convertENUToGeo(
    double E, double N, double U,
    double lon0, double lat0, double h0
) {
    // 将参考点(lon0, lat0)转换为ECEF坐标
    auto origin_ecef = convertGeoToECEF(lon0, lat0, h0);
    double X0 = origin_ecef[0];
    double Y0 = origin_ecef[1];
    double Z0 = origin_ecef[2];
    
    // 将ENU坐标转换为ECEF坐标
    auto ecef = convertENUToECEF(E, N, U, X0, Y0, Z0, lat0, lon0);
    
    // 将ECEF坐标转换为经纬度坐标
    return convertECEFToGeo(ecef[0], ecef[1], ecef[2]);
}

// 地理坐标(经纬度)转局部ENU坐标系
InitData convertGeoToLocal(const InitData& init_geo) {
    // 使用目标点作为坐标原点(输入经纬度已是弧度)
    double lat0 = init_geo.target_point[1];  // 纬度(弧度)
    double lon0 = init_geo.target_point[0];  // 经度(弧度)
    
    // 转换为局部坐标系的初始数据
    InitData init_local = init_geo;
    init_local.target_point = {0.0, 0.0};  // 局部坐标系中目标点为原点
    
    // 转换各个节点的坐标
    for (auto& node : init_local.nodes) {
        double lat = node.start_point[1];  // 纬度(弧度)
        double lon = node.start_point[0];  // 经度(弧度)
        double h = 0.0;  // 假设高程为0
        
        // 使用辅助函数：直接从经纬度转换到ENU
        auto enu = convertGeoToENU(lon, lat, h, lon0, lat0);
        
        node.start_point = {enu[0], enu[1]};  // 我们只关心平面坐标 (东北坐标)
        
        // // 航向角修正 - 从全球坐标系到局部坐标系 (保留在主函数中)
        // double heading_correction = atan2(sin(lon - lon0) * cos(lat), 
        //                               cos(lat0) * sin(lat) - sin(lat0) * cos(lat) * cos(lon - lon0));
        // node.heading = node.heading - heading_correction;
    }
    
    return init_local;
}

// 局部ENU坐标转回地理坐标(经纬度)
std::vector<std::array<double, 4>> convertLocalToGeo(
    const std::vector<std::array<double, 4>>& path_local, 
    const Point2D& origin_geo) {
    // 原点的地理坐标(弧度)
    double lat0 = origin_geo[1];  // 纬度(弧度)
    double lon0 = origin_geo[0];  // 经度(弧度)
    
    // 将局部坐标转换回经纬度
    std::vector<std::array<double, 4>> path_geo;
    path_geo.reserve(path_local.size());
    
    for (const auto& point : path_local) {
        double E = point[0];  // 东向坐标
        double N = point[1];  // 北向坐标
        double U = 0.0;       // 上向坐标（假设为0）
        
        // 使用辅助函数：直接从ENU转换到经纬度
        auto geo = convertENUToGeo(E, N, U, lon0, lat0);
        
        double lon = geo[0];
        double lat = geo[1];
        double h = geo[2];
        
        // 航向角修正 - 从局部坐标系到全球坐标系 (保留在主函数中)
        double heading_correction = atan2(sin(lon - lon0) * cos(lat), 
                                      cos(lat0) * sin(lat) - sin(lat0) * cos(lat) * cos(lon - lon0));
        double heading = point[2] + heading_correction;
        
        // 输出格式: 经度(弧度),纬度(弧度),高度(m),航向角(弧度)
        path_geo.push_back({lon, lat, h, heading});
    }
    
    return path_geo;
}

BEZIER_API std::tuple<double, double, double> inverseVincenty(double lon1, double lat1, double lon2, double lat2)
{
    // Vincenty算法计算椭球体上的距离和方位角
    double L = lon2 - lon1;
    double U1 = atan((1-f) * tan(lat1));
    double U2 = atan((1-f) * tan(lat2));

    double sinU1 = sin(U1), cosU1 = cos(U1);
    double sinU2 = sin(U2), cosU2 = cos(U2);

    double lambda = L;
    double lambdaP = 2*PI;
    double iterLimit = 100;

    double sinLambda, cosLambda, sinSigma, cosSigma, sigma, sinAlpha, cosSqAlpha, cos2SigmaM;

    // 迭代计算直到收敛
    while (abs(lambda-lambdaP) > 1e-12 && --iterLimit > 0) {
        sinLambda = sin(lambda);
        cosLambda = cos(lambda);
        sinSigma = sqrt((cosU2*sinLambda) * (cosU2*sinLambda) + 
                        (cosU1*sinU2-sinU1*cosU2*cosLambda) * (cosU1*sinU2-sinU1*cosU2*cosLambda));
        
        if (sinSigma == 0) return {0.0, 0.0, 0.0};  // 同一点
        
        cosSigma = sinU1*sinU2 + cosU1*cosU2*cosLambda;
        sigma = atan2(sinSigma, cosSigma);
        sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
        cosSqAlpha = 1 - sinAlpha*sinAlpha;
        
        // 防止除零错误
        if (cosSqAlpha != 0) {
            cos2SigmaM = cosSigma - 2*sinU1*sinU2/cosSqAlpha;
        } else {
            cos2SigmaM = 0;
        }
        
        double C = f/16*cosSqAlpha*(4+f*(4-3*cosSqAlpha));
        lambdaP = lambda;
        lambda = L + (1-C) * f * sinAlpha * 
                (sigma + C*sinSigma*(cos2SigmaM+C*cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)));
    }

    if (iterLimit == 0) return {-1.0, 0.0, 0.0};  // 不收敛

    double uSq = cosSqAlpha * (a*a - b*b) / (b*b);
    double A = 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)));
    double B = uSq/1024 * (256+uSq*(-128+uSq*(74-47*uSq)));
    double deltaSigma = B*sinSigma*(cos2SigmaM+B/4*(cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)-
                        B/6*cos2SigmaM*(-3+4*sinSigma*sinSigma)*(-3+4*cos2SigmaM*cos2SigmaM)));

    double distance = b*A*(sigma-deltaSigma);

    // 计算方位角(正向和反向)
    double alpha1 = atan2(cosU2*sinLambda, 
                        cosU1*sinU2 - sinU1*cosU2*cosLambda);
    double alpha2 = atan2(cosU1*sinLambda,
                        -sinU1*cosU2 + cosU1*sinU2*cosLambda);

    // 确保方位角在[0,2π)范围内
    alpha1 = fmod(alpha1 + 2*PI, 2*PI);
    alpha2 = fmod(alpha2 + 2*PI, 2*PI);

	return std::make_tuple(distance, alpha1, alpha2);
}

}  // namespace bezier
