# base
from geographiclib.geodesic import Geodesic
import pymap3d


def test_inverse():
    # WGS84 椭球模型
    geod = Geodesic.WGS84

    # 两个点的经纬度（单位：度）
    lon1, lat1 = 14.8504, 35.5802
    lon2, lat2 = 15.0038, 35.7029

    # 计算测地线距离（Vincenty 公式）
    g = geod.Inverse(lat1, lon1, lat2, lon2)

    # 输出结果
    distance = g['s12']  # 距离（米）
    azimuth1 = g['azi1']  # 起点到终点的方位角（度）
    azimuth2 = g['azi2']  # 终点到起点的方位角（度）

    print(f"距离: {distance / 1000:.3f} 公里")
    print(f"起点方位角: {azimuth1:.2f} 度")
    print(f"终点方位角: {azimuth2:.2f} 度")


def test_geodetic2enu():
    # # 参考点：上海 (31.2304°N, 121.4737°E, 0m)
    # ref_lat, ref_lon, ref_h = 31.2304, 121.4737, 0.0
    # # 目标点：附近点 (31.2350°N, 121.4800°E, 10m)
    # lat, lon, h = 31.2350, 121.4800, 10.0
    
    # ref_lat, ref_lon, ref_h = 35.5802, 14.8504, 0
    ref_lat, ref_lon, ref_h = 35.5841, 14.8541, 0
    lat, lon, h = 35.7029, 15.0038, 0
    
    # 转换为 ENU 坐标
    E, N, U = pymap3d.geodetic2enu(ref_lat, ref_lon, ref_h, lat, lon, h)
    print("ENU 坐标:")
    print(f"E: {E:.2f} m")
    print(f"N: {N:.2f} m")
    print(f"U: {U:.2f} m")


if __name__ == "__main__":
    # test_inverse()
    test_geodetic2enu()