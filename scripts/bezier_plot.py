import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import sys

def plot_bezier_curve(curve_file, control_points_file=None):
    """Plot Bezier curve and its control points"""
    curve_file = sys.path[0] + '/../output/' + curve_file  # for debug
    # Load curve data
    data = np.loadtxt(curve_file, skiprows=2)
    t = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    curvature = data[:, 3]
    
    # Create figure
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
    
    # Plot the curve
    ax1.plot(x, y, 'b-', linewidth=2, label='Bezier Curve')
    
    target_point = None
    radius = None
    # If control points file exists, load and plot control points
    if control_points_file:
        control_points_file = sys.path[0] + '/../output/' + control_points_file
        cp_data = np.loadtxt(control_points_file, skiprows=2)
        cp_x = cp_data[0:4, 0]
        cp_y = cp_data[0:4, 1]
        target_point = cp_data[4]
        radius = cp_data[5, 0]
        # Plot control points
        ax1.plot(cp_x, cp_y, 'ro--', linewidth=1, markersize=8, label='Control Points')
        
        # Label control points
        for i, (px, py) in enumerate(zip(cp_x, cp_y)):
            ax1.annotate(f'P{i}', (px, py), textcoords="offset points", 
                        xytext=(0,10), ha='center')
    
    # If target point and radius are provided, plot target point and constraint circle
    if target_point.any and radius:
        # Plot target point
        tx, ty = target_point
        ax1.plot(tx, ty, 'g*', markersize=15, label='Target Point')
        
        # Plot constraint circle
        circle = Circle((tx, ty), radius, fill=False, linestyle='--', 
                       color='g', alpha=0.5, label='Constraint Circle')
        ax1.add_patch(circle)
    
    # Set axis labels and legend
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_title('Bezier Curve and Control Points')
    ax1.grid(True)
    ax1.legend()
    ax1.axis('equal')  # Maintain aspect ratio
    
    # Plot curvature graph
    ax2.plot(t, curvature, 'r-', linewidth=2)
    ax2.set_xlabel('Parameter t')
    ax2.set_ylabel('Curvature')
    ax2.set_title('Curvature vs Parameter')
    ax2.grid(True)
    
    # Mark maximum curvature point
    max_curv_idx = np.argmax(curvature)
    max_curv_t = t[max_curv_idx]
    max_curv = curvature[max_curv_idx]
    ax2.plot(max_curv_t, max_curv, 'bo', markersize=8)
    ax2.annotate(f'Max Curvature: {max_curv:.4f}\nt={max_curv_t:.2f}', 
                (max_curv_t, max_curv), textcoords="offset points",
                xytext=(10,0), ha='left')
    
    # Also mark the maximum curvature point in the first plot
    max_curv_x = x[max_curv_idx]
    max_curv_y = y[max_curv_idx]
    ax1.plot(max_curv_x, max_curv_y, 'go', markersize=8, label='Max Curvature Point')
    
    plt.tight_layout()
    plt.savefig('../output/bezier_curve_plot.png', dpi=300)
    plt.show()

def plot_quintic_curve(curve_file, control_points_file=None):
    """Plot Quintic (5th order) Bezier curve and its control points"""
    # 构建文件路径
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    curve_file_path = os.path.join(base_dir, 'output', curve_file)
    
    # 加载曲线数据
    data = np.loadtxt(curve_file_path, skiprows=2)
    t = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    curvature = data[:, 3]
    
    # 创建单个图形
    fig, ax1 = plt.subplots(figsize=(10, 8))
    
    # 绘制贝塞尔曲线
    ax1.plot(x, y, 'b-', linewidth=2, label='Quintic Bezier Curve')
    
    target_point = None
    radius = None
    
    # 如果提供了控制点文件，加载并绘制控制点
    if control_points_file:
        cp_file_path = os.path.join(base_dir, 'output', control_points_file)
        cp_data = np.loadtxt(cp_file_path, skiprows=2)
        
        # 五阶贝塞尔曲线有6个控制点
        cp_x = cp_data[0:6, 0]
        cp_y = cp_data[0:6, 1]
        
        # 目标点和半径
        if cp_data.shape[0] > 6:
            target_point = cp_data[6]
            radius = cp_data[7, 0]
        
        # 绘制控制点和控制多边形
        ax1.plot(cp_x, cp_y, 'ro--', linewidth=1, markersize=8, label='Control Points')
        
        # 标记控制点
        for i, (px, py) in enumerate(zip(cp_x, cp_y)):
            ax1.annotate(f'P{i}', (px, py), textcoords="offset points", 
                        xytext=(0,10), ha='center')
        
        # 绘制起点和终点的方向向量
        dx0 = 5 * (cp_x[1] - cp_x[0])
        dy0 = 5 * (cp_y[1] - cp_y[0])
        ax1.arrow(cp_x[0], cp_y[0], dx0/5, dy0/5, head_width=1, head_length=1, fc='g', ec='g')
        
        dx5 = 5 * (cp_x[5] - cp_x[4])
        dy5 = 5 * (cp_y[5] - cp_y[4])
        ax1.arrow(cp_x[5], cp_y[5], -dx5/5, -dy5/5, head_width=1, head_length=1, fc='r', ec='r')
    
    # 如果提供了目标点和半径，绘制目标点和约束圆
    if target_point is not None and radius is not None:
        # 绘制目标点
        tx, ty = target_point
        ax1.plot(tx, ty, 'g*', markersize=15, label='Target Point')
        
        # 绘制约束圆
        circle = Circle((tx, ty), radius, fill=False, linestyle='--', 
                       color='g', alpha=0.5, label='Constraint Circle')
        ax1.add_patch(circle)
    
    # 标记最大曲率点
    max_curv_idx = np.argmax(curvature)
    max_curv_x = x[max_curv_idx]
    max_curv_y = y[max_curv_idx]
    ax1.plot(max_curv_x, max_curv_y, 'go', markersize=8, label='Max Curvature Point')
    
    # 设置坐标轴标签和图例
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_title('Quintic Bezier Curve and Control Points')
    ax1.grid(True)
    ax1.legend()
    ax1.axis('equal')  # 保持纵横比
    
    # 保存图像
    output_file = os.path.join(base_dir, 'output', f"{os.path.splitext(curve_file)[0]}_plot.png")
    plt.savefig(output_file, dpi=300)
    print(f"图像已保存到: {output_file}")
    plt.show()

if __name__ == "__main__":
    # Set target point and constraint radius (same values as used in C++ code)
    
    d = 5

    if (d == 3):
        # Call plotting function
        plot_bezier_curve(
            curve_file='bezier_curve_c0.txt',
            control_points_file='bezier_curve_c0_control_points.txt',
        )

        plot_bezier_curve(
            curve_file='bezier_curve_c1.txt',
            control_points_file='bezier_curve_c1_control_points.txt',
        )

        plot_bezier_curve(
            curve_file='bezier_curve_c2.txt',
            control_points_file='bezier_curve_c2_control_points.txt',
        )
    elif (d == 5):
        # Call plotting function
        plot_quintic_curve(
            curve_file='quintic_curve.txt',
            control_points_file='quintic_curve_control_points.txt',
        )
