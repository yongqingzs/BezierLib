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

if __name__ == "__main__":
    # Set target point and constraint radius (same values as used in C++ code)
    
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
