import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import sys
import os
import glob

def plot_multi_missile_curves(file_pattern='missile_*_path.txt', output_file='multi_missile_plot.png'):
    """
    Plot multiple bezier curves for missile paths
    
    Parameters:
        file_pattern: Pattern for missile path files (e.g., 'missile_*_path.txt')
        output_file: Output image filename
    """
    # Get script directory
    script_dir = sys.path[0]
    # Get output directory
    output_dir = os.path.join(script_dir, '..', 'output')
    
    # Find all matching curve files
    file_pattern_path = os.path.join(output_dir, file_pattern)
    curve_files = sorted(glob.glob(file_pattern_path))
    
    if not curve_files:
        print(f"No files matching '{file_pattern}' found")
        return
    
    # Create figure
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Color list for different trajectories
    colors = ['b', 'r', 'g', 'c', 'm', 'y', 'k', 'orange', 'purple', 'brown']
    
    target_point = None
    radius = None
    
    # Process all curve files
    for i, curve_file in enumerate(curve_files):
        color = colors[i % len(colors)]
        missile_id = os.path.basename(curve_file).split('_')[1]
        
        # Read curve data
        try:
            data = np.loadtxt(curve_file, skiprows=2)
            if data.size == 0:
                print(f"File {curve_file} is empty or has incorrect format")
                continue
                
            t = data[:, 0]
            x = data[:, 1]
            y = data[:, 2]
            curvature = data[:, 3]
            
            # Plot bezier curve
            ax.plot(x, y, f'{color}-', linewidth=2, label=f'Missile {missile_id}')
            
            # Find control points file (naming convention: curve filename + _control_points suffix)
            cp_file = curve_file.replace('.txt', '_control_points.txt')
            if os.path.exists(cp_file):
                cp_data = np.loadtxt(cp_file, skiprows=2)
                if cp_data.size > 0:
                    cp_x = cp_data[0:4, 0]
                    cp_y = cp_data[0:4, 1]
                    
                    # Plot control points and control polygon
                    ax.plot(cp_x, cp_y, f'{color}o--', linewidth=1, markersize=6, alpha=0.5)
                    
                    # Mark start and end points
                    ax.plot(cp_x[0], cp_y[0], f'{color}o', markersize=8)
                    ax.plot(cp_x[3], cp_y[3], f'{color}*', markersize=10)
                    
                    # Extract target point and radius (from the first file only)
                    if target_point is None and cp_data.shape[0] > 4:
                        target_point = cp_data[4]
                        if cp_data.shape[0] > 5:
                            radius = cp_data[5, 0]
            
            # Mark maximum curvature point
            max_curv_idx = np.argmax(curvature)
            max_curv_x = x[max_curv_idx]
            max_curv_y = y[max_curv_idx]
            max_curv = curvature[max_curv_idx]
            ax.plot(max_curv_x, max_curv_y, f'{color}D', markersize=8)
            ax.annotate(f'κ={max_curv:.3f}', (max_curv_x, max_curv_y), 
                        textcoords="offset points", xytext=(0,10), 
                        ha='center', color=color)
            
        except Exception as e:
            print(f"Error processing file {curve_file}: {e}")
    
    # If target point and radius exist, plot them
    if target_point is not None and target_point.size == 2:
        tx, ty = target_point
        ax.plot(tx, ty, 'k*', markersize=15, label='Target')
        
        if radius is not None:
            circle = Circle((tx, ty), radius, fill=False, linestyle='--', 
                        color='k', alpha=0.7, label='Constraint Circle')
            ax.add_patch(circle)
    
    # Set plot style
    ax.set_xlabel('X Coordinate', fontsize=12)
    ax.set_ylabel('Y Coordinate', fontsize=12)
    ax.set_title('Multi-Missile Bezier Curve Planning', fontsize=14)
    ax.grid(True)
    ax.legend(loc='best')
    ax.axis('equal')  # Keep aspect ratio equal
    
    # Make the plot look nice
    plt.tight_layout()
    
    # Save and display
    plt.savefig(os.path.join(output_dir, output_file), dpi=300)
    plt.show()

# Add a function for curvature analysis
def plot_curvature_analysis(file_pattern='missile_*_path.txt', output_file='curvature_analysis.png'):
    """
    Plot curvature analysis for multiple missile paths
    
    Parameters:
        file_pattern: Pattern for missile path files
        output_file: Output image filename
    """
    # Get script directory
    script_dir = sys.path[0]
    # Get output directory
    output_dir = os.path.join(script_dir, '..', 'output')
    
    # Find all matching curve files
    file_pattern_path = os.path.join(output_dir, file_pattern)
    curve_files = sorted(glob.glob(file_pattern_path))
    
    if not curve_files:
        print(f"No files matching '{file_pattern}' found")
        return
    
    # Create figure
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Color list
    colors = ['b', 'r', 'g', 'c', 'm', 'y', 'k', 'orange', 'purple', 'brown']
    
    # Process all curve files
    for i, curve_file in enumerate(curve_files):
        color = colors[i % len(colors)]
        missile_id = os.path.basename(curve_file).split('_')[1]
        
        # Read curve data
        try:
            data = np.loadtxt(curve_file, skiprows=2)
            if data.size == 0:
                continue
                
            t = data[:, 0]  # Parameter t
            curvature = data[:, 3]  # Curvature
            
            # Plot curvature
            ax.plot(t, curvature, f'{color}-', linewidth=2, label=f'Missile {missile_id}')
            
            # Mark maximum curvature point
            max_curv_idx = np.argmax(curvature)
            max_curv_t = t[max_curv_idx]
            max_curv = curvature[max_curv_idx]
            ax.plot(max_curv_t, max_curv, f'{color}o', markersize=8)
            ax.annotate(f'Max κ: {max_curv:.3f}\nt={max_curv_t:.2f}', 
                        (max_curv_t, max_curv), textcoords="offset points",
                        xytext=(10,0), ha='left', color=color)
            
            # Find corresponding control points file for min turning radius info
            cp_file = curve_file.replace('.txt', '_control_points.txt')
            if os.path.exists(cp_file):
                # Assuming min turning radius info is in the control points file
                # Adjust as needed for your actual file format
                pass
            
        except Exception as e:
            print(f"Error processing file {curve_file}: {e}")
    
    # Set plot style
    ax.set_xlabel('Parameter t', fontsize=12)
    ax.set_ylabel('Curvature', fontsize=12)
    ax.set_title('Multi-Missile Bezier Curve Curvature Analysis', fontsize=14)
    ax.grid(True)
    ax.legend(loc='best')
    
    # Make the plot look nice
    plt.tight_layout()
    
    # Save and display
    plt.savefig(os.path.join(output_dir, output_file), dpi=300)
    plt.show()

# Add a function to create a plot showing arrival times
def plot_arrival_times(file_pattern='missile_*_path.txt', output_file='arrival_times.png'):
    """
    Plot arrival times analysis for multiple missile paths
    
    Parameters:
        file_pattern: Pattern for missile path files
        output_file: Output image filename
    """
    # Get script directory
    script_dir = sys.path[0]
    # Get output directory
    output_dir = os.path.join(script_dir, '..', 'output')
    
    # Find all matching curve files
    file_pattern_path = os.path.join(output_dir, file_pattern)
    curve_files = sorted(glob.glob(file_pattern_path))
    
    if not curve_files:
        print(f"No files matching '{file_pattern}' found")
        return
    
    # Collect arrival time data
    missile_ids = []
    arrival_times = []
    speeds = []
    distances = []
    
    # Process all curve files
    for curve_file in curve_files:
        missile_id = os.path.basename(curve_file).split('_')[1]
        missile_ids.append(int(missile_id))
        
        # Read curve data to get length
        try:
            data = np.loadtxt(curve_file, skiprows=2)
            if data.size == 0:
                continue
            
            # Get info from header or control points file if available
            cp_file = curve_file.replace('.txt', '_control_points.txt')
            speed = 1.0  # Default value
            
            if os.path.exists(cp_file):
                # Try to read speed from file header
                with open(cp_file, 'r') as f:
                    lines = f.readlines()
                    for line in lines[:2]:  # Check first two lines
                        if 'speed' in line.lower():
                            try:
                                speed = float(line.split('=')[1].strip())
                            except:
                                pass
            
            # Calculate path length (approximate with sum of segments)
            x = data[:, 1]
            y = data[:, 2]
            dx = np.diff(x)
            dy = np.diff(y)
            path_length = np.sum(np.sqrt(dx**2 + dy**2))
            
            # Calculate arrival time
            arrival_time = path_length / speed
            
            arrival_times.append(arrival_time)
            speeds.append(speed)
            distances.append(path_length)
            
        except Exception as e:
            print(f"Error processing file {curve_file}: {e}")
            arrival_times.append(0)
            speeds.append(0)
            distances.append(0)
    
    # Sort by missile ID
    sorted_indices = np.argsort(missile_ids)
    missile_ids = [missile_ids[i] for i in sorted_indices]
    arrival_times = [arrival_times[i] for i in sorted_indices]
    speeds = [speeds[i] for i in sorted_indices]
    distances = [distances[i] for i in sorted_indices]
    
    # Create figure with 2 subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12))
    
    # Bar colors
    bar_colors = ['royalblue', 'forestgreen', 'darkorange', 'firebrick']
    
    # Plot arrival times
    bars = ax1.bar(missile_ids, arrival_times, color=bar_colors[:len(missile_ids)])
    ax1.set_xlabel('Missile ID', fontsize=12)
    ax1.set_ylabel('Arrival Time', fontsize=12)
    ax1.set_title('Time to Target Comparison', fontsize=14)
    ax1.grid(axis='y', linestyle='--', alpha=0.7)
    
    # Add value labels on bars
    for bar, time in zip(bars, arrival_times):
        ax1.text(bar.get_x() + bar.get_width()/2., time + 0.02*max(arrival_times),
                f'{time:.2f}', ha='center', fontsize=10)
    
    # Calculate statistics
    mean_time = np.mean(arrival_times)
    std_time = np.std(arrival_times)
    max_diff = max(arrival_times) - min(arrival_times)
    
    # Add statistics as text
    stats_text = f'Mean: {mean_time:.2f}\nStd Dev: {std_time:.2f}\nMax Diff: {max_diff:.2f}'
    ax1.text(0.02, 0.95, stats_text, transform=ax1.transAxes,
            bbox={'facecolor': 'white', 'alpha': 0.8, 'pad': 5},
            verticalalignment='top')
    
    # Plot the path lengths and speeds (secondary plot)
    ax2.set_xlabel('Missile ID', fontsize=12)
    ax2.grid(axis='y', linestyle='--', alpha=0.7)
    
    # Create twin axis for speeds
    ax2_twin = ax2.twinx()
    
    # Plot distances as bars
    bars1 = ax2.bar(np.array(missile_ids) - 0.2, distances, width=0.4, 
                   color='steelblue', label='Path Length')
    ax2.set_ylabel('Path Length', color='steelblue', fontsize=12)
    
    # Plot speeds as bars on twin axis
    bars2 = ax2_twin.bar(np.array(missile_ids) + 0.2, speeds, width=0.4, 
                        color='indianred', label='Speed')
    ax2_twin.set_ylabel('Speed', color='indianred', fontsize=12)
    
    # Add legend
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2_twin.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
    
    ax2.set_title('Path Lengths and Speeds', fontsize=14)
    
    # Make the plot look nice
    plt.tight_layout()
    
    # Save and display
    plt.savefig(os.path.join(output_dir, output_file), dpi=300)
    plt.show()

if __name__ == "__main__":
    # Plot multi-missile curves
    plot_multi_missile_curves()
    
    # Plot curvature analysis
    # plot_curvature_analysis()
    
    # Plot arrival time analysis
    # plot_arrival_times()