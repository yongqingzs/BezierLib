import matplotlib.pyplot as plt
import numpy as np
import glob
import os
from matplotlib.gridspec import GridSpec

def plot_bezier_paths(out_dir):
    """
    读取贝塞尔路径TXT文件并生成可视化图表
    
    参数:
        out_dir: 字符串，TXT文件的读取目录和图表的输出目录
    """
    # 确保输出目录存在
    os.makedirs(out_dir, exist_ok=True)
    
    # 查找所有节点路径文件
    path_files = glob.glob(os.path.join(out_dir, "node_*_path.txt"))
    path_files.sort()  # 确保顺序
    
    if not path_files:
        print(f"在 {out_dir} 目录中未找到任何路径文件")
        return
    
    # 读取目标信息 - 使用np.loadtxt
    target_file = os.path.join(out_dir, "target.txt")
    try:
        # 跳过注释行，直接读取数值
        target_data = np.loadtxt(target_file, comments='#')
        target_x = target_data[0]  # 第一个数据
        target_y = target_data[1]  # 第二个数据
        target_radius = target_data[2]  # 第三个数据
        print(f"从 {target_file} 读取目标点: ({target_x}, {target_y}), 半径: {target_radius}")
    except Exception as e:
        print(f"读取目标文件 {target_file} 时出错: {e}")
        return
    
    # 创建图形布局
    # plt.figure(figsize=(15, 10))
    # gs = GridSpec(2, 3, figure=plt.gcf())
    
    # 主图 - 所有路径
    fig, ax_main = plt.subplots()  # 左侧两列
    ax_main.set_title("Bezier Paths for All Nodes", fontsize=14)
    ax_main.set_xlabel("X Coordinate", fontsize=12)
    ax_main.set_ylabel("Y Coordinate", fontsize=12)
    ax_main.grid(True, linestyle='--', alpha=0.7)
    
    # 绘制目标圆
    theta = np.linspace(0, 2*np.pi, 100)
    x_circle = target_x + target_radius * np.cos(theta)
    y_circle = target_y + target_radius * np.sin(theta)
    ax_main.plot(x_circle, y_circle, 'r--', linewidth=2, label='Target Circle')
    ax_main.plot(target_x, target_y, 'ro', markersize=8, label='Target Point')
    
    # 颜色映射
    cmap = plt.cm.tab10
    colors = cmap(np.linspace(0, 1, len(path_files)))
    
    # 保存所有路径数据以便重用
    all_paths = []
    
    # 读取并绘制每个节点的路径
    for i, path_file in enumerate(path_files):
        try:
            # 使用np.loadtxt读取文件，只取前两列
            path_data = np.loadtxt(path_file, comments='#')
            # 提取x和y列
            x_data = path_data[:, 0]
            y_data = path_data[:, 1]
            
            # 创建字典存储数据，与之前的API保持一致
            df = {'x': x_data, 'y': y_data}
            
            # 保存路径数据
            all_paths.append(df)
            
            # 提取节点编号
            node_num = int(os.path.basename(path_file).split('_')[1])
            
            # 在主图中绘制路径
            ax_main.plot(x_data, y_data, '-', color=colors[i], 
                        linewidth=2, label=f'Node {node_num}')
            ax_main.plot(x_data[0], y_data[0], 'o', 
                        color=colors[i], markersize=8)
            
            # 添加起点标记
            # ax_main.annotate(f"Node {node_num} Start", 
            #                 xy=(x_data[0], y_data[0]),
            #                 xytext=(x_data[0]+2000, y_data[0]+2000),
            #                 arrowprops=dict(facecolor=colors[i], shrink=0.05, width=1.5),
            #                 fontsize=10)
            
        except Exception as e:
            print(f"处理 {path_file} 时出错: {e}")
    
    plt.show()
    # # 设置主图的显示范围
    # ax_main.axis('equal')  # 保持坐标轴比例一致
    # ax_main.legend(loc='upper right', fontsize=10)
    
    # # 右侧单独显示每个节点的路径
    # for i, df in enumerate(all_paths):
    #     if i >= 3:  # 最多显示3个子图
    #         break
            
    #     # 创建子图
    #     ax = plt.subplot(gs[i, 2])  # 右侧列
    #     ax.set_title(f"Node {i+1} Path", fontsize=12)
    #     ax.grid(True, linestyle='--', alpha=0.7)
        
    #     # 绘制路径
    #     ax.plot(df['x'], df['y'], '-', color=colors[i], linewidth=2)
    #     ax.plot(df['x'][0], df['y'][0], 'o', color=colors[i], markersize=8)
        
    #     # 绘制目标圆
    #     ax.plot(x_circle, y_circle, 'r--', linewidth=1.5)
    #     ax.plot(target_x, target_y, 'ro', markersize=6)
        
    #     # 设置显示范围
    #     ax.axis('equal')
        
    #     # 关闭坐标轴标签
    #     ax.set_xticks([])
    #     ax.set_yticks([])
    
    # plt.tight_layout()
    # plt.savefig(os.path.join(out_dir, 'bezier_paths.png'), dpi=300)
    # plt.show()
    
    # # 创建路径长度对比图
    # if all_paths:
    #     plt.figure(figsize=(10, 6))
    #     path_lengths = []
    #     node_names = []
        
    #     for i, df in enumerate(all_paths):
    #         # 计算路径长度
    #         x_diffs = np.diff(df['x'])
    #         y_diffs = np.diff(df['y'])
    #         segment_lengths = np.sqrt(x_diffs**2 + y_diffs**2)
    #         total_length = np.sum(segment_lengths)
    #         path_lengths.append(total_length)
    #         node_names.append(f"Node {i+1}")
        
    #     # 绘制柱状图
    #     plt.bar(node_names, path_lengths, color=colors[:len(path_lengths)])
    #     plt.title("Path Length Comparison", fontsize=14)
    #     plt.ylabel("Path Length", fontsize=12)
    #     plt.grid(True, axis='y', linestyle='--', alpha=0.7)
        
    #     # 在柱状图上显示具体数值
    #     for i, v in enumerate(path_lengths):
    #         plt.text(i, v + 0.01*max(path_lengths), f"{v:.1f}", 
    #                 ha='center', fontsize=10)
        
    #     plt.tight_layout()
    #     plt.savefig(os.path.join(out_dir, 'path_lengths.png'), dpi=300)
    #     plt.show()
    
    print(f"可视化完成，结果保存在: {out_dir}")


# 使用示例
if __name__ == "__main__":
    # 指定目录
    out_dir = "../out"
    # 调用函数
    plot_bezier_paths(out_dir)