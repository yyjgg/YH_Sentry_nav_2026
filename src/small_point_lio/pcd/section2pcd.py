import numpy as np
import struct
import os

def bin_to_pcd(bin_file_path, pcd_file_path):
    """
    将KITTI格式的.bin文件转换为PCD文件
    """
    # 读取bin文件
    points = []
    with open(bin_file_path, 'rb') as f:
        while True:
            # 每个点通常有4个float（x, y, z, intensity）
            data = f.read(4 * 4)  # 4个float，每个4字节
            if not data:
                break
            if len(data) < 16:
                break
                
            # 解析数据
            x, y, z, intensity = struct.unpack('ffff', data)
            points.append([x, y, z, intensity])
    
    points = np.array(points)
    print(f"读取到 {len(points)} 个点")
    
    # 写入PCD文件
    with open(pcd_file_path, 'w') as f:
        # 写入PCD头信息
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        
        # 写入点数据
        for point in points:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {point[3]:.6f}\n")
    
    print(f"PCD文件已保存到: {pcd_file_path}")

# 使用方法
if __name__ == "__main__":
    bin_file = "scan.bin"
    pcd_file = "scan.pcd"
    
    if os.path.exists(bin_file):
        bin_to_pcd(bin_file, pcd_file)
    else:
        print(f"错误: 文件 {bin_file} 不存在")