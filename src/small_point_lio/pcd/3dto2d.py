import numpy as np
import cv2
import os
import struct
from collections import defaultdict
import time

def read_pcd_file(file_path):
    """读取PCD文件"""
    points = []
    
    with open(file_path, 'rb') as f:
        lines = []
        header_end = False
        while not header_end:
            line = f.readline().decode('utf-8', errors='ignore').strip()
            lines.append(line)
            if line.startswith('DATA'):
                header_end = True
        
        data_format = 'ascii'
        points_num = -1
        
        for line in lines:
            if line.startswith('DATA'):
                data_format = line.split()[1].lower()
            elif line.startswith('POINTS'):
                points_num = int(line.split()[1])
        
        print(f"PCD格式: {data_format}, 点数: {points_num}")
        
        if data_format == 'ascii':
            for i in range(points_num):
                line = f.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                coords = line.split()
                if len(coords) >= 3:
                    try:
                        x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
                        points.append([x, y, z])
                    except ValueError:
                        continue
        elif data_format == 'binary' or data_format == 'binary_compressed':
            if points_num <= 0:
                return np.array(points)
            
            binary_data = f.read()
            point_size = 4 * 3
            expected_size = points_num * point_size
            
            if len(binary_data) >= expected_size:
                for i in range(points_num):
                    start = i * point_size
                    end = start + point_size
                    if end <= len(binary_data):
                        point_data = binary_data[start:end]
                        try:
                            x = struct.unpack('f', point_data[0:4])[0]
                            y = struct.unpack('f', point_data[4:8])[0]
                            z = struct.unpack('f', point_data[8:12])[0]
                            points.append([x, y, z])
                        except:
                            continue
    
    return np.array(points)

def optimized_vectorized_mapping(points, img_width, img_height, min_coords):
    """优化的向量化映射方法"""
    # 平移所有点
    translated_points = points - min_coords
    
    # 计算图像坐标
    x_coords = (translated_points[:, 0] * 10).astype(int)
    y_coords = (translated_points[:, 1] * 10).astype(int)
    z_values = (translated_points[:, 2] * 100).astype(int)
    
    # 限制z值范围
    z_values = np.clip(z_values, 0, 255)
    
    # 创建图像
    final_image = np.zeros((img_height, img_width), dtype=np.uint8)
    
    # 过滤有效坐标
    valid_mask = (x_coords >= 0) & (x_coords < img_width) & (y_coords >= 0) & (y_coords < img_height)
    x_valid = x_coords[valid_mask]
    y_valid = y_coords[valid_mask]
    z_valid = z_values[valid_mask]
    
    print(f"有效点数量: {len(x_valid)}")
    
    # 使用字典收集每个像素的所有z值
    pixel_dict = defaultdict(list)
    for x, y, z in zip(x_valid, y_valid, z_valid):
        pixel_dict[(y, x)].append(z)
    
    # 对每个像素取最大值
    for (y, x), z_list in pixel_dict.items():
        final_image[y, x] = max(z_list)
    
    return final_image

def process_point_cloud(input_path, output_image_path):
    """使用纯向量化方法的完整点云处理方案"""
    start_time = time.time()
    
    # 1. 读取PCD文件
    print("正在读取PCD文件...")
    points = read_pcd_file(input_path)
    print(f"成功读取 {len(points)} 个点")
    
    if len(points) == 0:
        print("错误: 没有读取到点云数据")
        return
    
    read_time = time.time()
    print(f"文件读取时间: {read_time - start_time:.2f}秒")
    
    # 2. 删除高度不在0.1m到2.55m之间的点
    mask = (points[:, 2] >= 0.1) & (points[:, 2] <= 2.55)
    filtered_points = points[mask]
    print(f"过滤后剩余 {len(filtered_points)} 个点 (保留0.1m-2.55m的点)")
    
    # 3. 降采样 - 使用字典方法
    print("正在进行降采样...")
    voxel_size = 0.1
    voxel_dict = {}
    for point in filtered_points:
        voxel_key = tuple((point // voxel_size).astype(int))
        if voxel_key not in voxel_dict:
            voxel_dict[voxel_key] = []
        voxel_dict[voxel_key].append(point)
    
    downsampled_points = []
    for voxel_points in voxel_dict.values():
        avg_point = np.mean(voxel_points, axis=0)
        downsampled_points.append(avg_point)
    
    downsampled_points = np.array(downsampled_points)
    print(f"降采样后剩余 {len(downsampled_points)} 个点")
    
    downsample_time = time.time()
    print(f"降采样时间: {downsample_time - read_time:.2f}秒")
    
    # 4. 计算图像参数
    min_coords = np.min(downsampled_points, axis=0)
    translated_points = downsampled_points - min_coords
    max_coords = np.max(translated_points, axis=0)
    img_width = int(np.ceil(max_coords[0] * 10)) + 1
    img_height = int(np.ceil(max_coords[1] * 10)) + 1
    
    print(f"图像尺寸: {img_width} x {img_height}")
    print(f"点云范围: X[{min_coords[0]:.2f}, {max_coords[0]:.2f}], "
          f"Y[{min_coords[1]:.2f}, {max_coords[1]:.2f}], "
          f"Z[{min_coords[2]:.2f}, {max_coords[2]:.2f}]")
    
    # 5. 点云映射到图像
    print("正在映射点到图像...")
    final_image = optimized_vectorized_mapping(downsampled_points, img_width, img_height, min_coords)
    
    mapping_time = time.time()
    print(f"点云映射时间: {mapping_time - downsample_time:.2f}秒")
    
    # 6. 显示并保存结果
    if np.max(final_image) > 0:
        # 调整显示大小
        display_img = final_image.copy()
        if img_width > 1000 or img_height > 1000:
            scale = min(800/img_width, 800/img_height)
            new_width = int(img_width * scale)
            new_height = int(img_height * scale)
            display_img = cv2.resize(final_image, (new_width, new_height), interpolation=cv2.INTER_NEAREST)
        
        # 创建彩色图像
        colored_img = cv2.applyColorMap(display_img, cv2.COLORMAP_JET)
        
        # 显示图像
        cv2.imshow("Point Cloud Image", display_img)
        cv2.imshow("Point Cloud Image (彩色)", colored_img)
        print("按任意键关闭图像窗口...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        # 保存图像
        cv2.imwrite(output_image_path, final_image)
        print(f"灰度图像已保存到: {output_image_path}")
        
        # 保存彩色版本
        color_output_path = output_image_path.replace('.png', '_color.png')
        full_color_img = cv2.applyColorMap(final_image, cv2.COLORMAP_JET)
        cv2.imwrite(color_output_path, full_color_img)
        print(f"彩色图像已保存到: {color_output_path}")
        
        # 统计信息
        non_zero_pixels = np.count_nonzero(final_image)
        print(f"最大灰度值: {np.max(final_image)}")
        print(f"非零像素数量: {non_zero_pixels}")
        print(f"图像覆盖率: {non_zero_pixels/(img_width * img_height)*100:.2f}%")
        print(f"总处理时间: {time.time() - start_time:.2f}秒")
        
        # 显示处理速度
        total_points_processed = len(filtered_points)
        processing_time = time.time() - read_time
        points_per_second = total_points_processed / processing_time if processing_time > 0 else 0
        print(f"处理速度: {points_per_second:.0f} 点/秒")
    else:
        print("错误: 生成的图像为空")

if __name__ == "__main__":
    # 输入文件路径
    input_pcd_path = "scan.pcd"
    
    if not os.path.exists(input_pcd_path):
        print(f"错误: 文件 {input_pcd_path} 不存在")
        exit(1)
    
    # 输出图像路径
    output_image_path = "point_cloud_image.png"
    
    print("=== 纯向量化点云处理 ===")
    print("开始处理...")
    
    process_point_cloud(input_pcd_path, output_image_path)