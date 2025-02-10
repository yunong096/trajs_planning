import matplotlib.pyplot as plt

# 读取文件1的坐标点
# def read_coordinates_from_file(file_path):
#     x_coords = []
#     y_coords = []
#     with open(file_path, 'r') as file:
#         lines = file.readlines()
#         for line in lines:
#             x, y = line.strip().split()
#             x_coords.append(float(x))
#             y_coords.append(float(y))
#     return x_coords, y_coords

# # 读取文件1的坐标点
# x1, y1 = read_coordinates_from_file('output2.txt')

# # 读取文件2的坐标点
# x2, y2 = read_coordinates_from_file('output.txt')

# # 读取文件3的坐标点
# x3, y3 = read_coordinates_from_file('inter_points.txt')

# # 使用matplotlib绘制坐标点
# plt.plot(x1, y1, marker='o', label='File 1')  # 绘制文件1的坐标点，并添加标签
# plt.plot(x2, y2, marker='x', label='File 2')  # 绘制文件2的坐标点，使用不同的标记并添加标签
# plt.plot(x3, y3, marker='.', label='File 3', markersize = 0.1)  # 绘制文件3的坐标点，使用不同的标记并添加标签
# plt.title('Coordinate Points from Two Files')  # 设置图表标题
# plt.xlabel('X Coordinate')  # 设置x轴标签
# plt.ylabel('Y Coordinate')  # 设置y轴标签
# plt.legend()  # 显示图例
# plt.grid(True)  # 显示网格
# plt.show()  # 显示图表

# 定义输入和输出文件的路径
input_file_path ='parking1.txt'
output_file_path = 'parking3.txt'
 
# 打开输入文件以读取模式，并读取所有行到列表中
with open(input_file_path, 'r', encoding='utf-8') as infile:
    lines = infile.readlines()
 
# 反转行列表
reversed_lines = lines[::-1]
 
# 打开输出文件以写入模式，并写入反转后的行
with open(output_file_path, 'w', encoding='utf-8') as outfile:
    outfile.writelines(reversed_lines)
 
print(f"已将 {input_file_path} 中的行反向输出到 {output_file_path}")