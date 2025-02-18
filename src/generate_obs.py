import csv

# 初始位置和速度
initial_x = 4.17
initial_y = 64.95 + 2.35 - 20
velocity_x = 0.0
velocity_y = 1.0

# 时间步长和预测长度
time_step = 0.1
prediction_length = 6.0
num_points = int(prediction_length / time_step) + 1

# 生成标题行
header = ["time"]
for i in range(1, num_points + 1):
    header.append(f"x{i}")
    header.append(f"y{i}")

# 生成数据
data = []
for t in range(0, 401):  # 30秒，每0.1秒一个时间步长，共301个时间步长
    time = t * time_step
    row = [time]
    for i in range(num_points):
        x = initial_x + (time + i * time_step) * velocity_x
        y = initial_y + (time + i * time_step) * velocity_y
        row.append(x)
        row.append(y)
    data.append(row)

# 写入CSV文件
with open('opposite1.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(header)
    writer.writerows(data)

print("CSV文件已生成")

# import csv

# # 读取 waypoints_data.txt 文件
# with open('/home/lynnn/test_ws/df_parking.txt', 'r') as file:
#     lines = file.readlines()

# # 解析数据
# waypoints = []
# for line in lines:
#     if line.strip():  # 跳过空行
#         x, y = map(float, line.split())
#         waypoints.append((x, y))
# # waypoints.reverse()

# # 生成 CSV 文件
# output_file = 'parking1.csv'
# time_step = 0.1
# prediction_length = 6.0
# num_points = int(prediction_length / time_step) + 1

# # 生成标题行
# header = ["time"]
# for i in range(1, num_points + 1):
#     header.append(f"x{i}")
#     header.append(f"y{i}")

# # 生成数据
# data = []
# for t in range(len(waypoints)):
#     time = t * time_step
#     row = [time]
#     for i in range(num_points):
#         if  t + i < len(waypoints):
#             x, y = waypoints[t + i]
#         else:
#             x, y = waypoints[len(waypoints) - 1]
#         row.append(x - 2.6 * 3 + 53.13666666666667 - 100.72)
#         row.append(y + 9.99 - 46.82 )
#     data.append(row)

# # 写入CSV文件
# with open(output_file, 'w', newline='') as csvfile:
#     writer = csv.writer(csvfile)
#     writer.writerow(header)
#     writer.writerows(data)

# print(f"CSV文件已生成: {output_file}")