import csv

# 初始位置和速度
initial_x = 52.35
initial_y = 65.95
velocity_x = -1.0
velocity_y = 0.0

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
for t in range(0, 301):  # 30秒，每0.1秒一个时间步长，共301个时间步长
    time = t * time_step
    row = [time]
    for i in range(num_points):
        x = initial_x + (time + i * time_step) * velocity_x
        y = initial_y + (time + i * time_step) * velocity_y
        row.append(x)
        row.append(y)
    data.append(row)

# 写入CSV文件
with open('opposite0.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(header)
    writer.writerows(data)

print("CSV文件已生成")