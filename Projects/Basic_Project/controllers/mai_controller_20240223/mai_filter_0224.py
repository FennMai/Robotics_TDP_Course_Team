class Sensor_data_filter:
    def __init__(self, n):
        """
        args:
        - n: represents the number of values to consider for averaging
        """
        self.sensor_data = []
        self.n = n
    def update_sensor_data(self, new_value):
        self.sensor_data.append(new_value)  # 将新值添加到列表中
        if len(self.sensor_data) > self.n:  # 如果列表长度超过n，则删除最旧的数据
            del self.sensor_data[0]

    def calculate_average(self):
        if len(self.sensor_data) == 0:
            return 0  # 如果列表为空，则返回0
        return sum(self.sensor_data) / len(self.sensor_data)  # 计算平均值
    
def IMU_filter(new_value):
    if new_value is None:
        return 0
    
    filtered_data = None  # 初始化变量
    
    if  5 < new_value <= 45:
        filtered_data = new_value + 20
    elif  45 < new_value <= 90:
        filtered_data = new_value + 15 
    elif  90 < new_value <= 135:
        filtered_data = new_value + 7 
    elif  135 < new_value <= 180:
        filtered_data = new_value + 5 
    elif -180 < new_value <= -135:
        filtered_data = new_value - 15
    elif -135 < new_value <= -90:
        filtered_data = new_value - 20
    elif -90 < new_value <= -45:
        filtered_data = new_value - 10
    elif -45 < new_value <= -5:
        filtered_data = new_value - 15
    else:
        filtered_data = new_value


    if filtered_data > 180:
        filtered_data = 180
    elif filtered_data < -180:
        filtered_data = -180
    
    return filtered_data




# class save_sensor_data:
#     def __init__(self):

        
# # 创建传感器对象
# fil = Sensor_data_filter()

# # 模拟传感器输入
# sensor_input = [10, 15, 20, 25, 30]

# for value in sensor_input:
#     fil.update_sensor_data(value)  # 更新传感器数据
#     average = fil.calculate_average()  # 计算平均值
#     print("当前输入:", value, "，平均值:", average)
