分析项目工具Solver

# auto_aim::Solver()

#### 初始化：读取配置文件，读入旋转和平移的转换矩阵`g2i`,`c2g`,相机内参和畸变，

#### g: gimbal, i: imu,c: camera

#### set_R_gimbal2world():根据输入的位姿计算云台到世界坐标系的旋转转换矩阵

#### solve():PNP解算获取装甲板在相机坐标系的位置，计算装甲板在世界坐标系的位置，并且根据目标类型(哪种机器人）判断是否进行调用yaw优化

#### reproject_armor():根据装甲板在世界坐标系下的位置和朝向，反推出它在当前相机图像上的投影点。

#### world2pixel(): 世界坐标系转像素坐标系

#### SJTU_cost(): 上交的装甲板轮廓匹配代价函数,衡量检测到的2d点与参考2d的差距

#### armor_reprojection_error(): 通过调用reproject_armor()计算误差

#### optimize_yaw(): 对特定的装甲板进行优化，-70到70度角度枚举误差最小的位置，项目使用的是armor_reprojection_error()计算误差





