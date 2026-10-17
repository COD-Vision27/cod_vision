一些需要了解的项目自制的yolo类。

1. auto_aim::YOLOBase

   yolo基类，只包含两个虚函数，要求继承此类的必须实现.

   1. detect: 需要图像帧和帧序号，返回装甲板结构体信息列表(list)，装甲板结构体在armor.cpp
   2. postprocess:同样需要图像帧和帧序号，还需要`Mat &output`，`double scale`.返回装甲板结构体信息列表(list).

   具体看子类实现。

2. auto_aim::YOLO

   是不同YOLO类的封装，统一封装为YOLO类，调用detect和postprocess时实际上调用内部具体的yolobase指针所指向的各种yolo类

   初始化：配置文件路径，是否启用debug(默认启用)

   有一个独占指针，类型为YOLOBase,会根据初始化过的配置文件，决定加载哪个yolo类,存在`YOLO5`,`YOLO8`,`YOLO11`这些基类派生类,调用YOLO类

猜测不同yolo类应该实现都差不多，所以这里挑选YOLO11类来看

3. YOLO11
   
   初始化读取配置文件，最低置信度，设备等
   
   保存图像路径在当前目录的imgs,读取模型结构,设置如下:
   输入1批次640\*640的三通道图像BGR,预处理转浮点数据，归一化，改为RGB图像
   
   detect:
   
   ​	检查是否使用roi,对输入的图像进行处理，裁剪。输入到模型，推理，将输出结果,缩放因子,帧序号,以及原图像传入parse函数(输出结果包含检查框数量，总共多少个类别)
   
   parse:
   
   ​	对输出结果进行划分，最终返回装甲板列表
   
   ​	包括
   
   1. 提取探测框xywh，IOU等减少重框等
   2. 各个类别的得分，并提取出最高分类别，低于置信度筛选
   3. 关键点提取
   
   ```
   模型输出 {1, 84, 8400}
          ↓ transpose
   output {8400, 84}
          ↓ 遍历每一行
          ├─ 切出 xywh / scores / keypoints
          ├─ 取最大类别分数
          ├─ 阈值过滤
          ├─ xywh → 原图 Rect
          ├─ keypoints → 原图坐标
          └─ 收集到 4 个 vector
          ↓ NMSBoxes
   indices（保留的框下标）
          ↓ 遍历 indices
          ├─ sort_keypoints 排序
          └─ 构造 Armor（带/不带 offset_）
          ↓ 过滤
          ├─ check_name
          ├─ check_type
          └─ 计算 center_norm
          ↓ debug 绘图（可选）
          ↓ return
   std::list<Armor>
   ```
   
   postprocess: 实际上只是调用parse
   
   

​	
