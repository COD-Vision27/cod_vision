一些需要了解的项目自制的io和工具类。

1. `io::SocketCAN`

   作用：串口通信。

   接收串口名字，回调函数，设置了守护线程，如果程序没结束但是读取线程结束了，会尝试重新打开读取线程

    	1. 打开指定串口，设置回调函数，初始化文件描述符(-1)
    	2. epoll等待，读到数据后用回调函数(此函数在cboard.cpp)解析
    	3. 关闭套接字

2. `tools::ThreadSafeQueue`

   其实就是单独开一个线程的，处理好锁的queue队列，队列满时自动弹出(也可以替换为别的回调函数)，push,pop等都进行了锁处理。

3. `io::Command`

   想要发出去的命令.只是一个结构体，内容如下
   ```cpp
   struct Command
   {
     bool control;
     bool shoot;
     double yaw;
     double pitch;
     double horizon_distance = 0;  //无人机专有
   };
   ```


4. `tools::eulers`


​	四元数转欧拉角,pitch,roll,yaw

5. `tools::xyz2ypd()`

   笛卡尔坐标系转球面坐标系

6. `tools::ypd2xyz_jacobian()`

   输出ypd(球面坐标系)转笛卡尔坐标系的雅可比矩阵

7. `tools::rotation_matrix()`

   把 yaw-pitch-roll（YPR）欧拉角 转成 3×3 旋转矩阵