# 标定流程

### opencv内置了一套完整的标定库，所以只需要调用即可，此处说明各种函数的作用。

#### 首先是摘自同济自瞄的标定代码，他们采用的是圆形标定板。下面将会对他们的标定代码中涉及到的一些函数进行解析。

#### 寻找圆形标定板角点函数

```cpp
bool cv::findCirclesGrid(
    InputArray image,
    Size patternSize,
    OutputArray centers,
    int flags,
    const Ptr<FeatureDetector>& blobDetector,
    const CirclesGridFinderParameters& parameters
);
```

​	InputArray image: 输入的原图像
​	Size patternSize: 角点大小，行乘列
​	OutputArray centers: 输出的角点坐标，二维点坐标
​	int flags: 标志,见下

| 标志                       | 含义                                                         |
| -------------------------- | ------------------------------------------------------------ |
| `CALIB_CB_SYMMETRIC_GRID`  | 对称圆网格（默认）。圆点按矩形排列。                         |
| `CALIB_CB_ASYMMETRIC_GRID` | 非对称圆网格。圆点交错排列。                                 |
| `CALIB_CB_CLUSTERING`      | 使用聚类算法，对透视变形较大的图像更鲁棒，但速度较慢。可与上面两个标志组合，如 `CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING`。 |

​	最后为原点检测器，一般不需要填写。

#### 相对应的，也存在棋盘格的标定函数`findChessboardCorners`。
