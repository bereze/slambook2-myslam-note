# 基本数据结构
包括帧frame、特征feature、地图点mappoint三个类，对于基本数据结构，将其设为struct即可。考虑到可能被多个线程访问，需要加上线程锁。
## 帧 Frame
Frame含有id、位姿、左右图像及左右图像的特征点等信息。
Frame可以由静态汉书构建，在函数中自动分配id。
## 特征 Feature
Feature主要包括自身的2D位置（用`cv::KeyPoint`表示)，此外，还包括异常点标志位`is_outlier_`、是否是在左侧相机提取的标志位`is_on_left_image_`、以及关联的关键帧和地图点指针。
## 地图点 MapPoint
地图点类，根据特征点三角化得到，主要包括3D位置（用Vec3表示）、异常点标志位`is_outlier_`，此外还记录了它被哪些Feature观测到`observation_`及观测到的次数`observed_times_`。
## 地图类 Map
地图类，实际持有Frame和MapPoint对象，以散列表形式记录了所有的关键帧和对应的地图点，
同时维护一个被激活的关键帧和地图点的窗口（类似局部地图）。

前端调用`InsertKeyFrame`和`InsertMapPoint`插入新帧和地图点，后端维护地图的结构，判定outlier/剔除等。

# 前端
前端的主要功能是根据双目图像确定该帧的位姿。

处理逻辑为：

1. 前端本身有**初始化**、**正常跟踪**、**跟踪丢失**三种状态。
2. 在初始化状态中，根据左右目之间的光流匹配，寻找可以三角化的地图点，成功时建立初始地图。
3. 跟踪阶段，前端计算上一帧的特征点到当前帧的光流，根据光流结果计算位姿。该计算只使用左目图像。
4. 如果跟踪到的点较少，就判定当前帧为关键帧，对于关键帧，做以下几件事：
    - 提取新的特征点；
    - 找到这些点在右图的对应特征点，并用三角化对新的特征点建立新的地图点；
    - 将新的关键帧和地图点加入地图，并触发一次后端优化。
5. 如果跟踪丢失，就重置前端系统，重新初始化。

# 后端
后端在启动后，会等待map_update_的条件变量。当地图更新被触发时，从地图中拿出激活的关键帧的地图点，进行BA优化。

# 其他部件
除了核心算法，还需要一些其他部件让系统更加完整。
## 相机类Camera
管理相机内外参和投影函数（世界坐标、相机坐标、像素坐标之间的变换）。
## 配置类Config
从配置文件中读取配置参数。
## 数据集类Dataset
构造时传入配置文件路径，配置文件中的dataset_dir为数据集路径，按照数据集的存储格式读取图像数据。
## 可视化类Viewer
在另外一个线程中运行，能观察系统的运行状态，及SLAM地图的可视化。


# C++知识

## cmake的debug和release版本
在写cmakelists.txt时，若要生成debug版本，则添加如下语句：

```cmake
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11" -O0 -ggdb)
```

其中，-O0表示编译优化级别，0为不做优化，3为最高级别，-ggdb表示用gdb调试器。

若要生成release版本，则改为添加：

```cmake
set(CMAKE_BUILD_TYPE Realse)
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11" -O3)
```

## 工厂模式生成frame和mappoint

在本项目中，作者采用工厂模式设计Frame和Mappoint类，其实例化使用工厂构建模式，由静态函数构建，并在静态函数中自动分配id。

```c++
// frame.h
// 工厂构建模式，分配id
static std::shared_ptr<Frame> CreateFrame();

// frame.cpp
Frame::Ptr Frame::CreateFrame() {
    static long factory_id = 0;
    Frame::Ptr new_frame(new Frame);
    new_frame->id_ = factory_id++;
    return new_frame;
}
```

