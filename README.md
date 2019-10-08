### 手写BA实现3D-2D的PnP匹配

PnP（ Perspective-n-Point ）是求解3D到2D点对运动的方法。PnP可以在很少的匹配点中获得较好的运动估计，是最重要的一种姿态估计方法，其本质是一个最小二乘问题，在SLAM系统中，一般会使用像 g2o，Ceres等优化库进行BA问题的求解，手写BA的意义在于熟悉整个PnP的推导过程和其中的非线性优化的编程实现。

问题描述：

在data文件下提供了一组RGB-D相机数据（相邻两帧的彩色图和深度图)，且相机内参给定。使用PnP构建一个3d-2d的地图点-像素点重投影误差的非线性最小二乘问题。重投影误差函数如下：

$$\xi *=\arg \min _{\xi} \frac{1}{2} \sum_{i=1}^{n}\left\|u_{i}-\frac{1}{s_{i}} \operatorname{Kexp}(\xi \wedge) P_{i}\right\|_{2}^{2}​$$



该最小二乘问题的优化可以使用高斯牛顿GN方法，或者LM算法。

具体步骤：

（1）令第一帧的相机坐标系为世界坐标，使用第一帧图像的彩色图和深度图可以获得第一帧图像上特征点对应的特征点的在世界坐标系下的坐标。

（2）将第一帧的世界坐标系坐标投影到第二帧上得到投影点的像素坐标

（3）求取最小二乘误差函数，得到相机位姿



高斯牛顿迭代

单个投影点的误差：

$$\text {error}=u_{i}-\frac{1}{s_{i}} K \exp \left(\xi^{\wedge}\right) P_{i}$$



雅克比矩阵为：

$$J=\left[\begin{array}{cccccc}{\frac{f x}{Z^{\prime}}} & {0} & {-\frac{f x X^{\prime}}{Z^{\prime 2}}} & {-\frac{f x X^{\prime} Y^{\prime}}{Z^{\prime 2} Y^{\prime}}} & {f x+\frac{f X^{\prime 2}}{X^{\prime 2}}} & {-\frac{f x Y^{\prime}}{Z^{\prime}}} \\ {0} & {\frac{f y}{Z^{\prime}}} & {-\frac{f y Y^{\prime}}{Z^{\prime 2}}} & {-f y-\frac{f y Y^{\prime 2}}{Z^{\prime 2}}} & {\frac{f x X^{\prime} Y^{\prime}}{Z^{\prime 2}}} & {\frac{f y X^{\prime}}{Z^{\prime}}}\end{array}\right]​$$

位姿估计更新为李代数左乘模型
$$
T_{k+1}=\exp (\triangle \xi \wedge) T_{k}
$$


也可使用LM算法实现。



代码的基本框架在文件中已给出。
**提示
注意李代数的旋转平移在求导时的顺序问题 


