# hdl_graph_slam_noted

![](http://39.107.30.202:8080/s/R4KdjCib8RmPdrw/preview)

![](http://39.107.30.202:8080/s/9pzPNkJa63GotyW/preview)

## 1. hdl_slam中的地面约束

![image-20200429160432101](/home/alex/SynologyDrive/self/markdown/A日常计划/代码阅读/hdl_slam.assets/image-20200429160432101.png)

在`hdl_slam`中的每一帧关键帧都会对应提取一个地面参数；

建图开始的时候将一个`Floor Plane Node`设置为true，并固定，参数设置为决定的垂直于地面的法向量[0,0,1,0]

后面的每一帧`keyframe`的地面参数`coeffs`都要和`Floor Plane Node`构建平面误差，如下所示：

```C++
void computeError() override {
    const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]); //T:[R,t]
    const g2o::VertexPlane* v2 = static_cast<const g2o::VertexPlane*>(_vertices[1]); //[0,0,1,0]

    Eigen::Isometry3d w2n = v1->estimate().inverse();// T.inverse()= T_iw;i是当前帧，w是世界坐标系
    Plane3D local_plane = w2n * v2->estimate();// T_iw*[0 0 1 0]= 将一个绝对的垂直于Z的法向量，投影到当前帧；
    _error = local_plane.ominus(_measurement);//将[0 0 1 0]变换到当前帧之后，和当前帧的地面的法向量 做ominus 作为误差
}
```

思路是，根据当前第i帧keyframe的位姿$T_{wi}$，将初始的严格垂直于地面的法向量[0,0,1,0]变换到第i帧的坐标系下；

并用这个参数来初始化Plane3D local_plane;其底层对`*`操作符进行了重载；具体如下：

```C++
inline Plane3D operator*(const Isometry3& t, const Plane3D& plane){
    Vector4 v=plane._coeffs;
    Vector4 v2;
    Matrix3 R=t.rotation();
    v2.head<3>() = R*v.head<3>();
    v2(3)=v(3) - t.translation().dot(v2.head<3>());
    return Plane3D(v2);
};
```

可以看到，传入参数`t`就是$T_{iw}$，`plane`就是[0,0,1,0]，经过一个变换，变成到当前帧雷达坐标系下的一个Vector4d值。



再使用这个值和当前帧点云拟合出来的地面点，进行一个$\ominus$操作，获得误差，误差的具体定义如下：

```C++
inline Vector3 ominus(const Plane3D& plane){
    //construct the rotation that would bring the plane normal in (1 0 0)
    Matrix3 R=rotation(normal()).transpose();
    Vector3 n=R*plane.normal();
    number_t d=distance()-plane.distance();
    return Vector3(azimuth(n), elevation(n), d);
}
```

那么平面误差是如何定义的？

下面则是重点推导部分。



## 2. g2o中的平面误差推导

首先，在三维平面下，平面方程可以使用4个参数来表示：
$$
    Ax+By+Cz+D=0
$$
所以，如果想要达到hdl_slam中对地面的约束，就需要借助平面方程，对地面点云进行约束；

**1.当前帧点云的地面方程：**

那么，当前帧雷达点云的地面方程拟合可以借鉴这篇文章[[透彻理解]由最小二乘到SVD分解](https://blog.csdn.net/supengufo/article/details/104553094)，那么可以获得当前帧雷达点云(第i帧)的地面方程为$plane_i$：
$$
A_ix+B_iy+C_iz+D_i=0
$$
该参数记做$P_i\in \mathbb{R}^4$

**2.全局坐标系下的点云地面方程：**

hdl_slam对此约束的处理思路是，最开始初始化一个全局地面方程参数为`[0,0,1,0]`，记为：
$$
A_wx+B_wy+C_wz+D_w=0
$$
也即，$A_w=B_w=D_w=0,C_w=1$，该参数记做$P_w \in \mathbb{R}^4$



**3. 平面方程的坐标系统一**

上面构建的两个平面方程，其坐标系并不统一，因此需要统一坐标系；按照hdl_slam的理解，我们将世界坐标系下的平面方程$P_w$投影到当前第i帧雷达坐标系下$T_wi$，(该位姿视作已知)：
$$
P_w' = T_{iw}P_w=T_{wi}^{-1}P_w = [A_w' \ \ B_w' \ \ C_w' \ \ D_w']
$$
然后在第$i$帧雷达坐标系下进行平面方程的误差构建，即如下代码部分：

```C++
Eigen::Isometry3d w2n = v1->estimate().inverse();
Plane3D local_plane = w2n * v2->estimate();
_error = local_plane.ominus(_measurement);
```


**4. g2o中的平面误差**

对$P_w'，P_i$两个平面参数进行$\ominus$操作，分为以下几步：

- 1.`Matrix3 R=rotation(normal()).transpose();`

  - 以$P_w'$的向量(前三维，也即法向量)为基底，将$P_w'$转换为旋转矩阵。

- `Vector3 n=R*plane.normal();`

  - 将$P_i$转换到以$P_w'$为基底的旋转空间中去，得到向量n;

    ![image-20200429182625982](/home/alex/SynologyDrive/self/markdown/A日常计划/代码阅读/hdl_slam.assets/image-20200429182625982.png)

- 对n计算仰角`azimuth`和方位角`elevation`

- 计算$d=D_w'-D_i$

- 误差组合为`Vector3(azimuth(n), elevation(n), d)`

