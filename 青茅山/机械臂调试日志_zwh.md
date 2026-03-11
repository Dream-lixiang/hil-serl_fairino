# 1. 实机
1. Bus 001 Device 018: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
## 1.1 机械臂使用
### 1.1.1 示教
1. 在去使能模式下拖动示教
2. 在使能模式下在线示教
## 1.2 ARUCO识别
1. 启动`realsense`:`roslaunch realsense2_camera rs_aligned_depth_1280x720.launch`
2. 启动二维码位姿估计：`roslaunch aruco_ros detect.launch `
3. 可视化：`rosrun image_view image_view image:=/aruco_1/result`，其中`image:=/aruco_1/result`可随话题选择
## 1.3 手眼标定（presented by MHF）

### 1.3.1 眼在手上

#### 1.3.1.1 准备工作

本节讨论的范围为“眼在手上”情况，即相机安装在机械臂末端。在进行标定前，请先确定满足以下条件：

#####  1. 设备条件

1. 拥有一块Aruco标定板（在线打印地址为[https://chev.me/arucogen/](https://chev.me/arucogen/)，打印时请选择origin类型）
2. 能够计算机械臂的运动学，能够输出基座和末端的tf坐标
3. 已经正确配置Realsense相机，启动rs_camera.launch正常
4. 安装完成了aruco程序并完成识别

##### 2. 下载程序

从NAS中复制功能包

或者从github下载

```bash
git clone https://gitee.com/ohhuo/handeye-calib.git
```

将下载的包复制到工作空间中

```shell
catkin_make
```

#### 1.3.1.2 配置参数

##### 1.  配置相机与Aruco

配置`aruco_start_realsense_sdk.launch`文件

```xml
    <arg name="markerId"        default="19"/>
    <arg name="markerSize"      default="0.41"/>    
    <arg name="eye"             default="left"/>
    <arg name="marker_frame"    default="aruco_marker_frame"/>
    <arg name="ref_frame"       default=""/>  
    <arg name="corner_refinement" default="LINES" /> 
    
    <node pkg="aruco_ros" type="single" name="aruco_single">
        <remap from="/camera_info" to="/camera/color/camera_info" />
        <remap from="/image" to="/camera/color/image_raw" />
        <param name="image_is_rectified" value="True"/>
        <param name="marker_size"        value="$(arg markerSize)"/>
        <param name="marker_id"          value="$(arg markerId)"/>
        <param name="reference_frame"    value="$(arg ref_frame)"/> 
        <param name="camera_frame"       value="stereo_gazebo_$(arg eye)_camera_optical_frame"/>
        <param name="marker_frame"       value="$(arg marker_frame)" />
        <param name="corner_refinement"  value="$(arg corner_refinement)" />
    </node> 
```

主要需修改参数文件如下

-  `markerId`，你打印的id编号
-  `markerSize`，实际打印出来标定版的宽度，单位m
- `/camera_info`和`/image`：如果有多个realsense相机，将参数改为要标定的相机如`/camera1/color/camera_info`


##### 2. 配置机械臂话题数据

眼在手上修改`src/handeye-calib/launch/online/online_hand_on_eye_calib.launch`

```xml
	<arg   name="arm_pose_topic"   default="/arm_pose" />
    <arg   name="camera_pose_topic"   default="/aruco_single/pose" />
    
    <arg   name="base_link"   default="/base_link" />
    <arg   name="end_link"   default="/link7" />
```

若使用的是tf方式获取机械臂位置和姿态，仅需要修改以下参数：

-  `base_link`，机械臂基坐标系名称
-  `end_link`，机械臂末端坐标系名称

若非使用tf方式，直接从话题获取机械臂姿态

-  `arm_pose_topic`, 机械臂位置和姿态所在话题数据，话题类型为：`PoseStamped`

#### 1.3.1.3 开始标定

##### 1. 启动机械臂（以履带车大族 E5 机械臂为例）

打开qt，运行主程序`MainMonitorProgram_Arm`，用于控制机械臂运动

发布基座和末端的tf坐标

```shell
roslaunch wholebody_plan sendArmEEPose.launch
```

##### 2. 发送标定板位姿数据

```shell
roslaunch handeye-calib aruco_start_realsense_sdk.launch
```

##### 3. 运行在线标定

注意标定前确定有以下数据
- 发布的tf坐标，可用此方式检查`rosrun tf tf_echo /arm_base_link /arm_end_link`。
- 标定板的位姿，可用此方式检查`rostopic echo /aruco_single/pose`

都有数据的话，即可运行在线标定程序。

```shell
source devel/setup.bash
roslaunch handeye-calib online_hand_on_eye_calib.launch
```

##### 4. 开始标定

程序运行是会对话题数据进行检测，检测是否收到机械臂数据和标定版位姿数据，如果没有会一直等待。 当检测到已经接收到数据之后，就会出现命令提示，命令定义如下：

```txt
r  rocord    记录一组手眼数据（当记录的数据大于程序计算所需的数据量之后会进行自动计算）
c  calculate 计算当前数据
s  save      保存数据
p  print     打印当前数据到屏幕上（格式为 type,x,y,z,rx,ry,rz 角度制）
q  quit      退出标定程序
```

拖拽机械臂或者用moveit移动机械臂，最终要保证相机中依然可以识别出标定板，输入`r`记录一组手眼数据，记录三组以上即可打印位姿数据。

##### 5. 获得结果

完成标定之后输入`s`即可进行保存，将保存标定结果数据和计算所使用的数据。

如眼在手外可选取结果中`base_link->camera`数据的某一算法的平均值作为最终结果

```txt
算法(end_link->camera)              x           y          z        rx         ry       rz  四元数姿态(w,x,y,z)                                                                       distance
---------------------------  --------  ----------  ---------  --------  ---------  -------  --------------------------------------------------------------------------------------  ----------
end_link->camera:Tsai-Lenz   0.123774  -0.0313415  0.02951    -1.22875  -0.139053  92.3865  [0.692197094229176, -0.0065468489872500864, -0.0085782650962608, 0.7216278368223901]      0.131046
end_link->camera:Park        0.12415   -0.030189   0.0297097  -1.26728  -0.200124  92.4468  [0.6918190405698125, -0.006390139805787927, -0.009193131580706145, 0.7219841186273189]    0.131176
end_link->camera:Horaud      0.12415   -0.0301863  0.0297108  -1.26728  -0.200277  92.4474  [0.6918152326646394, -0.006389112329921018, -0.0091940794269033, 0.7219877644394664]      0.131176
end_link->camera:Daniilidis  0.124973  -0.0375274  0.02833    -1.12196   0.146288  92.2465  [0.6930665755388047, -0.007706285117782837, -0.00617276406570523, 0.7208059600366709]     0.133526

算法平均值测试              x             y            z           rx          ry           rz         距离
----------------  -----------  ------------  -----------  -----------  ----------  -----------  -----------
mean              0.124262     -0.0323111    0.0293151    -1.22132     -0.0982914  92.3818      0.131731
var               1.92372e-07   9.29206e-06  3.30179e-07   0.00353767   0.0205628   0.00671314  1.07686e-06
std               0.000438602   0.00304829   0.000574612   0.0594783    0.143397    0.0819337   0.00103772
```

#### 1.3.1.4 代码分析

##### 1. 获取数据

进行机械臂手眼标定需要两个数据：**机械臂末端位姿、相机中标定板位姿**

**a) 获取机械臂的位姿**

相关代码如下：

```python
def online_callback(pose):
    global real_online_pose
    real_online_pose = pose.pose

rospy.Subscriber(online_pose_topic, PoseStamped, online_callback,queue_size=10)
```

其中的`pose.pose`是由`tf_to_pose.py`转发出的机械臂基座到末端的变化姿态，具体是通过建立`TransformListener`对象，监听tf坐标变换的更新，并通过`listener.lookupTransform`函数，获取从`base_link`到`end_link`的坐标变换，并把此坐标变换通过`PoseStamped`消息格式发布出去。

```python
if __name__ == '__main__':
    rospy.init_node('tf_to_pose', anonymous=False)   
    arm_pose_topic = rospy.get_param("/tf_to_pose/arm_pose_topic")
    base_link = rospy.get_param("/tf_to_pose/base_link")
    end_link = rospy.get_param("/tf_to_pose/end_link")

    rospy.loginfo("获取话题和link配置从参数服务器，要发布的话题是%s 从机械臂的%s->%s之间的变换关系" % (str(arm_pose_topic),str(base_link),str(end_link)))

    listener = tf.TransformListener()
    pub = rospy.Publisher(str(arm_pose_topic),PoseStamped,queue_size=10)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
		(trans1,rot1) = listener.lookupTransform(str(base_link),str(end_link), rospy.Time(0))
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.pose.position.x = trans1[0]
		pose.pose.position.y = trans1[1]
		pose.pose.position.z = trans1[2]

		pose.pose.orientation.x = rot1[0]
		pose.pose.orientation.y = rot1[1]
		pose.pose.orientation.z = rot1[2]
		pose.pose.orientation.w = rot1[3]

		pub.publish(pose)
        rate.sleep()
```


**b) 获取相机中标定板位姿**

标定板的位姿通过`aruco`开源程序获取，利用`aruco_ros`包输出的`/aruco_single/pose`数据，输出的也是`PoseStamped`格式的消息，包括三维位置(m)和四元数姿态。

**c) 记录数据**

通过订阅话题
- 机械臂末端相对于基座的位姿保存在`real_online_pose`中
- 标定板相对于相机的位姿保存在`real_camera_pose`中

通过下面的代码，将两个数据以列表+字典的形式进行封装。

```python
samples.append( {"robot": real_online_pose, "optical": real_camera_pose})   # 以字典形式记录数据
```

##### 2. 计算标定数据

`calculate`函数会根据记录的数据，使用四种算法计算机械臂末端与相机的位姿变换。

**a) 提取数据**

对于记录数据的`samples`列表中的每一条数据`{"robot": real_online_pose, "optical": real_camera_pose}`，都会将其中的`PoseStamped`变量(平移+四元数)的位姿转换成旋转矩阵(3,3)和平移向量(3,1)

```python
for s in samples:
	# 提取标定板的位姿
	camera_marker_msg = s['optical']
	(mcr, mct) = HandeyeCalibrationBackendOpenCV._msg_to_opencv(camera_marker_msg)  # 将ROS的transform_msg消息转换成OpenCV格式的旋转矩阵和平移向量（rot, tr）
	# 提取基座到末端的位姿
	base_hand_msg = s['robot']
	(hbr, hbt) = HandeyeCalibrationBackendOpenCV._msg_to_opencv(base_hand_msg)
```

并将各个旋转矩阵和平移向量分别做成列表

```python
	marker_camera_rot.append(mcr)
	marker_camera_tr.append(mct)
	hand_base_rot.append(hbr)
	hand_base_tr.append(hbt)
```

然后将四个列表两两组合成元组进行返回。

```python
return (hand_base_rot, hand_base_tr), (marker_camera_rot, marker_camera_tr)
```

这样得到的四个变量分别是：
- hand_base_rot：机械臂末端相对于基座的旋转矩阵
- hand_base_tr：机械臂末端相对于基座的平移向量
- marker_camera_rot：标定板相对于相机的旋转矩阵
- marker_camera_tr：标定板相对于相机的平移向量

**b) 手眼标定**

计算利用了`cv2.calibrateHandEye()`函数，该函数特用来计算手眼标定

- 输入
	- hand_base_rot：机械臂末端相对于基座的旋转矩阵
	- hand_base_tr：机械臂末端相对于基座的平移向量
	- marker_camera_rot：标定板相对于相机的旋转矩阵
	- marker_camera_tr：标定板相对于相机的平移向量
- 输出
	- hand_camera_rot：机械臂末端相对于相机坐标系的旋转矩阵
	- hand_camera_tr：机械臂末端相对于相机坐标系的平移向量

算法实际上利用`AX=XB`的思想
- A是两个相机坐标系之间的变换矩阵：利用三张标定板图片的位姿求出两个A矩阵
- B是机械臂末端前后两次的变换的齐次矩阵：利用对应的三个机械臂位姿可以求出两个B矩阵
- X为待求手眼矩阵

**c) 保存输出**

将获得的手眼标定结果`hand_camera_rot`和`hand_camera_tr`，合成成一个numpy数组，例如

```python
[[ 0.86497224 -0.29632916  0.40664753  0.457],
 [ 0.50180781  0.64203914 -0.58051084  0.482],
 [-0.00963098  0.70710678  0.70710678  0.129],
 [ 0.          0.          0.          1.   ]]
```

然后将numpy数组转换成了`(rx,ry,rz)`欧拉角、`(qx, qy, qz , qw)`四元数、`(hctx, hcty, hctz)`平移向量。

将这些数据打包返回，其中的`final_pose`是利用计算得到的标定变换矩阵，计算标定板相对于机械臂基座的变换。

```python
result_tuple = (hctx, hcty, hctz,rx,ry,rz,[qx, qy, qz , qw]),final_pose
```

### 1.3.2 眼在手外
#### 1.3.2.1 准备工作

本节讨论的范围为“眼在手外”情况，即相机通过外部支架进行安装。在进行标定前，请先确定满足以下条件：

##### 1. 设备条件

1. 拥有一块Aruco标定板（在线打印地址为 [https://chev.me/arucogen/](https://chev.me/arucogen/)，打印时请选择 origin 类型）
2. 能够计算机械臂的运动学，能够输出基座和末端的 tf 坐标
3. 已经正确配置 Realsense 相机，启动 rs_camera.launch 正常
4. 安装完成了 aruco 程序并完成识别

##### 2. 下载程序

从 NAS 中复制功能包

或者从 github 下载

```bash
git clone https://gitee.com/ohhuo/handeye-calib.git
```

将下载的包复制到工作空间中

```shell
catkin_make
```

#### 1.3.2.2 配置参数

##### 1. 配置相机与 Aruco

配置 `aruco_start_realsense_sdk.launch` 文件

```xml
    <arg name="markerId"        default="19"/>
    <arg name="markerSize"      default="0.41"/>    
    <arg name="eye"             default="left"/>
    <arg name="marker_frame"    default="aruco_marker_frame"/>
    <arg name="ref_frame"       default=""/>  
    <arg name="corner_refinement" default="LINES" /> 
    
    <node pkg="aruco_ros" type="single" name="aruco_single">
        <remap from="/camera_info" to="/camera/color/camera_info" />
        <remap from="/image" to="/camera/color/image_raw" />
        <param name="image_is_rectified" value="True"/>
        <param name="marker_size"        value="$(arg markerSize)"/>
        <param name="marker_id"          value="$(arg markerId)"/>
        <param name="reference_frame"    value="$(arg ref_frame)"/> 
        <param name="camera_frame"       value="stereo_gazebo_$(arg eye)_camera_optical_frame"/>
        <param name="marker_frame"       value="$(arg marker_frame)" />
        <param name="corner_refinement"  value="$(arg corner_refinement)" />
    </node> 
```

主要需修改参数文件如下

-  `markerId`，你打印的 id 编号
-  `markerSize`，实际打印出来标定版的宽度，单位 m
- `/camera_info` 和 `/image`：如果有多个realsense相机，将参数改为要标定的相机如 `/camera1/color/camera_info`
##### 2. 配置机械臂话题数据
眼在手上修改 `src/handeye-calib/launch/online/online_hand_on_eye_calib.launch`

```xml
	<arg   name="arm_pose_topic"   default="/arm_pose" />
    <arg   name="camera_pose_topic"   default="/aruco_single/pose" />
    
    <arg   name="base_link"   default="/base_link" />
    <arg   name="end_link"   default="/link7" />
```

若使用的是 tf 方式获取机械臂位置和姿态，仅需要修改以下参数：

-  `base_link`，机械臂基坐标系名称
-  `end_link`，机械臂末端坐标系名称

若非使用 tf 方式，直接从话题获取机械臂姿态，请修改

-  `arm_pose_topic`, 机械臂末端位置和姿态所在话题数据，话题类型为：`PoseStamped`

#### 1.3.2.3 开始标定

#####  1. 启动机械臂（以法奥 FR5 机械臂为例）

打开qt，运行主程序 `MainMonitorProgram_Arm`，用于控制机械臂运动，发布末端位姿。

```shell
python realrobot/test/04_keyboard_control.py
```
##### 2. 发送标定板位姿数据

```shell
# 别忘了修改markerId
roslaunch handeye-calib aruco_start_realsense_sdk.launch
```
##### 3. 运行在线标定

注意标定前确定有以下数据
- 机器人末端位姿，可用此方式检查 `rostopic echo /robot_pose`
- 标定板的位姿，可用此方式检查 `rostopic echo /aruco_single/pose`

都有数据的话，即可运行在线标定程序。

```shell
source devel/setup.bash
roslaunch handeye-calib online_hand_on_eye_calib.launch
```

#####  4. 开始标定

程序运行是会对话题数据进行检测，检测是否收到机械臂数据和标定版位姿数据，如果没有会一直等待。 当检测到已经接收到数据之后，就会出现命令提示，命令定义如下：

```txt
r  rocord    记录一组手眼数据（当记录的数据大于程序计算所需的数据量之后会进行自动计算）
c  calculate 计算当前数据
s  save      保存数据
p  print     打印当前数据到屏幕上（格式为 type,x,y,z,rx,ry,rz 角度制）
q  quit      退出标定程序
```

拖拽机械臂或者用moveit移动机械臂，最终要保证相机中依然可以识别出标定板，输入 `r` 记录一组手眼数据，记录三组以上即可打印位姿数据。

##### 5. 获得结果

完成标定之后输入 `s` 即可进行保存，将保存标定结果数据和计算所使用的数据。

如眼在手外可选取结果中 `base_link->camera` 数据的某一算法的平均值作为最终结果

```txt
算法(end_link->camera)              x           y          z        rx         ry       rz  四元数姿态(w,x,y,z)                                                                       distance
---------------------------  --------  ----------  ---------  --------  ---------  -------  --------------------------------------------------------------------------------------  ----------
end_link->camera:Tsai-Lenz   0.123774  -0.0313415  0.02951    -1.22875  -0.139053  92.3865  [0.692197094229176, -0.0065468489872500864, -0.0085782650962608, 0.7216278368223901]      0.131046
end_link->camera:Park        0.12415   -0.030189   0.0297097  -1.26728  -0.200124  92.4468  [0.6918190405698125, -0.006390139805787927, -0.009193131580706145, 0.7219841186273189]    0.131176
end_link->camera:Horaud      0.12415   -0.0301863  0.0297108  -1.26728  -0.200277  92.4474  [0.6918152326646394, -0.006389112329921018, -0.0091940794269033, 0.7219877644394664]      0.131176
end_link->camera:Daniilidis  0.124973  -0.0375274  0.02833    -1.12196   0.146288  92.2465  [0.6930665755388047, -0.007706285117782837, -0.00617276406570523, 0.7208059600366709]     0.133526

算法平均值测试              x             y            z           rx          ry           rz         距离
----------------  -----------  ------------  -----------  -----------  ----------  -----------  -----------
mean              0.124262     -0.0323111    0.0293151    -1.22132     -0.0982914  92.3818      0.131731
var               1.92372e-07   9.29206e-06  3.30179e-07   0.00353767   0.0205628   0.00671314  1.07686e-06
std               0.000438602   0.00304829   0.000574612   0.0594783    0.143397    0.0819337   0.00103772
```

> 参考文章
> 1. [鱼香ROS/handeye-calib](https://gitee.com/ohhuo/handeye-calib)
### 1.3.3 坑
1. 程序启动顺序：
	1. 先启动`roslaunch handeye-calib aruco_start_realsense_sdk.launch`，在base环境下
	2. 再启动`python realrobot/test/XWD/12_robot_pose.py`，在`DP4SurReal`环境下
	3. 最后启动`roslaunch handeye-calib online_hand_to_eye_calib.launch`，一定在主环境下
2. 使用前修改`aruco_start_realsense_sdk.launch`中的 `<arg name="markerId" default=""/>`与`<arg name="markerSize" default=""/>`

### 1.3.4 我自己的坑
1. 程序启动顺序：
	1. 先启动`roslaunch handeye-calib aruco_start_realsense_sdk.launch'
`，在base环境下
	2. 再启动`python /home/yujp/diffusion_policy_XWD/realrobot/test/XWD/12_robot_pose.py`
`，在`genesis`环境下
	3. 最后启动`roslaunch handeye-calib online_hand_to_eye_calib.launch`，base环境下
2. 使用前修改`aruco_start_realsense_sdk.launch`中的 `<arg name="markerId" default=""/>`与`<arg name="markerSize" default=""/>`
## 1.4 摄像头配置与安装
### 1.4.1 摄像头相关配置参数查看
1. 安装`V4L2`功能包：`sudo apt-get install v4l-utils`
2. 拔插摄像头，并查看拔插前后的区别以判断所使用的设备：`sudo ls /dev/video*`
3. 查看USB摄像头设备信息：`sudo v4l2-ctl --list-devices` 或者 `ls usb`
4. 查看USB摄像头分辨率信息，以`/dev/video0`为例：`v4l2-ctl -d /dev/video0 --list-formats-ext`
5. 查看USB摄像头不同压缩格式支持的分辨率：
	1. MJPG压缩格式：`   sudo v4l2-ctl --list-framesizes=MJPG -d /dev/video0`
	2. YUYV422压缩格式：`  sudo v4l2-ctl --list-framesizes=YUYV -d /dev/video0`
6. 查看USB摄像头详细参数：`v4l2-ctl -d /dev/video0 -D`
7. 查看USB摄像头所有参数：`sudo v4l2-ctl -d  /dev/video0 --all`
8. 帮助命令：`sudo v4l2-ctl help`
### 1.4.2 打开摄像头
1. 安装`茄子`：`sudo apt-get install cheese`
2. 打开指定设备：` cheese -d /dev/video0`
## 1.5 夹爪调试
1. 要安装usb转串口的驱动程序：搜索`cp2102 usb to UART bridge controller`，具体步骤可参考 ----> (#https://blog.csdn.net/braised_fish/article/details/137071586)
2. 烧录程序时报错：`F:\Apersonal\Doctor\设备调试\夹爪\ServoDriverST\wifi\CONNECT.h: In function 'void espNowInit()': F:\Apersonal\Doctor\设备调试\夹爪\ServoDriverST\wifi\CONNECT.h:328:28: error: invalid conversion from 'void (*)(const uint8_t*, const uint8_t*, int)' {aka 'void (*)(const unsigned char*, const unsigned char*, int)'} to 'esp_now_recv_cb_t' {aka 'void (*)(const esp_now_recv_info*, const unsigned char*, int)'} [-fpermissive]`
	解决方法：将`CONNECT.h`文件中第328行注释掉，即可正常烧录
3. 注意左右两个夹爪舵机的id，烧录开合程序时，请对应修改ID数组的各元素：
	1. 左臂夹爪舵机id分别为0、1
	2. 右臂夹爪舵机id分别为1、2
4. 左臂夹爪与右臂夹爪对应的`/dev/TZH-1`和`/dev/TZH-2`可能会替换，每次收集数据时要实现确认
5. 如果出现`esp32`无法与手机相连的情况，更改`wifiInit`中的`AP`设置函数，指定一个信道，比如`6`
## 1.6 杂项
1. vpn全局模式下无法进入192.168.58.2（机械臂控制界面）
	1. 也不一定，现在可以进去了
2. 在未安装`ros`的情况下，无法在`conda`环境下安装`rospy`；安装`ros`后，在相应的`conda`环境下执行`pip install rospkg`即可在该虚拟环境下使用`ros`
3. 夹爪的设备ID为`0403:6001`
4. 下载`force-dimension python sdk`时指定源，否则下不动：`pip install forcedimension-core -i https://pypi.tuna.tsinghua.edu.cn/simple`
5. aruco_ros
6. 在`Terminator`下，`pip`、`conda`、`sudo install`等下载奇慢，直接在系统自带终端中下载
7. usb扩展槽前头的usb3.0尽量别用
# 2. 仿真
## 2.1 模型设置
1. 删除`big_ring_cyl`中的碰撞：
```
<collision>  
    <origin rpy="0 0 0" xyz="0 0 0"/>  
    <geometry>  
        <mesh filename="meshes/big_ring_cyl.STL" scale="1 1 1"/>  
        <!-- <mesh filename="meshes/ring_cyl_vhacd.obj" scale="0.05 0.05 0.05"/>   -->  
        <!--        <box size="0.002 0.002 0.002"/>-->  
    </geometry>  
</collision>
```
2. `needle_40mm_RL.urdf`中的碰撞不要删除，否则环境初始化时针抓取失败：
```
<collision>  
    <origin rpy="0 0 0" xyz="0 0 0"/>  
    <geometry>  
        <mesh filename="meshes/needle_40mm_vhacd.obj" scale="1 1 1"/>  
    </geometry>  
</collision>
```
3. `needle_40mm_RL.urdf`针大，`needle_40mm.urdf`针小与实物接近。`needle_40mm_RL`的质量为`0.01`，`needle_40mm`的初始质量为`0.51`，后改为`0.01`，否则初始环境设置时，针抓取失败，会从夹爪中自动脱落

# 3. 主从调试 
## 3.1 sdk安装
1. 下载官方sdk：[主手sdk下载]https://forcedimension.com/software/sdk
		![[Pasted image 20240309145456.png]]
2. 解压下载的文件，`tar xjvf 文件名`
3. 使用`sudo make`命令安装
	1. 若提示没有`找不到 -lusb-1.0`，去安装libusb，详细安装步骤请参考：[libusb安装与调试]https://blog.csdn.net/jiguangfan/article/details/86492698?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522170996765116800182163402%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=170996765116800182163402&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-86492698-nullnull.142^v99^pc_search_result_base9&utm_term=libusb%E5%AE%89%E8%A3%85%E4%B8%8E%E8%B0%83%E8%AF%95&spm=1018.2226.3001.4187
	2. 若提示：fatal error：GL/glu.h：没有那个文件或目录，说明缺少该包，键入以下命令
			`sudo apt-get install freeglut3-dev`
	3. 若提示：fatal error：GLFW/glfw3.h：没有那个文件或目录，说明缺少该包，键入以下命令：
			`sudo apt install libglfw3 libglfw3-dev`
	4. 若提示`找不到 -lXcursor`，键入以下命令
			`sudo apt install libxcursor-dev`
	5. 若提示`找不到 -lXinerama`，键入以下命令
			`sudo apt install libxinerama-dev`
4. 在`python`环境下，安装sdk：`pip install forcedimension-core`
## 3.2 校准
1. 校准步骤：注意，<font color="#d99694">每次启动后都要校准</font>
	1. 确保插设备通电，打开主手开关，此时会发现状态灯闪烁（下图8）
			![[Pasted image 20240309183114.png]]
	2. 活动主手的关节，使其分别达到限位位置，即可完成校准，此时状态灯将停止闪烁，处于常亮状态
## 3.3 使用
1. HapticDesk，位于bin目录下，终端中，进入该文件夹并使用`./HapticDesk`来运行该程序
	1. 如启动时报错，缺少`libQt5Widget.so.5`，键入以下命令解决
			`sudo apt install qt5-defalut libqt5widgets5`
	2. 遇到`段错误（核心已转储）`
		1. 这通常意味着程序试图访问它没有权限访问的内存区域
			1. 首先，确保当前用户有权限访问ForeceDimension设备。Linux系统通常对USB设备的访问权限有严格的控制，可使用以下命令启动：`sudo ./HapticDesk`
