<img src="https://i.imgur.com/qI1Jfyl.gif"  width="60%"/>

# apriltag
apriltag的检测实现的ROS包，无需进行安装
## apriltag_mit

a c++ library from http://people.csail.mit.edu/kaess/apriltags/

## apriltag_umich

a c library from http://april.eecs.umich.edu/wiki/index.php/AprilTags

To decide which detector to use, have a look at
[the performance comparison of UMich vs MIT detectors](docs/performance_comparison.md).

## apriltag_node

a ros node for detecting apriltag.

检测apriltags的图像topic： `/usb_cam/camera_info`
检测apriltags的相机内参： `/usb_cam/image_raw`

```
roslaunch apriltag_detector usb_cam_apriltags.launch
```

## apriltag_msgs

apriltag ros messages
<img align="right" src="https://i.imgur.com/BzOnbkS.gif" />

# aprilTAg生成

官方提供的AprilTag 3系列的所有标签的图像：https://github.com/AprilRobotics/apriltag-imgs
# 方法一，通过alltags.ps生成全部的高清AprilTag标签图片
先安装“ ghostscript”软件包：

    sudo apt-get install ghostscript

使用ps2pdf命令进行转换

    ps2pdf alltags.ps alltags.pdf

alltags.pdf文件中就是高清AprilTag标签图片。

# 方法二，生成具体的二维码
 这样生成的二维码大小为7.5cm 

    cd ~/下载/apriltag-imgs-master/tag16h5/
    convert tag16_05_00000.png -scale 5000% big.png
