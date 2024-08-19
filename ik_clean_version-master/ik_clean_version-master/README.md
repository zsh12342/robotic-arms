# huawei_ik_python

在容器(或者主机)里面需要安装tkinter，并且正确设置PYTHONPATH环境变量。
```
apt install python3-tk
export PYTHONPATH=${PYTHONPATH}:/usr/lib/python3/dist-packages
```

然后运行测试脚本：
```
python3 ./torso_IK_test.py
```

# 数据
rosbag_joint.npy   : drake的rpy
rosbag_s.npy       : 四元数(x,y,z,w)
rosbag_s_origin.npy: 默认的rpy
