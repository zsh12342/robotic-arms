# drake install

以下说明适用于Ubuntu系统。

### Install prereqs

- 克隆github上的drake仓库。
  ```
  https://github.com/RobotLocomotion/drake.git
  ```
- 安装依赖。
  ```
  cd drake/
  sudo ./setup/ubuntu/install_prereqs.sh
  ```
  等待所有依赖安装完成。

### Install drake

- [此链接](https://github.com/RobotLocomotion/drake/releases)下载二进制发行包，20.04可选择`drake-xxx-focal.tar.gz`（`xxx`代表具体日期）。

- 安装到系统目录(`xxx`替换为下载的内容)。
  ```
  sudo tar -xvzf drake-xxx-focal.tar.gz -C /opt
  ```

- 在`~/.bashrc`中添加以下内容。
  ```
  export LD_LIBRARY_PATH="/opt/drake/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export PATH="/opt/drake/bin${PATH:+:${PATH}}"
  export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
  ```