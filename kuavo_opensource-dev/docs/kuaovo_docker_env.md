镜像下载地：https://kuavo.lejurobot.com/docker_images/leju_drake_docker_1_9_1_1.tar

1. 使用 docker load < ./xxx.tar 加载镜像
2. 使用 `sudo docker run --privileged -v {需要挂载目录的文正路径}:/home/carlos --name {容器名称} -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix image:tags /bin/bash` 启动容器
1. 如果是 vnc 远程访问，需要用  xhost +local:docker 这条命令允许 docker 使用 vnc, 然后在同一个终端下启动 docker. 启动之后可以先用 xeyes 测试一下是不是正常工作。这样才能在 VNC 里面通过 docker 容器启动 drake-visualizer.image.
2. 在 docker 里面启动仿真界面使用  drake-visualizer.image


