## KUAVO 版本选择切换工具

### 1. 介绍

用于KUAVO版本选择切换的工具，目标用户是技术支持人员，由于用户对命令行操作不熟悉，所以开发了交互式的版本切换工具。

### 2. 安装

```bash
cd dev_tools/kuavo_version_switch_tool
sudo chmod +x setup.sh
./setup.sh
```

工具会被安装到 `/usr/local/bin` 目录下, 直接在终端输入 `kuavo_version_switch_tool` 即可启动。

### 3. 使用方法

1. **启动程序：**
   在终端输入 `kuavo_version_switch_tool` 启动程序，选择**退出**或者 `Ctrl+C` 退出程序。

2. **切换版本：**
   选择需要切换的版本, 通过方向键上下移动光标，按下回车键确认选择, `Ctrl+C` 回到上一级菜单。

3. **一键编译：**
   程序会在仓库目录 `build` 中(如果没有则新建)执行 `cmake .. && make -j$(nproc)` 命令，`Ctrl+C` 中断编译。

### 4. 示例

启动程序：

![启动程序](./imgs/star_kuavo_version_switch_tool.gif)

---

切换版本：

![切换版本](./imgs/checkout_commit.gif)

---

一键编译：

![一键编译](./imgs/build_project.gif)

### 5. 删除工具
  
  ```bash
  sudo rm /usr/local/bin/kuavo_version_switch_tool
  ```

### 6. 注意事项

1. 请确保启动 `kuavo_version_switch_tool` 的终端在仓库目录下。如果不是程序会提示用户输入仓库目录。
2. 如果仓库存在未提交的更改，程序会提示用户是否丢弃更改。
