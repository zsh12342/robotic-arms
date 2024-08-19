#!/usr/bin/env python
# coding=utf-8
'''
Author: dongdongmingming
Date: 2024-04-17 17:31:40
LastEditors: Please set LastEditors
LastEditTime: 2024-04-25 14:41:52
FilePath: /kuavo/tools/check_tool/Hardware_tool.py
Description: 硬件测试脚本
'''

import time
import os,sys
import subprocess
import re
import shutil


if sys.version_info[0] == 2:
    print("你正在使用 Python 2.x , 请更换运行指令为：$ sudo python3 tools/check_tool/Hardware_tool.py ")
    exit()


folder_path = os.path.dirname(os.path.abspath(__file__))    # check_tool/

sys.path.append('/home/lab/.local/lib/python3.8/site-packages/')
sys.path.append(os.path.join(folder_path,"Ruierman"))

import serial
import serial.tools.list_ports
import dynamixel_servo


import ruierman
import claw_rs485


servo_usb_path = "/dev/usb_servo"
claw_usb_path = "/dev/claw_serial"
handL_usb_path = "/dev/stark_serial_L"
handR_usb_path = "/dev/stark_serial_R"

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# print(bcolors.HEADER + "This is a header" + bcolors.ENDC)
# print(bcolors.OKBLUE + "This is a blue text" + bcolors.ENDC)
# print(bcolors.OKCYAN + "This is a cyan text" + bcolors.ENDC)
# print(bcolors.OKGREEN + "This is a green text" + bcolors.ENDC)
# print(bcolors.WARNING + "This is a warning text" + bcolors.ENDC)
# print(bcolors.FAIL + "This is an error text" + bcolors.ENDC)
# print(bcolors.BOLD + "This is a bold text" + bcolors.ENDC)
# print(bcolors.UNDERLINE + "This is an underlined text" + bcolors.ENDC)


def get_robot_version():
    # 获取用户的主目录
    home_dir = os.path.expanduser('/home/lab/')
    bashrc_path = os.path.join(home_dir, '.bashrc')

    # 初始化变量
    robot_version = None

    # 读取 .bashrc 文件
    try:
        with open(bashrc_path, 'r') as file:
            for line in file:
                # 查找 export ROBOT_VERSION= 行
                if line.startswith('export ROBOT_VERSION='):
                    # 提取变量值
                    robot_version = line.split('=')[1].strip()
                    break
    except FileNotFoundError:
        print(f"文件 {bashrc_path} 不存在")

    return robot_version


def get_core_count():
    try:
        # Execute the 'nproc' command
        result = subprocess.run(['nproc'], capture_output=True, text=True, check=True)
        core_count = result.stdout.strip()
        return int(core_count)
    except subprocess.CalledProcessError as e:
        print(f"Error executing nproc: {e}")
        return None


def usb_port():
    print("Hardware_tool begin")

    # 获取所有连接的串口设备
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        # print(port)
        print("设备: {}".format(port.device))
        print("描述: {}".format(port.description))
        print("hwid: {}".format(port.hwid))
        print("------------------------------")

    # 舵机串口
    if os.path.exists(servo_usb_path):
        print(bcolors.OKGREEN + "{} 舵机出串口设备存在".format(servo_usb_path) + bcolors.ENDC)
    else:
        print(bcolors.FAIL + "{} 舵机出串口设备不存在".format(servo_usb_path) + bcolors.ENDC)


    # RS485 USB
    if os.path.exists(claw_usb_path):
        print(bcolors.OKGREEN + "{} 手抓串口设备存在".format(claw_usb_path) + bcolors.ENDC)
    else:
        print(bcolors.FAIL + "{} 手抓串口设备不存在".format(claw_usb_path) + bcolors.ENDC)

    # hand L RS485 USB
    if os.path.exists(handL_usb_path):
        print(bcolors.OKGREEN + "{} 左强脑手串口设备存在".format(handL_usb_path) + bcolors.ENDC)
    else:
        print(bcolors.FAIL + "{} 左强脑手串口设备不存在".format(handL_usb_path) + bcolors.ENDC)
    # hand R RS485 USB
    if os.path.exists(handR_usb_path):
        print(bcolors.OKGREEN + "{} 右强脑手串口设备存在".format(handR_usb_path) + bcolors.ENDC)
    else:
        print(bcolors.FAIL + "{} 右强脑手串口设备不存在".format(handR_usb_path) + bcolors.ENDC)


    result,canBus = ruierman.canbus_open()
    if(result == True):
        print(bcolors.OKGREEN + "canbus open success 设备存在" + bcolors.ENDC)
    else:
        print(bcolors.FAIL + "canbus open fail 设备不存在" + bcolors.ENDC)
    time.sleep(2)
    canBus.close_canbus()


def imu_software():
    # 定义要运行的命令
    command = "/home/lab/mtmanager/linux-x64/bin/mtmanager" 

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)


def imu_test():
    # 定义要运行的命令
    if(folder_path.startswith("/home/lab/kuavo_opensource/")):
        command = "sudo /home/lab/kuavo_opensource/bin/imu_test"
    else:
        command = "sudo "+ folder_path +"/../../build/lib/xsens_ros_mti_driver/imu_test" 

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)


def dxl_zero():
    # 定义要运行的命令
    if(folder_path.startswith("/home/lab/kuavo_opensource/")):
        command = "sudo bash /home/lab/kuavo_opensource/bin/start_tools.sh /home/lab/kuavo_opensource/bin/dynamixel_calibrate_servos  --record"
    else:
        command =  folder_path +"/../../build/lib/DynamixelSDK/dynamixel_calibrate_servos --record" 

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

def update_kuavo():
    # 定义要运行的命令
    command = "bash "+ folder_path +"/update_kuavo.sh"  

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)


def update_kuavo_opensource():
    # 定义要运行的命令
    command = "bash "+ folder_path +"/update_kuavo_opensource.sh"  

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)


def folder_backup():
    # 定义要运行的命令
    command = "bash "+ folder_path +"/folder_backups.sh"  

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

def control_H12():
    # 定义要运行的命令
    command = "bash "+ folder_path +"/creat_remote_udev_rule.sh"  

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

def change_imu_usb():
    file_path = '/etc/udev/rules.d/imu.rules'
    new_rule = 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="d388", MODE:="0777", ATTR{latency_timer}="1"'
    if not os.path.exists(file_path) or not os.access(file_path, os.W_OK):
        print("Error: {} does not exist or you do not have write permission.".format(file_path))
        exit(1)

    with open(file_path, 'r') as file:
        lines = file.readlines()

    found = False
    with open(file_path, 'w') as file:
        for line in lines:
            # 使用正则表达式匹配规则（假设整行都是规则）
            if re.match(r'^\s*KERNEL=="ttyUSB.*', line):
                # 替换现有规则（如果需要）
                file.write(new_rule)
                found = True
            else:
                # 保留其他行
                file.write(line)

        # 如果未找到匹配项，则在文件末尾添加新规则
        if not found:
            file.write(new_rule)

    print("Rule has been added or replaced in {}".format(file_path))

def elmo_position_read():
    almoZR_path = folder_path + "/elmoZeroRead.py"
    print("1.复制运行该行命令修改内容进行零点数据转换：code " + almoZR_path)
    print("2.复制运行指令，将运行结果复制粘贴到零点文件中保存：python " + almoZR_path)
    print("3.复制运行该行命令打开零点文件进行修改：code /home/lab/.config/lejuconfig/offset.csv")

def claw_usb(choice):

    device_list = []
    
    # 获取所有连接的串口设备
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if(port.description == "LJ485A - LJ485A"):
            hwid_string = port.hwid
            # 编写正则表达式
            pattern = re.compile(r"SER=([0-9A-Z]+)")
            # 使用findall方法查找所有匹配项，这将返回一个包含所有匹配结果的列表
            matches = pattern.findall(hwid_string)
            # 输出SER值
            for match in matches:
                print("串口：", port.device,"SER", match)  # 输出: DB81ASSB

                device_list.append(port.device)
    
        if(port.description == "LJ485B - LJ485B"):
            hwid_string = port.hwid
            # 编写正则表达式
            pattern = re.compile(r"SER=([0-9A-Z]+)")
            # 使用findall方法查找所有匹配项，这将返回一个包含所有匹配结果的列表
            matches = pattern.findall(hwid_string)
            # 输出SER值
            for match in matches:
                print("串口：", port.device,"SER", match)  # 输出: DB81ASSB

                device_list.append(port.device)

    if len(device_list) == 2:
        # 定义脚本路径和参数
        arg1 = device_list[choice]
        arg2 = "claw_serial"
        # 定义要运行的命令
        command = "sudo bash "+ folder_path +"/generate_serial.sh "  +  arg1  + " " + arg2
        print(command)
        # 使用 subprocess.run() 运行命令
        subprocess.run(command, shell=True)
    else:
        print(bcolors.WARNING + "失败，请确认kuavo电源板是否正常" + bcolors.ENDC)



def hand_usb():

    device_list = []
    
    # 获取所有连接的串口设备
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if(port.description == "LJ485A - LJ485A"):
            hwid_string = port.hwid
            # 编写正则表达式
            pattern = re.compile(r"SER=([0-9A-Z]+)")
            # 使用findall方法查找所有匹配项，这将返回一个包含所有匹配结果的列表
            matches = pattern.findall(hwid_string)
            # 输出SER值
            for match in matches:
                print("串口：", port.device,"SER", match)  # 输出: DB81ASSB

                device_list.append(port.device)

        if(port.description == "LJ485B - LJ485B"):
            hwid_string = port.hwid
            # 编写正则表达式
            pattern = re.compile(r"SER=([0-9A-Z]+)")
            # 使用findall方法查找所有匹配项，这将返回一个包含所有匹配结果的列表
            matches = pattern.findall(hwid_string)
            # 输出SER值
            for match in matches:
                print("串口：", port.device,"SER", match)  # 输出: DB81ASSB

                device_list.append(port.device)


    if len(device_list) == 2:
        # 定义脚本路径和参数
        arg1 = device_list[1]
        arg2 = "stark_serial_R"
        # 定义要运行的命令
        command = "sudo bash "+ folder_path +"/generate_serial.sh "  +  arg1  + " " + arg2
        print(command)
        # 使用 subprocess.run() 运行命令
        subprocess.run(command, shell=True)

        
        # 定义脚本路径和参数
        arg1 = device_list[0]
        arg2 = "stark_serial_L"
        # 定义要运行的命令
        command = "sudo bash "+ folder_path +"/generate_serial.sh "  +  arg1  + " " + arg2
        print(command)
        # 使用 subprocess.run() 运行命令
        subprocess.run(command, shell=True)
    else:
        print(bcolors.WARNING + "失败，kuavo电源板485设备连接异常" + bcolors.ENDC)

def ruiwo_zero():
    source_file = '/home/lab/kuavo_opensource/lib/ruiwo_controller/config.yaml'
    target_file = '/home/lab/.config/lejuconfig/config.yaml'
    if not os.path.exists(source_file):
        print("kuavo_opensource 瑞沃电机 config.yaml 文件丢失")
    elif not os.path.exists(target_file):
        # 如果不存在，则复制源文件到目标位置
        shutil.copy2(source_file, target_file)
        print("Copied {} to {}".format(source_file, target_file))
    else:
        print("{} already exists.".format(target_file))

    # 定义要运行的命令
    command = "bash "+ folder_path +"/ruiwo_zero_set.sh" 

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

def ruiwo_negtive():
    source_file = '/home/lab/kuavo_opensource/lib/ruiwo_controller/config.yaml'
    target_file = '/home/lab/.config/lejuconfig/config.yaml'
    if not os.path.exists(source_file):
        print("kuavo_opensource 瑞沃电机 config.yaml 文件丢失")
    elif not os.path.exists(target_file):
        # 如果不存在，则复制源文件到目标位置
        shutil.copy2(source_file, target_file)
        print("Copied {} to {}".format(source_file, target_file))
    else:
        print("{} already exists.".format(target_file))

    # 定义要运行的命令
    command = "bash "+ folder_path +"/ruiwo_negtive_set.sh" 

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)


def qiangnao_hand():
    # 定义要运行的命令
    command = "bash "+ folder_path +"/hand_grab_test.sh" 

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)


def compile_kuavo_opensource():
    # 定义要运行的命令
    command = "bash "+ folder_path +"/compile_kuavo_opensource.sh"  

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

def compile_kuavo():
    # 定义要运行的命令
    command = "bash "+ folder_path +"/compile_kuavo.sh"  

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

def license_sign():
    license_str = input("请输入提供的License：")
    # 定义要运行的命令
    command = "bash "+ folder_path +"/EtherCAT_license.sh "  + license_str

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)
    print("请检查机器人程序运行后有显示 EtherCAT license is verified! 为license注册成功。")

if __name__ == '__main__':

    # 获取 ROBOT_VERSION 变量值
    robot_version = get_robot_version()
    if robot_version:
        print("ROBOT_VERSION={}".format(robot_version))
    else:
        print("未找到 ROBOT_VERSION 变量")

    core_count = get_core_count()
    if core_count is not None:
        print("8 Number of CPU cores: {}".format(core_count))

    # 提示用户选择
    print(bcolors.BOLD + "请输入一个选项按回车(q回车退出)：" + bcolors.ENDC)
    print("0. 打开零点文件")
    print("1. 硬件连接检查")
    print("2. 打开imu上位机软件(接屏幕)")
    print("3. 测试imu(先编译)")
    print("4. 瑞尔曼电机设置ID")
    print("5. 瑞尔曼电机设置零点")
    print("6. 瑞尔曼电机测试(先设置零点)")
    print("7. 测试舵机")
    print("8. 舵机校准")
    print("9. 配置夹爪usb")
    print("a. 测试夹爪")
    print("b. 配置强脑手usb")
    print("c. 测试强脑手")
    print("d. 瑞沃电机设置零点")
    print("e. 瑞沃电机辨识方向(注意电机限位不要堵转)")    
    print("f. 零点文件备份")
    print("g. 遥控器配置usb")
    print("h. 新款IMU配置usb")
    print("i. 电机零点数据转换")
    print("k. 更新当前目录程序(注意：会重置文件内容，建议备份零点urdf)")
    print("r. 更新kuavo_opensource程序(注意：会重置文件内容，建议备份零点urdf)")
    print("m. 编译kuavo_opensource(请先更新，不要用sudo运行,其他需要使用sudo)")
    print("n. 编译当前目录(请先更新，不要用sudo运行,其他需要使用sudo)")
    print("l. license导入")

    # 获取用户输入的选项
    option = input("请输入选项编号：")

    # 如果用户选择退出，则退出循环
    if option == "q":
        print("已退出")
        exit()

    # 根据用户选择执行相应的操作
    if option == "0":
        print(bcolors.HEADER + "###开始，打开零点文件###" + bcolors.ENDC)
        print("复制运行该行命令打开：code /home/lab/.config/lejuconfig/offset.csv")
        print(bcolors.HEADER + "###结束，打开零点文件###" + bcolors.ENDC)
    elif option == "1":
        print(bcolors.HEADER + "###开始，硬件连接检查###" + bcolors.ENDC)
        usb_port()
        print(bcolors.HEADER + "###结束，硬件连接检查###" + bcolors.ENDC)
    elif option == "2":
        print(bcolors.HEADER + "###正在打开imu上位机软件###" + bcolors.ENDC)
        imu_software()
        print(bcolors.HEADER + "###已打开imu上位机软件###" + bcolors.ENDC)
    elif option == "3":
        print(bcolors.HEADER + "###开始，测试imu###" + bcolors.ENDC)
        imu_test()
        print(bcolors.HEADER + "###结束，测试imu###" + bcolors.ENDC)
    elif option == "4":
        print(bcolors.HEADER + "###开始，设置电机ID###" + bcolors.ENDC)
        rui_id = input("请输入电机ID(左1右2): ")
        rui_id = rui_id.split(" ")

        if rui_id[0] == '1' or rui_id[0] == '2':
            pass
        else:
            print("无效的选项编号，请重新输入")
            exit()

        if len(rui_id) == 1:
            rui_id[0] = int(rui_id[0])
            rui_id.append(3)
        elif len(rui_id) == 2:
            if rui_id[0] == '1' or rui_id[0] == '2':
                pass
            else:
                print("无效的选项编号，请重新输入")
                exit()
            rui_id[0] = int(rui_id[0])
            rui_id[1] = int(rui_id[1])
        else:
            print("无效的选项编号，请重新输入")
            exit()

        new_id,old_id = rui_id
        print("新电机ID：" + str(new_id),"原电机ID：" + str(old_id), "修改ID后需重新上电")
        ruierman.ruierman_setId(new_id,old_id)
        print(bcolors.HEADER + "###结束，设置电机ID。请拔插电机电源重新上电###" + bcolors.ENDC)
    elif option == "5":
        print(bcolors.HEADER + "###开始，设置电机零点###" + bcolors.ENDC)
        ruierman.ruierman_setZero()
        print(bcolors.HEADER + "###结束，设置电机零点###" + bcolors.ENDC)
    elif option == "6":
        print(bcolors.HEADER + "###开始，电机测试###" + bcolors.ENDC)
        ruierman.ruierman_mov()
        print(bcolors.HEADER + "###结束，电机测试###" + bcolors.ENDC)
    elif option == "7":
        print(bcolors.HEADER + "###开始，测试舵机###" + bcolors.ENDC)
        dynamixel_servo.dxl_test()
        print(bcolors.HEADER + "###结束，测试舵机###" + bcolors.ENDC)
    elif option == "8":
        print(bcolors.HEADER + "###开始，舵机校准(每个舵机转动到限位，再转动到零点)###" + bcolors.ENDC)
        dxl_zero()
        print(bcolors.HEADER + "###结束，舵机校准###" + bcolors.ENDC)
    elif option == "9":
        print(bcolors.HEADER + "###开始，配置夹爪usb###" + bcolors.ENDC)
        usb_id = input("输入串口选项(1/2): ")
        if(usb_id == '1'):
            claw_usb(0)
        elif(usb_id == '2'):
            claw_usb(1)
        elif(usb_id == ''):
            claw_usb(0)
        else:
            print("无效的选项编号，请重新输入")
            exit()
        print(bcolors.HEADER + "###结束，配置夹爪usb###" + bcolors.ENDC)
    elif option == "a":
        print(bcolors.HEADER + "###开始，测试夹爪###" + bcolors.ENDC)
        claw_rs485.claw_test("/dev/claw_serial", 115200)
        print(bcolors.HEADER + "###结束，测试夹爪###" + bcolors.ENDC)
    elif option == "b":
        print(bcolors.HEADER + "###开始，配置强脑手usb###" + bcolors.ENDC)
        hand_usb()
        print(bcolors.HEADER + "###结束，配置强脑手usb###" + bcolors.ENDC)
    elif option == "c":
        print(bcolors.HEADER + "###开始，测试强脑手###" + bcolors.ENDC)
        print(bcolors.OKCYAN + "先左右手一起握，然后依次握左手，握右手" + bcolors.ENDC)
        qiangnao_hand()
        print(bcolors.HEADER + "###结束，测试强脑手###" + bcolors.ENDC)  
    elif option == "d":
        print(bcolors.HEADER + "###开始，瑞沃电机设置零点###" + bcolors.ENDC)
        ruiwo_zero()
        print(bcolors.HEADER + "###结束，瑞沃电机设置零点###" + bcolors.ENDC)
    elif option == "e":
        print(bcolors.HEADER + "###开始，瑞沃电机辨识方向###" + bcolors.ENDC)
        ruiwo_negtive()
        print(bcolors.HEADER + "###结束，瑞沃电机辨识方向###" + bcolors.ENDC)
    elif option == "f":
        print(bcolors.HEADER + "###开始，文件备份###" + bcolors.ENDC)
        folder_backup()
        print(bcolors.HEADER + "###结束，文件备份###" + bcolors.ENDC)
    elif option == "g":
        print(bcolors.HEADER + "###开始，遥控器配置usb###" + bcolors.ENDC)
        control_H12()
        print(bcolors.HEADER + "###结束，遥控器配置usb###" + bcolors.ENDC)
    elif option == "h":
        print(bcolors.HEADER + "###开始，新IMU模块配置usb###" + bcolors.ENDC)
        change_imu_usb()
        print(bcolors.HEADER + "###结束，新IMU模块配置usb###" + bcolors.ENDC)
    elif option == "i":
        print(bcolors.HEADER + "###开始，电机零点数据转换###" + bcolors.ENDC)
        elmo_position_read()
        print(bcolors.HEADER + "###结束，电机零点数据转换###" + bcolors.ENDC)
    elif option == "k":
        print(bcolors.HEADER + "###开始，更新当前目录程序###" + bcolors.ENDC)
        update_kuavo()
        print(bcolors.HEADER + "###结束，更新当前目录程序###" + bcolors.ENDC)
    elif option == "r":
        print(bcolors.HEADER + "###开始，更新kuavo_opensource程序###" + bcolors.ENDC)
        update_kuavo_opensource()
        print(bcolors.HEADER + "###结束，更新kuavo_opensource程序###" + bcolors.ENDC)
    elif option == "m":
        print(bcolors.HEADER + "###开始，编译kuavo_opensource程序###" + bcolors.ENDC)
        compile_kuavo_opensource()
        print(bcolors.OKCYAN + "如果编译错误请运行：" + bcolors.ENDC)
        print(bcolors.OKCYAN + "sudo apt-get install -y protobuf-c-compiler libprotobuf-c-dev" + bcolors.ENDC)
        print(bcolors.HEADER + "###结束，编译kuavo_opensource程序###" + bcolors.ENDC)
    elif option == "n":
        print(bcolors.HEADER + "###开始，编译当前目录程序###" + bcolors.ENDC)
        compile_kuavo()
        print(bcolors.OKCYAN + "如果编译错误请运行：" + bcolors.ENDC)
        print(bcolors.OKCYAN + "sudo apt-get install -y protobuf-c-compiler libprotobuf-c-dev" + bcolors.ENDC)
        print(bcolors.HEADER + "###结束，编译当前目录程序###" + bcolors.ENDC)
    elif option == "l":
        print(bcolors.HEADER + "###开始，license导入###" + bcolors.ENDC)
        license_sign()
        print(bcolors.HEADER + "###结束，license已导入，请确认验证###" + bcolors.ENDC)       
    else:
        print("无效的选项编号，请重新输入")


