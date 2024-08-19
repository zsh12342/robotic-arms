#!/usr/bin/env python
# coding=utf-8
'''
Author: dongdongmingming
Date: 2024-04-12 15:35:06
LastEditors: Please set LastEditors
LastEditTime: 2024-04-20 14:55:54
FilePath: /kuavo/tools/check_tool/dynamixel_servo.py
Description: 舵机测试程序
'''

import sys, tty, termios
import select

sys.path.append('/home/lab/.local/lib/python3.8/site-packages/')

from dynamixel_sdk import *                 # Uses Dynamixel SDK library


def getch_wait():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def getch_goon():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                ch = sys.stdin.read(1)
                return ch
            else:
                return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)




# Protocol version
PROTOCOL_VERSION        = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE                = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME              = '/dev/usb_servo'    # Check which port is being used on your controller
                                            # ex) Windows: "COM1"   Linux: "/dev/usb_servo" Mac: "/dev/tty.usbserial-*"

# Control table address
ADDR_MX_TORQUE_ENABLE       = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36

# Data Byte Length
LEN_MX_PRESENT_POSITION     = 2

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque




class DXL_servo:
    def __init__(self, port, baur):
        self.port = port
        self.baur = baur
        
        self.recognize_id = []
        self.missing_ids = []
        self.ID_list = [17, 18, 19 , 24, 25, 26]                 # Dynamixel ID 
        self.presentPosition = [0,0,0,0,0,0,0,0,0,0]

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(port)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Initialize GroupBulkRead instace for Present Position
        self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)


        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch_wait()
            quit()


        # Set port baudrate
        if self.portHandler.setBaudRate(baur):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch_wait()
            quit()


    def ping_servo(self):
        for dxl_id in self.ID_list:
            # Try to ping the Dynamixel
            # Get Dynamixel model number
            dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, dxl_id)
            if dxl_comm_result != COMM_SUCCESS:
                print("舵机未找到 %d  %s " % (dxl_id, self.packetHandler.getTxRxResult(dxl_comm_result)))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                self.recognize_id.append(dxl_id)
                print("识别到舵机 [ID:%03d] ping Succeeded. Dynamixel model number : %d" % (dxl_id, dxl_model_number))
            self.missing_ids = [id for id in self.ID_list if id not in self.recognize_id]
        
    def unlock_servo(self):
        for dxl_id in self.recognize_id:
            # Enable Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("舵机ID %d 处于解锁状态，可掰动舵机查看舵机角度值" % dxl_id)


    def lock_servo(self):
        for dxl_id in self.recognize_id:
            # Enable Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("舵机ID %d 加锁，掰动确认舵机是否处于加锁状态" % dxl_id)

    def bulkread(self):
        for i in self.recognize_id:
            # Add Dynamixel#1 goal position value to the Bulkread list
            dxl_addparam_result = self.groupBulkRead.addParam(i, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkRead addparam failed" % i)
                quit()
        while True:
            dxl_comm_result = self.groupBulkRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            for j in self.recognize_id:
                # Check if groupbulkread data of Dynamixel#1 is available
                dxl_getdata_result = self.groupBulkRead.isAvailable(j, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
                if dxl_getdata_result != True:
                    print("[ID:%03d] groupBulkRead getdata failed" % j)
                    # quit()
            for k in self.recognize_id:
                # Check if groupbulkread data of Dynamixel#2 is available
                self.presentPosition[k-17] = self.groupBulkRead.getData(k, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
            indices = [0, 1, 2, 7, 8, 9]
            temp_positions = [self.presentPosition[i] for i in indices]
            formatted_positions = [str(position).ljust(5) for position in temp_positions]
            print(formatted_positions)
            # 获取按键值
            key = getch_goon()

            # 打印按键值
            if key == 'q':
                break
            elif key is not None:
                break


def dxl_test():

    dynamixel_servo = DXL_servo(DEVICENAME,BAUDRATE)

    dynamixel_servo.ping_servo()
    print("识别到舵机 ", dynamixel_servo.recognize_id, "未识别到舵机 ", dynamixel_servo.missing_ids)
    print("按任意键继续...")
    getch_wait()

    dynamixel_servo.lock_servo()
    print("按任意键继续...")
    getch_wait()
    
    dynamixel_servo.unlock_servo()
    print("按任意键继续...")
    getch_wait()
    dynamixel_servo.bulkread()
    
    # Close port
    dynamixel_servo.portHandler.closePort()

if __name__ == '__main__':
    dynamixel_servo = DXL_servo(DEVICENAME,BAUDRATE)

    dynamixel_servo.ping_servo()
    print("识别到舵机 ", dynamixel_servo.recognize_id, "未识别到舵机 ", dynamixel_servo.missing_ids)
    print("按任意键继续...")
    getch_wait()

    dynamixel_servo.lock_servo()
    print("按任意键继续...")
    getch_wait()
    
    dynamixel_servo.unlock_servo()
    print("按任意键继续...")
    getch_wait()
    dynamixel_servo.bulkread()

    # Close port
    dynamixel_servo.portHandler.closePort()


