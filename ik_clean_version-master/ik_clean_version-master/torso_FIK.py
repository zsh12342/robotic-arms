import sys
import os
import re
import time
import math
# sys.path.append("/usr/lib/python3/dist-packages")
import numpy as np
import pydrake
from pydrake.math import RigidTransform
import matplotlib.pyplot as plt

from pydrake.all import (
    AddMultibodyPlantSceneGraph, DiagramBuilder, Parser,PiecewisePolynomial
)
from pydrake.all import StartMeshcat, AddMultibodyPlantSceneGraph, MeshcatVisualizer, RollPitchYaw

sys.path.append(os.path.abspath(os.path.dirname(__file__) + r'../../../'))
from scripts.common.IK import *
from scripts.common.utils import *

from scipy.spatial.transform import Rotation as R

class ArmIk:
    def __init__(self,model_file, end_frames_name, meshcat):
        builder = DiagramBuilder()
        self.__plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
        parser = Parser(self.__plant)
        robot = parser.AddModelFromFile(model_file)
        self.__plant.Finalize()

        self.__visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
        self.__diagram = builder.Build()
        self.__diagram_context = self.__diagram.CreateDefaultContext()

        self.__plant_context = self.__plant.GetMyContextFromRoot(self.__diagram_context)
        self.__q0 = self.__plant.GetPositions(self.__plant_context)
        self.__v0 = self.__plant.GetVelocities(self.__plant_context)
        self.__r0 = self.__plant.CalcCenterOfMassPositionInWorld(self.__plant_context)

        self.__base_link_name = end_frames_name[0]
        self.__left_eef_name = end_frames_name[1]
        self.__right_eef_name = end_frames_name[2]

        self.__IK = TorsoIK(self.__plant, end_frames_name, 1e-4, 1e-4)
    
    def q0(self):
        return self.__q0
    
    def init_state(self, torso_yaw_deg, torso_height):
        self.__torso_yaw_rad = math.radians(torso_yaw_deg)       
        self.__torso_height = torso_height       
        self.__q0[6] = torso_height

    def computeIK(self, q0, l_hand_pose, r_hand_pose, l_hand_RPY=None, r_hand_RPY=None):
            torsoR = [0.0, self.__torso_yaw_rad, 0.0]
            r = [0.0, 0.0, self.__torso_height]
            
            pose_list = [
                [torsoR, r],
                [l_hand_RPY, l_hand_pose],
                [r_hand_RPY, r_hand_pose],
            ]
            is_success, q = self.__IK.solve(pose_list, q0=q0)
            if not is_success:
                print(f"pose: {pose_list[0][0]}, {pose_list[0][1]}")
                print(f"lhand: {pose_list[1][0]}, {pose_list[1][1]}")
                print(f"rhand: {pose_list[2][0]}, {pose_list[2][1]}")
                # raise RuntimeError("Failed to IK0!")
                return None
            else:
                return q 

    def start_recording(self):
        self.__visualizer.StartRecording()

    def stop_andpublish_recording(self):
        self.__visualizer.StopRecording()
        self.__visualizer.PublishRecording()

    def visualize_animation(self, q_list, start_time=0.0, duration=1.1):
        t_sol = np.arange(start_time, start_time+duration, 1)  
        q_sol = np.array(q_list).T
        # print(f"q_sol: {q_sol.shape}, t_sol: {t_sol.shape}")
        q_pp = PiecewisePolynomial.FirstOrderHold(t_sol, q_sol)
        t0 = t_sol[0]
        tf = t_sol[-1]
        t = t0
        # self.__visualizer.StartRecording()
        while t < tf:
            q = q_pp.value(t)
            self.__plant.SetPositions(self.__plant_context, q)
            self.__diagram_context.SetTime(t)
            self.__diagram.ForcedPublish(self.__diagram_context)
            t += 0.01
        # self.__visualizer.StopRecording()
        # self.__visualizer.PublishRecording()
        # while True:
        time.sleep(0.1)
    
    def left_hand_jacobian(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        J_hand_in_world = self.__plant.CalcJacobianSpatialVelocity(
            self.__plant_context, JacobianWrtVariable.kV,
            self.__plant.GetFrameByName(self.__left_eef_name), [0, 0, 0], self.__plant.world_frame(), self.__plant.world_frame())
        return J_hand_in_world[:, 6:13]

    def right_hand_jacobian(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        J_hand_in_world = self.__plant.CalcJacobianSpatialVelocity(
            self.__plant_context, JacobianWrtVariable.kV,
            self.__plant.GetFrameByName(self.__right_eef_name), [0, 0, 0], self.__plant.world_frame(), self.__plant.world_frame())
        return J_hand_in_world[:, -7:]
    
    def left_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        l_hand_in_base = self.__plant.GetFrameByName(self.__left_eef_name).CalcPose(self.__plant_context, self.__plant.GetFrameByName(self.__base_link_name))
        print("left hand position in base:", l_hand_in_base.translation())
        return l_hand_in_base
    
    def right_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        r_hand_in_base = self.__plant.GetFrameByName(self.__right_eef_name).CalcPose(self.__plant_context, self.__plant.GetFrameByName(self.__base_link_name))
        print("left hand position in base:", r_hand_in_base.translation())
        print("left hand rotation in base:", r_hand_in_base.rotation())
        return r_hand_in_base
    
    def parse_data_line(line):
        # 使用改进的正则表达式匹配所有的浮点数或整数
        numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?', line)        # 将字符串转换为浮点数列表
        numbers = [float(num) for num in numbers]
        # 分割前7位和后1位
        return numbers[:-1], numbers[-1]

    def processed_files(source_directory, target_directory, qall, xyzrpy_values):
        # 如果目标目录不存在，则创建它
        os.makedirs(target_directory, exist_ok=True)
        
        for root, dirs, files in os.walk(source_directory):
            for file in files:
                if file.endswith('ed_1.txt'):
                    file_path = os.path.join(root, file)
                    # 构造目标文件的相对路径，并拼接到目标目录
                    relative_path = os.path.relpath(root, source_directory)
                    target_file_dir = os.path.join(target_directory, relative_path)
                    target_file_path = os.path.join(target_file_dir, file)
                    
                    # 确保目标文件的目录存在
                    os.makedirs(os.path.dirname(target_file_path), exist_ok=True)
                    with open(file_path, 'r') as f, open(target_file_path, 'w') as fout:
                        print(f"Processing file: {file_path} -> {target_file_path}")
                        for line in f:
                            if line.strip():
                                data_parts, last_part = ArmIk.parse_data_line(line)
                                print("data_parts:", data_parts, "last_part", last_part)
                                # 将前7位数据和最后一列拼接，然后写入新文件
                                qall = np.zeros(21)
                                qall[0] = 1
                                qall[14:21] = np.radians(data_parts)
                                r_pose = arm_ik.right_hand_pose(qall)
                                rpy = RollPitchYaw(r_pose.rotation()).vector()
                                if xyzrpy_values:
                                    previous_rpy = rpy_values[-1]
                                    for i in range(3):
                                        delta = rpy[i] - previous_rpy[i]
                                        if delta > np.pi:
                                            rpy[i] -= 2 * np.pi
                                        elif delta < -np.pi:
                                            rpy[i] += 2 * np.pi
                                # rotation = R.from_matrix(rotation_matrix)
                                print(f"rpy: {rpy}")
                                xyzrpy = np.concatenate((r_pose.translation(), rpy))
                                xyzrpy_values.append(xyzrpy)  # 记录 xyzrpy 值
                                new_line = f"[{', '.join(map(str, xyzrpy + [last_part]))}, 0]"
                                fout.write(new_line + '\n')

if __name__ == "__main__":
    test = np.load("./rosbag_joint.npy") # drake RPY版本
    source_directory = '/media/u20-2/Ventoy/1720580716.1550224_Data/command'
    target_directory = '/media/u20-2/Ventoy/Dataset_0710_edEEF'
    # test = np.load("./rosbag_s.npy") # 四元数版本（x,y,z,w）
    np.set_printoptions(linewidth=240)
    np.set_printoptions(threshold=2000)
    np.set_printoptions(precision=4)
    np.set_printoptions(suppress=True)
    rpy_values = []

    # 调用函数，传入你想要搜索的目录

    meshcat = StartMeshcat()
    # model_file = "robots/biped_gen4.0/urdf/biped_v3_arm_only.urdf"
    model_file = "robots/biped_s4/urdf/biped_s4.urdf"

    base_link_name = 'torso'
    end_frames_name = [base_link_name,'l_hand_roll','r_hand_roll']

    arm_ik = ArmIk(model_file, end_frames_name, meshcat)
    torso_yaw_deg = 0.0
    torso_height = 0.0
    arm_ik.init_state(torso_yaw_deg, torso_height)
    q0 = arm_ik.q0()
    q_list = [q0]
    last_q = q0
    arm_ik.start_recording()
    t = 0.0
    l_pose = arm_ik.right_hand_pose(q0)
    ArmIk.processed_files(source_directory, target_directory, q0, rpy_values)
    rpy_values = np.array(rpy_values)
    print(f"The size of rpy_values is: {rpy_values.size}")
    print(f"left_hand_pose: {l_pose.translation()}, {l_pose.rotation()}")
    for i in range(2000):
        r_hand_pose = rpy_values[i,0:3] # 不计算就给None
        r_hand_RPY = rpy_values[i,3:6]
        l_hand_RPY = None
        l_hand_pose = None#[x, y, z]
        time_0 = time.time()
        q = arm_ik.computeIK(q0, l_hand_pose, r_hand_pose, l_hand_RPY, r_hand_RPY)
        time_cost = time.time() - time_0
        print(f"time cost: {1e3*time_cost:.3f} ms")
        if q is not None:
            q_list.append(q)
            q0 = q
            # animate trajectory
            arm_ik.visualize_animation([last_q, q], t)
            last_q = q
            t = t + 1.0
        else:
            print(f"Failed to IK in step {i}!")
        print(f"i: {i}")
    arm_ik.stop_andpublish_recording()
    print('Program end, Press Ctrl + C to exit.')
    plt.figure()
    plt.plot(q_list[:, 20], label='j7')
    plt.plot(q_list[:, 19], label='j6')
    plt.plot(q_list[:, 18], label='j5')
    plt.plot(q_list[:, 17], label='j4')
    plt.plot(q_list[:, 16], label='j3')
    plt.plot(q_list[:, 15], label='j2')
    plt.legend()
    plt.xlabel('Index')
    plt.ylabel('Radians')
    plt.title('jnt values over time')
    plt.show()

    print('Program end, Press Ctrl + C to exit.')
    plt.figure()
    plt.plot(rpy_values[:, 0], label='Roll')
    plt.plot(rpy_values[:, 1], label='Pitch')
    plt.plot(rpy_values[:, 2], label='Yaw')
    plt.legend()
    plt.xlabel('Index')
    plt.ylabel('Radians')
    plt.title('RPY values over time')
    plt.show()
    while True:
        time.sleep(0.01)
