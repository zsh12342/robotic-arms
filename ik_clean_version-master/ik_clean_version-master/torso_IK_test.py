import sys
import os
import time
import math
# sys.path.append("/usr/lib/python3/dist-packages")
import numpy as np
import pydrake

from pydrake.all import (
    AddMultibodyPlantSceneGraph, DiagramBuilder, Parser,PiecewisePolynomial
)
from pydrake.all import StartMeshcat, AddMultibodyPlantSceneGraph, MeshcatVisualizer

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
        return r_hand_in_base

if __name__ == "__main__":
    test = np.load("./rosbag_joint.npy") # drake RPY版本
    # test = np.load("./rosbag_s.npy") # 四元数版本（x,y,z,w）
    np.set_printoptions(linewidth=240)
    np.set_printoptions(threshold=2000)
    np.set_printoptions(precision=4)
    np.set_printoptions(suppress=True)

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
    l_pose = arm_ik.left_hand_pose(q0)
    print(f"left_hand_pose: {l_pose.translation()}, {l_pose.rotation()}")
    for i in range(250):
        # l_hand_pose = [0.1, 0.3, 0.55]
        l_hand_pose = test[i,0:3] # 不计算就给None
        # l_hand_RPY = test[i,3:6]
        l_hand_RPY = test[i,3:6]
        # quat = test[i,3:7]
        # l_hand_RPY = R.from_quat([quat[3], quat[0], quat[1], quat[2]]).as_euler('xyz', degrees=False)
        # print(f"l_hand matrix: {l_hand_RPY}")
        # l_hand_RPY = [1.57/2.0, -1.57, 0]
        # l_hand_RPY = [1.57/2.0, -1
        # print(f"l_hand_pose: {l_hand_pose}, l_hand_RPY: {l_hand_RPY}")
        r_hand_RPY = None
        # r_hand_RPY = [0, -1.0, 0]
        # x = 0.1 #+ math.sin(i*0.1)*0.1
        # y = -0.3 + math.sin(i*0.1)*0.1
        # z = 0.0 + math.sin(i*0.1)*0.1
        r_hand_pose = None#[x, y, z]
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
    while True:
        time.sleep(0.01)
