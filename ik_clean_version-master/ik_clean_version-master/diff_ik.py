import time
import math

import numpy as np
from IPython.display import clear_output
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    JacobianWrtVariable,
    MathematicalProgram,
    MeshcatVisualizer,
    PiecewisePolynomial,
    PiecewisePose,
    RigidTransform,
    RotationMatrix,
    Solve,
    StartMeshcat,
    Quaternion,
    RollPitchYaw,
    Parser,
)
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as R

class DiffIK:
    def __init__(self, model_file, end_frames_name, meshcat=None):
        builder = DiagramBuilder()
        self.__plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
        parser = Parser(self.__plant)
        robot = parser.AddModelFromFile(model_file)
        self.__plant.Finalize()

        if meshcat is not None:
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
    
    def q0(self):
        return self.__q0
    
        
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
        return l_hand_in_base
    
    def right_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        r_hand_in_base = self.__plant.GetFrameByName(self.__right_eef_name).CalcPose(self.__plant_context, self.__plant.GetFrameByName(self.__base_link_name))
        return r_hand_in_base

    def solve_left_hand(self, q, V_G_desired, h=0.01, v_max=3.0):
        J_l = self.left_hand_jacobian(q)
        prog = self.MakeMathematicalProgram(q, J_l, V_G_desired, h, v_max)
        result = Solve(prog)
        return np.asarray(result.GetSolution())

    '''
    This is a simple diff IK by QP for robots with 7 joints.
        @q: current joint angles
        @J_G: Jacobian matrix of the end-effector w.r.t. the joint angles
        @v_Gdesired: desired end-effector position and orientation (r, p, y, dx, dy, dz)
        @h: time step/(s)
    '''
    @staticmethod
    def MakeMathematicalProgram(q, J_G, v_Gdesired, h, v_max):
        prog = MathematicalProgram()
        v = prog.NewContinuousVariables(7, "v")
        # v_max = 3.0

        error = J_G @ v - np.asarray(v_Gdesired)
        prog.AddCost(error.dot(error))
        prog.AddBoundingBoxConstraint(-v_max, v_max, v)

        return prog

    def start_recording(self):
        if meshcat is None:
            return
        self.__visualizer.StartRecording()

    def stop_andpublish_recording(self):
        if meshcat is None:
            return
        print("Publish recording...")
        self.__visualizer.StopRecording()
        self.__visualizer.PublishRecording()

    def visualize_animation(self, q_list, start_time=0.0, duration=1.1):
        if meshcat is None:
            return
        t_sol = np.array([start_time, start_time + duration])  
        q_sol = np.array(q_list).T
        q_pp = PiecewisePolynomial.FirstOrderHold(t_sol, q_sol)
        t0 = t_sol[0]
        tf = t_sol[-1]
        t = t0
        while t < tf:
            q = q_pp.value(t)
            self.__plant.SetPositions(self.__plant_context, q)
            self.__diagram_context.SetTime(t)
            self.__diagram.ForcedPublish(self.__diagram_context)
            t += duration/2.0
        # time.sleep(0.1)

# pose: tuple(pos, ori(x,y,z,w))
def interpolate_pose(pose1, pose2, t0, dt):
    norm = np.linalg.norm(pose2[0] - pose1[0])
    norm_ori = np.linalg.norm(pose2[1] - pose1[1])
    # print(f"norm: {norm}")
    # print(f"norm_ori: {norm_ori}")
    num =int((norm + norm_ori)/0.001)
    num = max(2, min(100, num))
    # print(f"num: {num}")
    quat = pose1[1]
    mat1 = R.from_quat([quat[0], quat[1], quat[2], quat[3]]).as_matrix()
    quat = pose2[1]
    mat2 = R.from_quat([quat[0], quat[1], quat[2], quat[3]]).as_matrix()
    X1 = RigidTransform(RotationMatrix(mat1), pose1[0])
    X2 = RigidTransform(RotationMatrix(mat2), pose2[0])
    return PiecewisePose.MakeLinear([t0, t0+num*dt], [X1, X2])

def control_to_pos(diff_ik: DiffIK, q_now, traj, dt=0.01):
    t_duration = traj.get_segment_times()[1] - traj.get_segment_times()[0]
    print(f"t_duration: {t_duration}")
    t_sim = traj.get_segment_times()[0]
    q0 = q_now
    last_q = q_now
    # diff_ik.start_recording()
    traj_V_G = traj.MakeDerivative()

    t_play = 0.0
    while t_sim < t_duration:
        t_sim += dt
        l_pose = diff_ik.left_hand_pose(q0)
        time_0 = time.time()
        V_G = traj_V_G.value(t_sim)
        V_G_vec = np.array(V_G)[:, 0]
        # print(f"t_sim: {t_sim}, V_G: {V_G_vec}")
        v_left = diff_ik.solve_left_hand(q0, V_G_vec, dt)
        # print(f"v_left: {v_left}")
        q0[7:14] += dt*v_left
        time_cost = time.time() - time_0
        # print(f"time cost: {1e3*time_cost:.3f} ms")
        t_record.append(1e3*time_cost)
        # animate trajectory
        diff_ik.visualize_animation([last_q, q0], t_play, dt)
        last_q = q0
        t_play = t_play + dt
    
    l_pose = diff_ik.left_hand_pose(q0)
    # print(f"left_hand_pose end: {l_pose.translation()}, {l_pose.rotation().ToRollPitchYaw().vector()}")
    return last_q, l_pose.translation(), l_pose.rotation().ToRollPitchYaw().vector()


def draw_traj(traj_X_G):
    traj_p_G = traj_X_G.get_position_trajectory()
    traj_R_G = traj_X_G.get_orientation_trajectory()
    p_G = traj_p_G.vector_values(traj_p_G.get_segment_times())
    R_G = traj_R_G.vector_values(traj_R_G.get_segment_times())
    print(traj_p_G.get_segment_times())
    plt.plot(traj_p_G.get_segment_times(), p_G.T)
    t_interp = 0.4
    plt.plot(t_interp, traj_p_G.value(t_interp)[0], 'x')
    plt.plot(t_interp, traj_p_G.value(t_interp)[1], 'x')
    plt.plot(t_interp, traj_p_G.value(t_interp)[2], 'x')
    # plt.legend(["x", "y", "z"])
    plt.title("p_G")
    plt.show()

    plt.plot(traj_R_G.get_segment_times(), R_G.T)
    plt.legend(["qx", "qy", "qz", "qw"])
    plt.title("R_G")
    plt.show()

    traj_v_G = traj_p_G.MakeDerivative()
    v_G = traj_v_G.vector_values(traj_v_G.get_segment_times())
    plt.plot(traj_v_G.get_segment_times(), v_G.T)
    plt.legend(["vx", "vy", "vz"])
    plt.title("v_G")
    plt.show()

    traj_V_G = traj_X_G.MakeDerivative()
    V_G = traj_V_G.vector_values(traj_V_G.get_segment_times())
    plt.plot(traj_V_G.get_segment_times(), V_G.T)
    plt.legend(["vr", "vp", "vy", "vx", "vy", "vz"])
    plt.title("V_G")
    plt.show()

def draw_real_traj(x_record, pos_record, ori_record):
    pos_record = np.array(pos_record)
    plt.subplot(2, 3, 1)
    plt.plot(x_record, 1e3*pos_record[:, 0])
    plt.title('p_x')
    plt.ylabel('mm')
    plt.subplot(2, 3, 2)
    plt.plot(x_record, 1e3*pos_record[:, 1])
    plt.title('p_y')
    plt.ylabel('mm')
    plt.subplot(2, 3, 3)
    plt.plot(x_record, 1e3*pos_record[:, 2])
    plt.title('p_z')
    plt.ylabel('mm')

    ori_record = np.array(ori_record)
    RadToDeg = 180.0/math.pi
    plt.subplot(2, 3, 4)
    plt.plot(x_record, RadToDeg*ori_record[:, 0])
    plt.title('roll')
    plt.ylabel('deg')
    plt.subplot(2, 3, 5)
    plt.plot(x_record, RadToDeg*ori_record[:, 1])
    plt.title('pitch')
    plt.ylabel('deg')
    plt.subplot(2, 3, 6)
    plt.plot(x_record, RadToDeg*ori_record[:, 2])
    plt.title('yaw')
    plt.ylabel('deg')

    # plt.show()


if __name__ == "__main__":
    # test = np.load("./rosbag_joint.npy") # drake RPY版本
    test = np.load("./rosbag_s.npy") # quat版本
    np.set_printoptions(linewidth=240)
    np.set_printoptions(threshold=2000)
    np.set_printoptions(precision=4)
    np.set_printoptions(suppress=True)

    meshcat = StartMeshcat()
    model_file = "robots/biped_s4/urdf/biped_s4.urdf"

    end_frames_name = ['torso','l_hand_roll','r_hand_roll']

    diff_ik = DiffIK(model_file, end_frames_name, meshcat)
    q0 = diff_ik.q0()
    last_q = q0
    diff_ik.start_recording()
    t = 0.0
    l_pose_0 = diff_ik.left_hand_pose(q0)
    q0[7:14] = [0.1084,  0.0478 , 0.1954 ,-0.0801 , 0.1966 ,-0.5861 , 0.0755]
    q_start = q0
    
    def get_pos(record_data, idx):
        return (record_data[idx, 0:3], record_data[idx, 3:7])

    diff_ik.start_recording()

        
    x_record = []
    pos_record = []
    ori_record = []
    for i in range(250):
        quat = test[i, 3:7]
        quat_drake = Quaternion(quat[3], quat[0], quat[1], quat[2])
        rpy = RollPitchYaw(quat_drake).vector()

        ori_record.append(rpy)
        pos_record.append(test[i, 0:3])
        x_record.append(i)
    draw_real_traj(x_record, pos_record, ori_record)

    x_record = []
    t_record = []
    pos_record = []
    ori_record = []
    controller_dt = 0.01
    for i in range(250-1): 
        traj_X_G = interpolate_pose(get_pos(test, i), get_pos(test, i+1), 0, controller_dt)
        pos_err = np.linalg.norm(get_pos(test, i)[0] - get_pos(test, i+1)[0])
        print(f"pos_err start: {pos_err}")
        # draw_traj(traj_X_G)

        time_0 = time.time()
        q_now, pos, rpy = control_to_pos(diff_ik, q_start, traj_X_G)
        q_start = q_now
        time_cost = time.time() - time_0
        print(f"time cost: {1e3*time_cost:.3f} ms")

        x_record.append(i)
        pos_record.append(pos)
        ori_record.append(rpy)
        t_record.append(1e3*time_cost)
        pos_err = np.linalg.norm(pos - get_pos(test, i+1)[0])
        print(f"pos_err end: {pos_err}")
    
    diff_ik.stop_andpublish_recording()
    print('Program end, Press Ctrl + C to exit.')
    draw_real_traj(x_record, pos_record, ori_record)
    plt.show()

    # plt.plot(x_record, np.array(t_record))
    # plt.title('time cost')
    # plt.ylabel('ms')
    
    while True:
        time.sleep(0.01)


def test_diff_ik():
    # test = np.load("./rosbag_s.npy") # 四元数版本（x,y,z,w）
    np.set_printoptions(linewidth=240)
    np.set_printoptions(threshold=2000)
    np.set_printoptions(precision=4)
    np.set_printoptions(suppress=True)

    meshcat = None
    model_file = "robots/biped_s4/urdf/biped_s4.urdf"

    base_link_name = 'torso'
    end_frames_name = [base_link_name,'l_hand_roll','r_hand_roll']

    diff_ik = DiffIK(model_file, end_frames_name, meshcat)
    q0 = diff_ik.q0()
    q_list = [q0]
    last_q = q0
    # diff_ik.start_recording()
    t = 0.0
    pose_l0 = diff_ik.left_hand_pose(q0)
    # print(f"J size: {J_l.shape}")
    V_G_desired = np.array(
                [
                    0,      # rotation about x
                    -0.0,   # rotation about y
                    0,      # rotation about z
                    0,      # x
                    -0.05,  # y
                    -0.0,   # z
                ]
            )
    
    time_start = time.time()
    q_left = diff_ik.solve_left_hand(q0, V_G_desired)
    time_cost = time.time() - time_start
    print(f"time cost: {1e3*time_cost} ms")
    # print(f"result: {q_left}")
    q_result = q0
    q_result[7:14] += q_left
    pose_l = diff_ik.left_hand_pose(q_result)
    print(f"pose_l0: {pose_l0.translation()}")
    print(f"pose_l: {pose_l.translation()}")
    print(f"diff: {np.linalg.norm(pose_l.translation() - pose_l0.translation() - V_G_desired[-3:])}")
   