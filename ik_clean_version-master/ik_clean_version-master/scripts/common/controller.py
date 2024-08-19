
import time
import numpy as np
import pdb
from pydrake.multibody.plant import CoulombFriction, MultibodyPlant
from pydrake.systems.framework import LeafSystem
from pydrake.systems.framework import BasicVector
from pydrake.trajectories import PiecewisePolynomial
from pydrake.all import (JacobianWrtVariable, MathematicalProgram, Solve)
from pydrake.all import (LinearQuadraticRegulator,
                         LinearSystem,
                         FiniteHorizonLinearQuadraticRegulator,
                         FiniteHorizonLinearQuadraticRegulatorOptions)


class PidController(LeafSystem):
    def __init__(self, plant, kp, ki, kd, pub=False, nqf=7, nvf=6):
        LeafSystem.__init__(self)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pub = pub
        self.nq_f, self.nv_f = nqf, nvf
        self.plant = plant
        self.nq = self.plant.num_positions()
        self.nv = self.plant.num_velocities()
        self.na = self.plant.num_actuated_dofs()
        self.dt = plant.time_step()
        self.Ba = plant.MakeActuationMatrix()[self.nv_f:, :].T
        self.last_qv = np.zeros(self.nq + self.nv)
        self.last_qv_des = np.zeros(self.nq + self.nv)
        self.intergral = np.zeros(self.na)
        self.DeclareVectorInputPort("state", BasicVector(self.nq + self.nv))
        self.DeclareVectorInputPort("desire", BasicVector(self.nq + self.nv))
        self.DeclareVectorOutputPort("control", BasicVector(self.na), self.Output)

    def Output(self, context, output):
        qv = self.get_input_port(0).Eval(context)
        qv_des = self.get_input_port(1).Eval(context)
        q_err = qv_des[self.nq_f:self.nq] - qv[self.nq_f:self.nq]
        v_err = qv_des[self.nq + self.nv_f:] - qv[self.nq + self.nv_f:]
        self.intergral = self.intergral + q_err
        tau = self.kp * q_err + self.ki * self.intergral + self.kd * v_err
        output.SetFromVector(self.Ba.dot(tau))

        if self.pub:
            q = qv[0:self.nq]
            v = qv[self.nq:self.nq + self.nv]
            vdot = (qv[self.nq:self.nq + self.nv] - self.last_qv[self.nq:self.nq + self.nv]) / self.dt
            lcm_pub_state("state", q, v, vdot)
            q_des = qv_des[0:self.nq]
            v_des = qv_des[self.nq:self.nq + self.nv]
            vdot_des = (qv_des[self.nq:self.nq + self.nv] - self.last_qv_des[self.nq:self.nq + self.nv]) / self.dt
            lcm_pub_state("desire", q_des, v_des, vdot_des)
            lcm_pub_vector("desire/tau", tau)

        self.last_qv = qv
        self.last_qv_des = qv_des


class WholeBodyController(LeafSystem):
    def __init__(self, plant,
                 contacts_frame_name,
                 Q, R,
                 kp, kd,
                 w_V, w_qdd,
                 pub=False, nq_f=7, nv_f=6):
        LeafSystem.__init__(self)
        self.plant = plant
        self.plant_context = self.plant.CreateDefaultContext()
        self.nq = self.plant.num_positions()
        self.nv = self.plant.num_velocities()
        self.na = self.plant.num_actuated_dofs()
        self.dt = plant.time_step()
        self.nq_f, self.nv_f = nq_f, nv_f
        self.nq_com, self.nv_com = 3, 3
        self.contacts_frame_name = contacts_frame_name
        self.contacts_per_frame = contacts_frame_name
        self.pub = pub
        self.DeclareVectorInputPort("state", BasicVector(self.nq + self.nv))
        self.DeclareVectorInputPort("desire", BasicVector(self.nq + self.nv))
        self.DeclareVectorOutputPort("tau", BasicVector(self.plant.num_actuated_dofs()), self.Output)

        self.w_V = w_V
        self.w_qdd = w_qdd
        self.K_p = kp
        self.K_d = kd

        self.epsilon = 1.0e-8
        self.eta_min = -0.2
        self.eta_max = 0.2

        self.mu = 1.0  # Coefficient of friction
        self.N_d = 4  # friction cone approximated as a i-pyramid
        self.N_f = 3  # contact force dimension

        self.prev_v_des = [0] * self.nv
        self.prev_rd_des = [0] * self.nv_com
        self.prev_qv = np.zeros(self.nq + self.nv)

        self.is_wbc = True
        if self.is_wbc:
            self.DeclareVectorInputPort("com_des", BasicVector(self.nq_com + self.nv_com))

            self.x_size = self.nq_com + self.nv_com
            self.u_size = self.nv_com

            def LQR(Q, R):
                A = np.vstack([np.hstack([0 * np.identity(3), 1 * np.identity(3)]),
                               np.hstack([0 * np.identity(3), 0 * np.identity(3)])])
                B_1 = np.vstack([0 * np.identity(3),
                                1 * np.identity(3)])
                K, S = LinearQuadraticRegulator(A, B_1, Q, R)

                def V_full(x, u, r, rd, rdd):
                    x_bar = x - np.concatenate([r, rd])
                    u_bar = u - rdd
                    '''
                    xd_bar = d(x - [r, rd].T)/dt
                            = xd - [rd, rdd].T
                            = Ax + Bu - [rd, rdd].T
                    '''
                    xd_bar = A.dot(x) + B_1.dot(u) - np.concatenate([rd, rdd])
                    return x_bar.T.dot(Q).dot(x_bar) + u_bar.T.dot(R).dot(u_bar) + 2 * x_bar.T.dot(S).dot(xd_bar)
                self.V_full = V_full

            def tvLQR(com_traj, Q, R):
                A = np.vstack([np.hstack([np.zeros((3, 3)), np.identity(3)]), np.zeros((3, 6))])
                B = np.vstack([np.identity(3), np.zeros((3, 3))])
                C = np.identity(6)
                D = np.zeros((6, 3))
                particle_sys = LinearSystem(A, B, C, D)
                particle_context = particle_sys.CreateDefaultContext()

                x_dim = self.nq_com + self.nv_com
                u_dim = self.nv_com
                breaks = np.linspace(0, com_traj.shape[0] * self.dt, com_traj.shape[0])
                samples = np.zeros((x_dim, com_traj.shape[0]))
                for i in range(x_dim):
                    samples[i, :] = com_traj[:, i]
                x_com_pp = PiecewisePolynomial.CubicShapePreserving(breaks, samples, zero_end_point_derivatives=True)

                samples = np.zeros((u_dim, com_traj.shape[0]))
                for i in range(u_dim):
                    samples[i, :] = com_traj[:, x_dim + i]
                u_com_pp = PiecewisePolynomial.CubicShapePreserving(breaks, samples, zero_end_point_derivatives=True)

                '''
                y = [[] for i in range(x_dim)]
                label = []
                x = np.linspace(0, com_traj.shape[0]*self.dt, com_traj.shape[0])
                for i in range(com_traj.shape[0]):
                    for j in range(x_dim):
                        y[j].append(x_com_pp.value(i*self.dt)[j])
                for i in range(x_dim):
                    label.append('x{}'.format(i))
                plt.subplot(1,2,1)
                plot_scatter(x, y, label)

                y = [[] for i in range(u_dim)]
                label = []
                for i in range(com_traj.shape[0]):
                    for j in range(u_dim):
                        y[j].append(u_com_pp.value(i*self.dt)[j])
                for i in range(u_dim):
                    label.append('u{}'.format(i))
                plt.subplot(1,2,2)
                plot_scatter(x, y, label)
                plt.show()
                '''

                options = FiniteHorizonLinearQuadraticRegulatorOptions()
                options.Qf = Q
                options.x0 = x_com_pp
                options.u0 = u_com_pp
                self.result = FiniteHorizonLinearQuadraticRegulator(system=particle_sys, context=particle_context,
                                                                    t0=options.x0.start_time(),
                                                                    tf=options.x0.end_time(),
                                                                    Q=Q,
                                                                    R=R,
                                                                    options=options)

                def V_full(x, u, r, rd, rdd):
                    x_bar = (x - np.concatenate([r, rd])).reshape(1, -1).T
                    xd_bar = (A.dot(x) + B.dot(u) - np.concatenate([rd, rdd])).reshape(1, -1).T
                    S = self.result.S.value(self.__traj_index * self.dt)
                    sx = self.result.sx.value(self.__traj_index * self.dt)
                    '''
                    print('x_bar:{}\n'.format(x_bar))
                    print('x_bar_T:{}\n'.format(x_bar.T))
                    print('xd_bar:{}\n'.format(xd_bar))
                    print('S:{}\n'.format(S))
                    print('sx:{}\n'.format(sx))
                    print('1:{}\n'.format(x_bar.T.dot(Q).dot(x_bar)))
                    print('2:{}\n'.format((x_bar.T.dot(S.T+S)+sx.T).dot(xd_bar)))
                    print('2:{}\n'.format(x_bar.T.dot(Q).dot(x_bar) + (x_bar.T.dot(S.T+S)+sx.T).dot(xd_bar)))
                    '''
                    return (x_bar.T.dot(Q).dot(x_bar) + (x_bar.T.dot(S.T + S) + sx.T).dot(xd_bar))[0][0]
                self.V_full = V_full

            LQR(Q, R)
            # tvLQR(self.__com_traj, Q, R)
        else:
            g = 9.81
            z_com = 1.2
            zmp_state_size = 2

            # Only x, y coordinates of COM is considered
            com_dim = 2
            self.x_size = 2 * com_dim
            self.u_size = com_dim
            self.DeclareVectorInputPort("zmp_des", BasicVector(zmp_state_size))
            ''' Eq(1) '''
            A = np.vstack([np.hstack([0 * np.identity(com_dim), 1 * np.identity(com_dim)]),
                           np.hstack([0 * np.identity(com_dim), 0 * np.identity(com_dim)])])
            B_1 = np.vstack([0 * np.identity(com_dim),
                             1 * np.identity(com_dim)])

            ''' Eq(4) '''
            C_2 = np.hstack([np.identity(2), np.zeros((2, 2))])  # C in Eq(2)
            D = -z_com / g * np.identity(zmp_state_size)
            Q = 1.0 * np.identity(zmp_state_size)

            ''' Eq(6) '''
            '''
            y.T*Q*y
            = (C*x+D*u)*Q*(C*x+D*u).T
            = x.T*C.T*Q*C*x + u.T*D.T*Q*D*u + x.T*C.T*Q*D*u + u.T*D.T*Q*C*X
            = ..                            + 2*x.T*C.T*Q*D*u
            '''
            K, S = LinearQuadraticRegulator(A, B_1, C_2.T.dot(Q).dot(C_2), D.T.dot(Q).dot(D), C_2.T.dot(Q).dot(D))
            # Use original formulation

            def V_full(x, u, y_des):  # Assume constant z_com, we don't need tvLQR
                y = C_2.dot(x) + D.dot(u)

                def dJ_dx(x):
                    return x.T.dot(S.T + S)  # https://math.stackexchange.com/questions/20694/vector-derivative-w-r-t-its-transpose-fracdaxdxt
                y_bar = y - y_des
                x_bar = x - np.concatenate([y_des, [0.0, 0.0]])  # FIXME: This doesn't seem right...
                # FIXME: xd_bar should depend on yd_des
                xd_bar = A.dot(x_bar) + B_1.dot(u)
                return y_bar.T.dot(Q).dot(y_bar) + dJ_dx(x_bar).dot(xd_bar)
            self.V_full = V_full

        # Calculate values that don't depend on context
        self.B_7 = self.plant.MakeActuationMatrix()
        # From np.sort(np.nonzero(B_7)[0]) we know that indices 0-5 are the unactuated 6 DOF floating base and 6-35 are the actuated 30 DOF robot joints
        self.v_idx_act = 6  # Start index of actuated joints in generalized velocities
        self.B_a = self.B_7[self.v_idx_act:, :]
        self.B_a_inv = np.linalg.inv(self.B_a)

        # Sort joint effort limits to be the same order as tau in Eq(13)
        self.sorted_max_efforts = self.plant.GetEffortUpperLimits()

    def getActuatorIndex(self, plant, joint_name):
        return int(plant.GetJointActuatorByName(joint_name + "_motor").index())

    def getJointLimitsSortedByActuator(self, plant, joint_limits):
        return sorted(joint_limits.items(), key=lambda entry: self.getActuatorIndex(plant, entry[0]))

    def getJointIndexInGeneralizedPositions(self, plant, joint_name):
        return int(plant.GetJointByName(joint_name).position_start())

    def getJointIndexInGeneralizedVelocities(self, plant, joint_name):
        return self.getJointIndexInGeneralizedPositions(plant, joint_name) - 1

    def create_qp(self, plant_context, V, q_des, v_des, vd_des):
        # Determine contact points
        contact_positions_per_frame = {}
        active_contacts_per_frame = {}  # Note this should be in frame space
        for frame, contacts in self.contacts_per_frame.items():
            contact_positions = self.plant.CalcPointsPositions(plant_context, self.plant.GetFrameByName(frame),
                                                               contacts, self.plant.world_frame())
            active_contacts_per_frame[frame] = contacts[:, np.where(contact_positions[2, :] <= 1e-4)[0]]

        N_c = sum([active_contacts.shape[1] for active_contacts in active_contacts_per_frame.values()])  # num contact points
        if N_c == 0:
            print("Not in contact!")
            return None

        ''' Eq(7) '''
        H = self.plant.CalcMassMatrixViaInverseDynamics(plant_context)
        # Note that CalcGravityGeneralizedForces assumes the form Mv̇ + C(q, v)v = tau_g(q) + tau_app
        # while Eq(7) assumes gravity is accounted in C (on the left hand side)
        C_7 = self.plant.CalcBiasTerm(plant_context) - self.plant.CalcGravityGeneralizedForces(plant_context)
        B_7 = self.B_7

        # TODO: Double check
        Phi_foots = []
        for frame, active_contacts in active_contacts_per_frame.items():
            if active_contacts.size:
                Phi_foots.append(self.plant.CalcJacobianTranslationalVelocity(plant_context, JacobianWrtVariable.kV, self.plant.GetFrameByName(frame),
                                                                              active_contacts, self.plant.world_frame(), self.plant.world_frame()))
        Phi = np.vstack(Phi_foots)

        ''' Eq(8) '''
        v_idx_act = self.v_idx_act
        H_f = H[0:v_idx_act, :]
        H_a = H[v_idx_act:, :]
        C_f = C_7[0:v_idx_act]
        C_a = C_7[v_idx_act:]
        B_a = self.B_a
        Phi_f_T = Phi.T[0:v_idx_act:, :]
        Phi_a_T = Phi.T[v_idx_act:, :]

        ''' Eq(9) '''
        # Assume flat ground for now
        n = np.array([[0],
                      [0],
                      [1.0]])
        d = np.array([[1.0, -1.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0, -1.0],
                      [0.0, 0.0, 0.0, 0.0]])
        v = np.zeros((self.N_d, N_c, self.N_f))
        for i in range(self.N_d):
            for j in range(N_c):
                v[i, j] = (n + self.mu * d)[:, i]

        def tau(qdd, lambd):
            return self.B_a_inv.dot(H_a.dot(qdd) + C_a - Phi_a_T.dot(lambd))
        self.tau = tau

        ''' Quadratic Program I '''
        prog = MathematicalProgram()
        qdd = prog.NewContinuousVariables(self.nv, name="qdd")  # To ignore 6 DOF floating base
        self.qdd = qdd
        beta = prog.NewContinuousVariables(self.N_d, N_c, name="beta")
        self.beta = beta
        lambd = prog.NewContinuousVariables(self.N_f * N_c, name="lambda")
        self.lambd = lambd

        # Jacobians ignoring the 6DOF floating base
        J_foots = []
        for frame, active_contacts in active_contacts_per_frame.items():
            if active_contacts.size:
                num_active_contacts = active_contacts.shape[1]
                J_foot = np.zeros((self.N_f * num_active_contacts, self.nv))
                # TODO: Can this be simplified?
                for i in range(num_active_contacts):
                    J_foot[self.N_f * i:self.N_f * (i + 1), :] = self.plant.CalcJacobianSpatialVelocity(
                        plant_context, JacobianWrtVariable.kV, self.plant.GetFrameByName(frame),
                        active_contacts[:, i], self.plant.world_frame(), self.plant.world_frame())[3:]
                J_foots.append(J_foot)
        J = np.vstack(J_foots)
        assert(J.shape == (N_c * self.N_f, self.nv))

        eta = prog.NewContinuousVariables(J.shape[0], name="eta")
        self.eta = eta

        q = self.plant.GetPositions(plant_context)
        qd = self.plant.GetVelocities(plant_context)

        com = self.plant.CalcCenterOfMassPositionInWorld(plant_context)
        comv = self.plant.CalcJacobianCenterOfMassTranslationalVelocity(plant_context, JacobianWrtVariable.kV,
                                                                        self.plant.world_frame(), self.plant.world_frame()).dot(qd)

        # x = np.array([com[0], com[1], comv[0], comv[1]])
        x = np.concatenate((com, comv))
        self.x = x
        u = prog.NewContinuousVariables(self.u_size, name="u")  # x_com_dd, y_com_dd
        self.u = u

        ''' Eq(10) '''
        # Convert q, q_nom to generalized velocities form
        q_err = self.plant.MapQDotToVelocity(plant_context, q_des - q)
        # print(f"Pelvis error: {q_err[0:3]}")
        # FIXME: Not sure if it's a good idea to ignore the x, y, z position of pelvis
        frame_weights = np.ones((self.nv))
        # ignored_pose_indices = {3, 4, 5} # Ignore x position, y position
        ignored_pose_indices = {}  # Ignore x position, y position
        relevant_pose_indices = list(set(range(self.nv)) - set(ignored_pose_indices))
        qdd_ref = self.K_p * q_err + self.K_d * (v_des - qd) + vd_des  # Eq(27) of [1]
        # qdd_err = qdd - qdd_ref
        # qdd_err = qdd_err*frame_weights
        # qdd_err = qdd_err[relevant_pose_indices]

        self.cost_v = prog.AddCost((V(self.x, u)) * self.w_V)

        Q = np.identity((self.nv)) * self.w_qdd
        x_desired = qdd_ref
        vras = np.array(qdd)
        self.cost_qdd = prog.AddQuadraticErrorCost(Q, x_desired, vras)

        Q = np.identity((self.N_d * N_c)) * self.epsilon * 2.0
        b = np.zeros((self.N_d * N_c, 1))
        vras = np.array(beta.flatten())
        self.cost_epsilon = prog.AddQuadraticCost(Q, b, vras)

        Q = np.identity(eta.shape[0]) * 2.0
        b = np.zeros((eta.shape[0]))
        vras = np.array(eta)
        self.cost_eta = prog.AddQuadraticCost(Q, b, vras)

        ''' Eq(11) '''
        Aeq = np.hstack([H_f, -Phi_f_T])
        beq = -C_f
        vras = np.concatenate([qdd, lambd])
        prog.AddLinearEqualityConstraint(Aeq, beq, vras)

        ''' Eq(12) '''
        alpha = 0.1
        # TODO: Double check
        Jd_qd_foots = []
        for frame, active_contacts in active_contacts_per_frame.items():
            if active_contacts.size:
                Jd_qd_foot = self.plant.CalcBiasTranslationalAcceleration(plant_context, JacobianWrtVariable.kV, self.plant.GetFrameByName(frame),
                                                                          active_contacts, self.plant.world_frame(), self.plant.world_frame())
                Jd_qd_foots.append(Jd_qd_foot.flatten())
        Jd_qd = np.concatenate(Jd_qd_foots)
        assert(Jd_qd.shape == (N_c * 3,))
        Aeq = np.hstack([J, -1 * np.eye(N=J.shape[0])])
        beq = -alpha * J.dot(qd) - Jd_qd
        vras = np.concatenate([qdd, eta])
        # prog.AddLinearEqualityConstraint(Aeq, beq, vras)

        ''' Eq(13) '''
        A = np.hstack([H_a, Phi_a_T])
        lb = self.B_a.dot(-self.sorted_max_efforts) - C_a
        ub = self.B_a.dot(self.sorted_max_efforts) - C_a
        vars = np.concatenate([qdd, lambd])
        prog.AddLinearConstraint(A, lb, ub, vars)

        ''' Eq(14) '''
        for j in range(N_c):
            Aeq = np.hstack([v[:, j].T, np.diag([-1, -1, -1])])
            beq = np.zeros((3, 1))
            vras = np.concatenate([beta[:, j], lambd[self.N_f * j:self.N_f * j + 3]])
            prog.AddLinearEqualityConstraint(Aeq, beq, vras)

        ''' Eq(15) '''
        prog.AddBoundingBoxConstraint(0, np.inf, beta).evaluator().set_description("Eq(15)")

        ''' Eq(16) '''
        prog.AddBoundingBoxConstraint(self.eta_min, self.eta_max, eta).evaluator().set_description("Eq(16)")

        ''' Enforce u as com_dd '''
        Aeq = np.hstack([self.plant.CalcJacobianCenterOfMassTranslationalVelocity(plant_context, JacobianWrtVariable.kV,
                                                                                  self.plant.world_frame(), self.plant.world_frame()), np.diag([-1, -1, -1])])
        beq = self.plant.CalcBiasCenterOfMassTranslationalAcceleration(plant_context, JacobianWrtVariable.kV,
                                                                       self.plant.world_frame(), self.plant.world_frame())
        vras = np.concatenate([qdd, u])
        prog.AddLinearEqualityConstraint(Aeq, beq, vras)

        return prog

    def Output(self, context, output):
        calc_t0 = time.time()
        qv = self.get_input_port(0).Eval(context)
        qv_des = self.get_input_port(1).Eval(context)
        q_des = qv_des[:self.nq]
        v_des = qv_des[self.nq:self.nq + self.nv]
        vd_des = (v_des - self.prev_v_des) / self.dt
        self.prev_v_des = v_des
        if self.is_wbc:
            com_des = self.get_input_port(2).Eval(context)
            r_des = com_des[:self.nq_com]
            rd_des = com_des[self.nq_com:self.nq_com + self.nv_com]
            rdd_des = (rd_des - self.prev_rd_des) / self.dt
            self.prev_rd_des = rd_des
            def V(x, u): return self.V_full(x, u, r_des, rd_des, rdd_des)
        else:
            zmp_des = self.get_input_port(2).Eval(context)
            def V(x, u): return self.V_full(x, u, zmp_des)

        if not np.array_equal(qv, self.plant.GetPositionsAndVelocities(self.plant_context)):
            self.plant.SetPositionsAndVelocities(self.plant_context, qv)

        formulate_t0 = time.time()
        prog = self.create_qp(self.plant_context, V, q_des, v_des, vd_des)
        formulate_t1 = time.time()
        if not prog:
            print("Invalid program!")
            output.SetFromVector([0] * self.na)
            return
        solve_t0 = time.time()
        result = Solve(prog)
        solve_t1 = time.time()
        if not result.is_success():
            print(f"Faled to Solve")
            output.SetFromVector([0] * self.na)
            pdb.set_trace()
        cost = [result.EvalBinding(self.cost_v), result.EvalBinding(self.cost_qdd),
                result.EvalBinding(self.cost_epsilon), result.EvalBinding(self.cost_eta)]
        qdd_sol = result.GetSolution(self.qdd)
        lambd_sol = result.GetSolution(self.lambd)
        x_sol = result.GetSolution(self.x)
        u_sol = result.GetSolution(self.u)
        beta_sol = result.GetSolution(self.beta)
        eta_sol = result.GetSolution(self.eta)

        tau = self.tau(qdd_sol, lambd_sol)
        output.SetFromVector(tau)
        calc_t1 = time.time()

        # print("Solver: {}".format(result.get_solver_id().name()))
        # print(f"Cost: {result.get_optimal_cost()}")
        print('Context: {:.3f}, Formulate: {:.4f}, Solve: {:.4f}, Calc: {:.4f}'.format(context.get_time(),
                                                                                       formulate_t1 - formulate_t0,
                                                                                       solve_t1 - solve_t0,
                                                                                       calc_t1 - calc_t0), end='\r')

        if self.pub:
            q = qv[0:self.nq]
            v = qv[self.nq:self.nq + self.nv]
            vdot = (qv[self.nq:self.nq + self.nv] - self.prev_qv[self.nq:self.nq + self.nv]) / self.dt
            # lcm_pub_state("state", q, v, vdot)
            # lcm_pub_state("desire", q_des, v_des, vd_des)
            # lcm_pub_vector("wbc/cost", cost)
            # lcm_pub_vector("wbc/tau", tau)

        self.prev_qv = qv
