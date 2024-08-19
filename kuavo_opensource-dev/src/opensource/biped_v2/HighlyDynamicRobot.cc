#include <HighlyDynamicRobot.h>
#ifdef KUAVO_CATKIN_MAKE_OPTION
#include <ros/ros.h>
#endif
#include "git_describe.h"
#include "env_utils.h"

DEFINE_double(dt, 1e-3, "Control period.");
DEFINE_double(realtime, 1.0, "Target realtime rate.");
DEFINE_double(simulation_time, 20.0, "simulation Time.");
DEFINE_bool(pub, false, "Publish lcm msg");
DEFINE_bool(real, false, "Run real");
DEFINE_bool(cali, false, "Run calibration");
DEFINE_uint32(traj, 0, "Soure traj");
DEFINE_bool(play_back_mode, false, "play_back_mode, load play_path");
DEFINE_bool(log_lcm, true, "Record log to log file");
DEFINE_bool(log, false, "the same as log_lcm");

DEFINE_bool(rm_est, false, "Remove state estimation");
DEFINE_bool(pub_real, false, "pub real state in sim while !rm_est");
DEFINE_bool(cal_time, false, "calculate time for each loop");
DEFINE_bool(use_motion_cal, false, "use motion capture to estimate");
DEFINE_bool(record_motion, false, "use motion capture, but only record lcm logs");
DEFINE_bool(play, false, "play mode..");
DEFINE_double(powerscale, 1.0, "Scale the period time for make low power cpu can simulator the control period in 1ms.");

DEFINE_bool(disable_keyboard_thread, false, "Disable the keyboard thread when you want to use ROS to change the Phase");

namespace HighlyDynamic
{
  HighlyDynamicRobot *global_robot_ptr = nullptr;
  using namespace drake;
  std::atomic<bool> sigint_caught(false);
  static TeeRedirect stdout_redirect;

  void qv_no_arm_to_arm(Eigen::VectorXd &no_arm_state, Eigen::VectorXd &with_arm_state, uint32_t nq_with_arm, uint32_t nq_no_arm)
  {
    // Eigen::VectorXd state(nq_no_arm + nv_no_arm);
    // state = g_plant->GetPositionsAndVelocities(*g_plant_context_no_arm);

    uint16_t n_half_f = 12, nq_f = 7, nv_f = 6; // 单腿的q数量

    with_arm_state.segment(0, nq_f) << no_arm_state.segment(0, nq_f);
    with_arm_state.segment(nq_f, 6) << no_arm_state.segment(nq_f, 6);
    with_arm_state.segment(nq_f + n_half_f, 6) << no_arm_state.segment(nq_f + 6, 6);

    with_arm_state.segment(nq_with_arm, nv_f) << no_arm_state.segment(nq_no_arm, nv_f);
    with_arm_state.segment(nq_with_arm + nv_f, 6) << no_arm_state.segment(nq_no_arm + nv_f, 6);
    with_arm_state.segment(nq_with_arm + n_half_f + nv_f, 6) << no_arm_state.segment(nq_no_arm + nv_f + 6, 6);
  }
  void qv_arm_to_no_arm(Eigen::VectorXd &no_arm_state, Eigen::VectorXd &with_arm_state, uint32_t nq_with_arm, uint32_t nq_no_arm)
  {

    uint16_t n_half_f = 12, nq_f = 7, nv_f = 6; // 单腿的q数量
    no_arm_state.segment(0, nq_f) << with_arm_state.segment(0, nq_f);
    no_arm_state.segment(nq_f, 6) << with_arm_state.segment(nq_f, 6);
    no_arm_state.segment(nq_f + 6, 6) << with_arm_state.segment(nq_f + n_half_f, 6);

    no_arm_state.segment(nq_no_arm, nv_f) << with_arm_state.segment(nq_with_arm, nv_f);
    no_arm_state.segment(nq_no_arm + nv_f, 6) << with_arm_state.segment(nq_with_arm + nv_f, 6);
    no_arm_state.segment(nq_no_arm + nv_f + 6, 6) << with_arm_state.segment(nq_with_arm + n_half_f + nv_f, 6);
  }

  void HighlyDynamicRobot::setCenterOfMass(const Eigen::Vector3d& new_com, double time, double acc)
  {
    traj_ptr->setCenterOfMass(new_com,time, acc);
  }

  void HighlyDynamicRobot::getCenterOfMass(Eigen::Vector3d& com_out) const
  {
    RobotData robotData = robot_state_storge->newestData();
    RobotState_t state_est = robotData.state_est;
    // traj_ptr->getCenterOfMass(com_out);
    com_out = state_est.r;
  }


  HighlyDynamicRobot::HighlyDynamicRobot() : start_sync_(WHIT_THREAD_NUM), end_sync_(WHIT_THREAD_NUM)
  {
    global_robot_ptr = this;
    std::string pose_filepath = env_utils::GetConfigFileParentPath().append("/pose.csv");
    hand_param_ptr = new CSVParamLoader(pose_filepath, ' ');
    arms_init_pos.resize(NUM_ARM_JOINT);
    arms_init_pos << hand_param_ptr->GetParameter("init_hand_pos").segment(1, NUM_ARM_JOINT);
    head_joint_data_.resize(2);
    if (!lc_instance.good())
    {
      std::cout << "Error: Lcm not good!\n";
      exit(1);
    }
    std::cout << std::fixed << std::setprecision(5);
  };
  void HighlyDynamicRobot::setAMBACReady(bool value)
  {
    AMBAC_ready = value;
  }

  void HighlyDynamicRobot::buildMultibodyPlant()
  {
    std::string model_path = env_utils::GetConfigRootPath();
    model_path.append("/").append(RobotConfig.getValue<std::string>("model_path"));
    std::string model_with_arm_path = env_utils::GetConfigRootPath();
    model_with_arm_path.append("/").append(RobotConfig.getValue<std::string>("model_with_arm_path"));

    g_plant = builder.AddSystem<multibody::MultibodyPlant>(FLAGS_dt);
    g_plant_with_arm = builder.AddSystem<multibody::MultibodyPlant>(FLAGS_dt);
    if (!FLAGS_real)
    {
      lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
      scene_graph = builder.AddSystem<geometry::SceneGraph>();
      g_plant_with_arm->RegisterAsSourceForSceneGraph(scene_graph);
      builder.Connect(g_plant_with_arm->get_geometry_poses_output_port(), scene_graph->get_source_pose_port(g_plant_with_arm->get_source_id().value()));
      builder.Connect(scene_graph->get_query_output_port(), g_plant_with_arm->get_geometry_query_input_port());
      geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, lcm);

      const multibody::ModelInstanceIndex model_instance = multibody::Parser(g_plant_with_arm, scene_graph).AddModelFromFile(model_with_arm_path);
      // Add model of the ground.
      const double static_friction = 1.0;
      const drake::Vector4<double> color(0.9, 0.9, 0.9, 1.0);
      g_plant_with_arm->RegisterVisualGeometry(g_plant_with_arm->world_body(),
                                               math::RigidTransformd(), geometry::HalfSpace(),
                                               "GroundVisualGeometry",
                                               color);
      // For a time-stepping model only static friction is used.
      const multibody::CoulombFriction<double> ground_friction(static_friction, static_friction);
      g_plant_with_arm->RegisterCollisionGeometry(g_plant_with_arm->world_body(),
                                                  math::RigidTransformd(),
                                                  geometry::HalfSpace(),
                                                  "GroundCollisionGeometry",
                                                  ground_friction);
    }
    else
    {
      multibody::Parser(g_plant_with_arm).AddModelFromFile(model_with_arm_path);
    }
    multibody::Parser(g_plant).AddModelFromFile(model_path);
    g_plant_with_arm->set_discrete_contact_solver(drake::multibody::DiscreteContactSolver::kSap);

    auto AddBallConstraintForAnkle = [&](const string &base_body,
                                         const string &tendon_l_body, const string &tendon_r_body,
                                         const string &base_l, const string &base_r,
                                         const string &tendon_l, const string &tendon_r)
    {
      const auto &p_base_l_socket = g_plant_with_arm->GetFrameByName(base_l).GetFixedPoseInBodyFrame();
      const auto &p_tendon_l_socket = g_plant_with_arm->GetFrameByName(tendon_l).GetFixedPoseInBodyFrame();
      const auto &p_base_r_socket = g_plant_with_arm->GetFrameByName(base_r).GetFixedPoseInBodyFrame();
      const auto &p_tendon_r_socket = g_plant_with_arm->GetFrameByName(tendon_r).GetFixedPoseInBodyFrame();
      g_plant_with_arm->AddBallConstraint(g_plant_with_arm->GetBodyByName(base_body),
                                          p_base_l_socket.translation(),
                                          g_plant_with_arm->GetBodyByName(tendon_l_body),
                                          p_tendon_l_socket.translation());
      g_plant_with_arm->AddBallConstraint(g_plant_with_arm->GetBodyByName(base_body),
                                          p_base_r_socket.translation(),
                                          g_plant_with_arm->GetBodyByName(tendon_r_body),
                                          p_tendon_r_socket.translation());
    };
    if (HighlyDynamic::RobotConfig.getValue<bool>("isParallelArm"))
    { // arm
      AddBallConstraintForAnkle("l_hand_pitch", "l_r_arm_tendon", "l_l_arm_tendon",
                                "l_r_hand_socket", "l_l_hand_socket",
                                "l_r_arm_tendon_socket", "l_l_arm_tendon_socket");
      AddBallConstraintForAnkle("r_hand_pitch", "r_r_arm_tendon", "r_l_arm_tendon",
                                "r_r_hand_socket", "r_l_hand_socket",
                                "r_r_arm_tendon_socket", "r_l_arm_tendon_socket");
    }
    // foot
    AddBallConstraintForAnkle("l_foot_roll", "l_l_tendon_y", "l_r_tendon_y",
                              "l_l_foot_socket", "l_r_foot_socket", "l_l_tendon_socket", "l_r_tendon_socket");
    AddBallConstraintForAnkle("r_foot_roll", "r_l_tendon_y", "r_r_tendon_y",
                              "r_l_foot_socket", "r_r_foot_socket", "r_l_tendon_socket", "r_r_tendon_socket");

    g_plant->Finalize();
    g_plant_with_arm->Finalize();

    na_ = g_plant->num_actuated_dofs();
    nq_ = g_plant->num_positions();
    nv_ = g_plant->num_velocities();

    na_with_arm = g_plant_with_arm->num_actuated_dofs();
    nq_with_arm = g_plant_with_arm->num_positions();
    nv_with_arm = g_plant_with_arm->num_velocities();
    std::cout << "na_no_arm:" << na_ << " nq_no_arm:" << nq_ << " nv_no_arm:" << nv_ << std::endl;
    std::cout << "na_with_arm:" << na_with_arm << " nq_with_arm:" << nq_with_arm << " nv_with_arm:" << nv_with_arm << std::endl;

    if (!FLAGS_real)
    {
      sim_sensors_ptr = new SimSensor(g_plant_with_arm, FLAGS_dt);
      sim_sensors_ptr->AddImu(&builder, *g_plant_with_arm);
    }
    diagram = builder.Build();
    diagram_context = diagram->CreateDefaultContext();
    g_plant_context = &diagram->GetMutableSubsystemContext(*g_plant, diagram_context.get());
    g_plant_context_with_arm = &diagram->GetMutableSubsystemContext(*g_plant_with_arm, diagram_context.get());
    robot_state_storge = new ThreadSafeDataStorage();
  }
  void HighlyDynamicRobot::real_init_wait()
  {
    std::vector<double> inital_pos(NUM_JOINT, 0);
    std::vector<double> ready_inital_pos(NUM_JOINT, 0);
    ready_inital_pos[0] = 0;

    ready_inital_pos[2] = -1.3 * (180.0 / M_PI);
    ready_inital_pos[3] = 1.6 * (180.0 / M_PI);
    // left ankle
    ready_inital_pos[4] = -0.3 * TO_DEGREE; // 适用二代踝关节顺序

    ready_inital_pos[6] = -0;

    ready_inital_pos[8] = -1.3 * (180.0 / M_PI);
    ready_inital_pos[9] = 1.6 * (180.0 / M_PI);
    // right ankle
    ready_inital_pos[10] = -0.3 * TO_DEGREE;

    for (int i = 0; i < NUM_ARM_JOINT; i++)
    {
      ready_inital_pos[LEGS_TOTEL_JOINT + i] = arms_init_pos[i];
    }
    bool ready_to_feedback = false;

    if (!FLAGS_cali)
    {
      ready_to_feedback = true;
      inital_pos = ready_inital_pos;
      std::cout << "moving to ready posture ..." << std::endl;
    }
    else
    {
      std::cout << "moving to calibration posture ..." << std::endl;
    }
    hw_ptr->jointMoveTo(inital_pos, 60, FLAGS_dt);

    /********************************测试结束请注销此段***********************************/

    if (!FLAGS_cali)
      std::cout << "\033[32mCheck the status of your robot:"
                << "Type 'o' when you're ready(the robot will stand up!):\033[0m" << std::endl;
    else
      std::cout << "\033[32mCheck the status of your robot and calibrate it\nType 'c' to use temply offset and adjust the joint offset\n"
                << "Type 'o' to move to ready posture, or just 'ctrl+c' to exit..\033[0m" << std::endl;

    if (isTempOffsetFileExist()) {
      std::string tempOffsetFilePath = std::string(TEMP_OFFSET_PATH) + "/offset.csv";
      std::cout << "\033[32m临时零点文件存在，我们将使用临时零点文件: " << tempOffsetFilePath << "\033[0m" << std::endl;
    }

    size_t count = 0;
    while (1)
    {
      if (kbhit())
      {
        Walk_Command = '\0';
        Walk_Command = getchar();
        if (Walk_Command == 'c' && FLAGS_cali)
        {
          hw_ptr->reloadZeroOffsetConfig(inital_pos);
        }
        if (Walk_Command == 'o')
        {
          if (!ready_to_feedback)
          {
            inital_pos = ready_inital_pos;
            std::cout << "moving to ready posture..." << std::endl;
            hw_ptr->jointMoveTo(inital_pos, 60, FLAGS_dt);
            std::cout << "\033[32mType 'q' to goback to calibration status, or type 'o' again when you're ready(the robot will stand up!):\033[0m" << std::endl;
            ready_to_feedback = true;
            continue;
          }
          printf("feedback start!!! \r\n");
          break;
        }
        if (Walk_Command == 'q')
        {
          if (ready_to_feedback)
          {
            std::fill(inital_pos.begin(), inital_pos.end(), 0.0);
            std::cout << "moving to calibration posture..." << std::endl;
            hw_ptr->jointMoveTo(inital_pos, 60, FLAGS_dt);
            std::cout << "\033[32mType 'o' to enter ready status\033[0m" << std::endl;
            ready_to_feedback = false;
            continue;
          }
          exit(0);
        }
      }

      if (AMBAC_ready)
      {
        break;
      }
      if (FLAGS_cali && count++ % 1000 == 0)
      {
        std::vector<JointParam_t> joint_data;
        joint_data.resize(NUM_JOINT);
        hw_ptr->GetJointData(motor_info.joint_ids, joint_data);
        std::cout << "\n"
                  << std::setw(80) << std::left << "Joints current:      " << std::endl;

        int num_clo = 4; // 分四列
        int num_row = std::ceil(static_cast<double>(NUM_JOINT) / num_clo);
        for (int i = 0; i < num_row; i++)
        {
          std::cout << std::left;
          for (int j = 0; j < num_clo; j++)
          {
            if (i * num_clo + j >= NUM_JOINT)
              break;
            std::cout << std::setw(2) << i * num_clo + j + 1 << ":"
                      << std::setw(10) << joint_data[i * num_clo + j].torque;
          }
          std::cout << std::endl;
        }

        for (int i = 0; i < num_row + 2; i++)
        {
          std::cout << "\033[F";
        }
        std::cout.flush();
      }
#ifdef KUAVO_CATKIN_MAKE_OPTION
      ros::spinOnce();
#endif
      usleep(1000);
    }

    if (FLAGS_cali)
    {
      for (int i = 0; i < NUM_ARM_JOINT; i++)
      {
        inital_pos[LEGS_TOTEL_JOINT + i] = arms_init_pos[i];
      }
    }
    Eigen::VectorXd q_tmp(NUM_JOINT);
    q_tmp << q_initial.segment(nq_f, 12), Eigen::VectorXd::Zero(NUM_ARM_JOINT); // TODO L no arm fix
    std::vector<double> q_initial_vec;
    q_initial_vec.assign(&q_tmp[0], q_tmp.data() + q_tmp.rows() * q_tmp.cols());
    for (uint32_t i = 0; i < q_initial_vec.size(); i++)
    {
      q_initial_vec[i] *= TO_DEGREE;
    }
    std::copy(inital_pos.begin() + LEGS_TOTEL_JOINT, inital_pos.end(), q_initial_vec.begin() + LEGS_TOTEL_JOINT); // 覆盖手部关节
    hw_ptr->jointMoveTo(q_initial_vec, 30, FLAGS_dt);
    FLAGS_realtime = 1.0;
  }
  void HighlyDynamicRobot::initialState(multibody::MultibodyPlant<double> *plant, systems::Context<double> *plant_context,
                                        std::vector<std::string> &initial_joint_name, std::vector<double> &initial_joint_pos)
  {
    math::RigidTransformd foot_in_torso = plant->GetFrameByName(end_frames_name[1]).CalcPose(*plant_context, plant->GetFrameByName(end_frames_name[0]));
    math::RigidTransformd r_foot_in_torso = plant->GetFrameByName(end_frames_name[2]).CalcPose(*plant_context, plant->GetFrameByName(end_frames_name[0]));
    Eigen::Vector3d p0_foot(0, foot_in_torso.translation()[1], 0);
    Eigen::VectorXd q0 = plant->GetPositions(*plant_context);
    q0[6] = -foot_in_torso.translation()[2]; // 从动力学模型中测量出来的脚和躯干的高度（值为负），将躯提升到地面上这个高度
    plant->SetPositions(plant_context, q0);
    Eigen::VectorXd r0 = plant->CalcCenterOfMassPositionInWorld(*plant_context);

    std::cout << "foot width: " << foot_in_torso.translation()[1] - r_foot_in_torso.translation()[1] << std::endl;

    for (uint32_t i = 0; i < initial_joint_name.size(); i++)
    {
      const multibody::RevoluteJoint<double> &joint = plant->GetJointByName<multibody::RevoluteJoint>(initial_joint_name[i]);
      joint.set_angle(plant_context, initial_joint_pos[i]);
    }
    q0 = plant->GetPositions(*plant_context);

    if (FLAGS_traj == 0)
    {
      CoMIK ik(plant, end_frames_name, 1e-6, 1e-6);
      auto torsoY = RobotConfig.getValue<double>("torsoY") * TO_RADIAN;
      std::vector<std::vector<Eigen::Vector3d>> pose_vec{
          {Eigen::Vector3d(0, RobotConfig.getValue<double>("torsoP") * TO_RADIAN, torsoY), Eigen::Vector3d(0, 0, RobotConfig.getValue<double>("com_z"))},
          {Eigen::Vector3d(0, 0, torsoY), Eigen::Vector3d(p0_foot[0], RobotConfig.getValue<double>("StepWith") / 2, p0_foot[2])},
          {Eigen::Vector3d(0, 0, torsoY), Eigen::Vector3d(p0_foot[0], -RobotConfig.getValue<double>("StepWith") / 2, p0_foot[2])}};
      Eigen::VectorXd q;
      bool result = ik.solve(pose_vec, q0, q);
      if (result == false)
      {
        std::cout << "torso: " << pose_vec[0][0].transpose() << ", " << pose_vec[0][1].transpose() << "\n";
        std::cout << "lfoot: " << pose_vec[1][0].transpose() << ", " << pose_vec[1][1].transpose() << "\n";
        std::cout << "rfoot: " << pose_vec[2][0].transpose() << ", " << pose_vec[2][1].transpose() << "\n";
        throw std::runtime_error("Failed to IK0!");
      }
      plant->SetPositions(plant_context, q);
      q_initial = q;
    }
    else if (FLAGS_traj == 1)
    {
      uint32_t nq_ = plant->num_positions();
      std::vector<std::vector<double_t>> traj;
      if (!readCsvData(traj_file.c_str(), true, traj))
      {
        throw std::runtime_error("Failed to open " + std::string(traj_file));
      }
      Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd>(traj[0].data(), nq_, 1);
      plant->SetPositions(plant_context, q);
      q_initial = q;
    }

    state_des.q.resize(nq_);
    state_des.v.resize(nv_);
    state_des.vd.resize(nv_);
    state_des.arm_q.resize(NUM_ARM_JOINT);
    state_des.arm_v.resize(NUM_ARM_JOINT);
    state_des.arm_vd.resize(NUM_ARM_JOINT);
    state_des.arm_q = arms_init_pos;
    state_des.arm_v.setZero();
    state_des.arm_vd.setZero();
    state_des.tau.resize(na_with_arm);
    state_des.tau_max.resize(na_with_arm);
    state_des.tau_ratio.resize(na_with_arm);
    state_des.q = q_initial;
    state_des.v.setZero();
    state_des.vd.setZero();
    state_des.tau.setZero();
    state_des.tau_ratio = Eigen::VectorXd::Constant(na_with_arm, 1);
    state_des.phase = P_stand;
    state_des.sub_phase = walk_pre;
    state_des.walk_contact = Double_contact;
    state_des.lf.setZero();
    state_des.rf.setZero();
    state_des.lfv.setZero();
    state_des.rfv.setZero();
    state_des.phase_time = 0;
    Eigen::VectorXd init_zero(6);
    state_des.end_effectors = {{motor_info.end_effector_type[0], 0, 0, 0, init_zero, init_zero, init_zero},
                               {motor_info.end_effector_type[1], 0, 0, 0, init_zero, init_zero, init_zero}};

    state_est = state_des;

    state_des.arm_q << arms_init_pos;
    state_des.arm_q = state_des.arm_q * TO_RADIAN;

    actuation.resize(na_with_arm);
    actuation_sim.resize(na_with_arm);
    for (uint32_t i = 0; i < na_with_arm; i++)
    {
      actuation[i] = 0;
      state_des.tau_max[i] = motor_info.max_current[i];
      state_est.tau_max[i] = motor_info.max_current[i];
    }
    actuation_sim.setZero();
    state_est.contact_force = Eigen::VectorXd::Constant(9, 0);

    Eigen::VectorXd qv_with_arm(nq_with_arm + nv_with_arm);
    Eigen::VectorXd qv_no_arm(nq_ + nv_);
    qv_with_arm.setZero();
    qv_no_arm = g_plant->GetPositionsAndVelocities(*g_plant_context);
    hw_ptr->qv_joint_to_motor(qv_no_arm, qv_with_arm, nq_with_arm, nq_);
    // qv_no_arm_to_arm(qv_no_arm, qv_with_arm, nq_with_arm, nq_);
    g_plant_with_arm->SetPositionsAndVelocities(g_plant_context_with_arm, qv_with_arm);
    // set g_plant input port
    Eigen::VectorXd tau(na_);
    tau.setZero();
    g_plant->get_actuation_input_port().FixValue(g_plant_context, tau);
    Eigen::VectorXd tau_wa(na_with_arm);
    tau_wa.setZero();
    g_plant_with_arm->get_actuation_input_port().FixValue(g_plant_context_with_arm, tau_wa);
    if (!FLAGS_real)
    {
      sim_sensors_ptr->SetContext(diagram.get(), diagram_context.get());
    }
    double total_mass_ = g_plant->CalcTotalMass(*g_plant_context);
    double total_mass_with_arm = g_plant_with_arm->CalcTotalMass(*g_plant_context_with_arm);
    std::cout << "total_mass_:" << total_mass_ << "\ntotal_mass_with_arm:" << total_mass_with_arm << std::endl;
    initialFSM();
  }
  void HighlyDynamicRobot::initialRobot()
  {
    SensorData_t sensor_data_motor;
    sensor_data_motor.resizeJoint(NUM_JOINT);
    if (FLAGS_real)
    {
      if (hw_ptr->HWPlantInit(lc_instance) != 0)
      {
        std::cout << "HWPlantInit failed!\n";
        exit(1);
      }
      // if (!hw_ptr->HWPlantCheck()) exit(1);
      sensor_data_motor = hw_ptr->sensorsInit();
      // if (!hw_ptr->sensorsCheck()) exit(1);
      real_init_wait();
    }
    else
    {
      sim_sensors_ptr->readSensor(sensor_data_motor, *g_plant_context_with_arm);
    }
    if (!FLAGS_play_back_mode)
      Estimate_ptr->Initialize(state_est, sensor_data_motor, q_initial);

    traj_ptr->Initialize(state_des, state_est, q_initial);
  }
  void HighlyDynamicRobot::initialFSM()
  {
    static FSMStateStand *fsm_stand_ptr = new FSMStateStand(FSMStateMap, Estimate_ptr, traj_ptr, wbc_ptr);
    static FSMStateJump *fsm_jump_ptr = new FSMStateJump(FSMStateMap, Estimate_ptr, traj_ptr, wbc_ptr);
    static FSMStateWalk *fsm_walk_ptr = new FSMStateWalk(FSMStateMap, Estimate_ptr, traj_ptr, wbc_ptr);
    static FSMStateSquat *fsm_squat_ptr = new FSMStateSquat(FSMStateMap, Estimate_ptr, traj_ptr, wbc_ptr);
    static FSMStateError *fsm_error_ptr = new FSMStateError(FSMStateMap, Estimate_ptr, traj_ptr, wbc_ptr);

    FSMStateMap[mainPhase_t::P_stand] = fsm_stand_ptr;
    FSMStateMap[mainPhase_t::P_jump] = fsm_jump_ptr;
    FSMStateMap[mainPhase_t::P_walk] = fsm_walk_ptr;
    FSMStateMap[mainPhase_t::P_squat] = fsm_squat_ptr;
    FSMStateMap[mainPhase_t::P_None] = nullptr;
    FSMStateMap[mainPhase_t::P_ERROR] = fsm_error_ptr;
    size_t num_elements = FSMStateMap.size();
    std::cout << "Number of FSMState: " << num_elements << std::endl;
    fsm_ptr = FSMStateMap[mainPhase_t::P_stand];
  }

  void HighlyDynamicRobot::getRobotState(RobotState_t &robotState)
  {
    RobotData new_data = robot_state_storge->newestData();
    RobotState_t state_des = new_data.state_des;
    robotState = new_data.state_est;
    robotState.position_cmd = traj_ptr->position_cmd;
    robotState.vel_cmd = traj_ptr->vel_cmd;
    robotState.phase = state_des.phase;
  }

  void HighlyDynamicRobot::plan_thread_func()
  {
    double dt = FLAGS_dt;

    RobotState_t state_des_ = state_des;
    FSMState *current_fsm_ptr_ = fsm_ptr;
    timespec cal_time0, cal_time1, cal_time2, cal_time3, cal_time4;
    uint64_t index = 0;
    robot_state_storge->update_state_des(index, state_des_);

    while (th_runing)
    {
      clock_gettime(CLOCK_MONOTONIC, &cal_time0);
      mtx_fsm.lock();
      current_fsm_ptr_ = fsm_ptr;
      mtx_fsm.unlock();

      RobotData &new_data_ref = robot_state_storge->get(index, false, true, false);
      RobotState_t &state_est_ = new_data_ref.state_est;

      clock_gettime(CLOCK_MONOTONIC, &cal_time1);
      current_fsm_ptr_->onPlan(state_des_, state_est_);
      mtx_fsm.lock();
      current_fsm_ptr_->checkFsmChange(state_est_, fsm_ptr);
      mtx_fsm.unlock();

      next_time.tv_sec += (next_time.tv_nsec + dt * FLAGS_powerscale * 1e9) / 1e9;
      next_time.tv_nsec = (int)(next_time.tv_nsec + dt * FLAGS_powerscale * 1e9) % (int)1e9;

      clock_gettime(CLOCK_MONOTONIC, &cal_time2);
      index++;
      robot_state_storge->update_state_des(index, state_des_);

      clock_gettime(CLOCK_MONOTONIC, &cal_time3);

      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
      if (FLAGS_cal_time)
      {

        clock_gettime(CLOCK_MONOTONIC, &cal_time4);
        Eigen::Vector4d time = {TIME_DIFF_MS(cal_time0, cal_time1), TIME_DIFF_MS(cal_time0, cal_time2), TIME_DIFF_MS(cal_time0, cal_time3), TIME_DIFF_MS(cal_time0, cal_time4)};
        Eigen::Vector4d time_diff = {TIME_DIFF_MS(cal_time0, cal_time1), TIME_DIFF_MS(cal_time1, cal_time2), TIME_DIFF_MS(cal_time2, cal_time3), TIME_DIFF_MS(cal_time3, cal_time4)};
        lcmPublishVector(&lc_instance, "sync/plan_thread", time);
        lcmPublishVector(&lc_instance, "sync/plan_thread_diff", time_diff);
      }
    }
  }
  void HighlyDynamicRobot::simStep(RobotState_t &state_des_, Eigen::VectorXd &actuation_)
  {
    Eigen::VectorXd cmd_out;
    hw_ptr->joint2motor(state_des_, actuation_, cmd_out);
    actuation_sim = cmd_out.segment(NUM_JOINT * 2, na_with_arm); // 从cmd_out获取tau
    g_plant_with_arm->get_actuation_input_port().FixValue(g_plant_context_with_arm, actuation_sim);
    g_simulator->AdvanceTo(step_count * FLAGS_dt);
    step_count++;
    // state_with_arm = g_plant_with_arm->GetPositionsAndVelocities(*g_plant_context_with_arm);
  }
  void HighlyDynamicRobot::updateHeadJointData()
  {
    hw_ptr->head_joint_data_ = head_joint_data_;
  }
  void HighlyDynamicRobot::state_thread_func()
  {
    double dt = FLAGS_dt;

    RobotState_t state_est_ = state_est;
    SensorData_t sensor_data_motor;
    sensor_data_motor.resizeJoint(NUM_JOINT);

    StateEstimation *stateEstimation;
    stateEstimation = Estimate_ptr;
    FSMState *current_fsm_ptr_ = fsm_ptr;
    timespec cal_time0, cal_time1, cal_time2, cal_time3, cal_time4, cal_time11, cal_time12, cal_time13, cal_time14;
    uint64_t index = 0;
    robot_state_storge->update_state_est(index, state_est_);
    while (th_runing)
    {
      clock_gettime(CLOCK_MONOTONIC, &cal_time0);
      mtx_fsm.lock();
      current_fsm_ptr_ = fsm_ptr;
      mtx_fsm.unlock();

      RobotData &new_data_ref = robot_state_storge->get(index, true, false, true);
      RobotState_t &state_des_ = new_data_ref.state_des;
      Eigen::VectorXd &actuation_ = new_data_ref.actuation;

      clock_gettime(CLOCK_MONOTONIC, &cal_time1);
      if (!FLAGS_real) // 仿真
      {
        simStep(state_des_, actuation_);
        sim_sensors_ptr->readSensor(sensor_data_motor, *g_plant_context_with_arm);
        
        {
          std::lock_guard<std::mutex> lock(mtx_sensor_data_); 
          sensor_data_ = sensor_data_motor;
        }

        stateEstimation->Update(state_des_, state_est_, sensor_data_motor, *g_plant_context_with_arm);
      }
      else // 实物
      {
        updateHeadJointData();
        hw_ptr->Update(state_des_, actuation_);
        hw_ptr->readSensor(sensor_data_motor);

        {
          std::lock_guard<std::mutex> lock(mtx_sensor_data_); 
          sensor_data_ = sensor_data_motor;
        }
        
        stateEstimation->Update(state_des_, state_est_, sensor_data_motor);
        state_est_.head_joint_q = sensor_data_motor.head_joint_q;
      }

      clock_gettime(CLOCK_MONOTONIC, &cal_time2);
      index++;

      robot_state_storge->update_state_est(index, state_est_);

      clock_gettime(CLOCK_MONOTONIC, &cal_time3);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
      if (FLAGS_cal_time)
      {

        clock_gettime(CLOCK_MONOTONIC, &cal_time4);
        if (TIME_DIFF_MS(cal_time0, cal_time4) > (15 * FLAGS_powerscale))
        {
          std::cout << " TIME_DIFF_MS(cal_time3, cal_time4) > 15 :" << TIME_DIFF_MS(cal_time3, cal_time4) << std::endl;
        }
        Eigen::Vector4d time = {TIME_DIFF_MS(cal_time0, cal_time1), TIME_DIFF_MS(cal_time0, cal_time2), TIME_DIFF_MS(cal_time0, cal_time3), TIME_DIFF_MS(cal_time0, cal_time4)};
        Eigen::Vector4d time_diff = {TIME_DIFF_MS(cal_time0, cal_time1), TIME_DIFF_MS(cal_time1, cal_time2), TIME_DIFF_MS(cal_time2, cal_time3), TIME_DIFF_MS(cal_time3, cal_time4)};
        lcmPublishVector(&lc_instance, "sync/state_thread", time);
        lcmPublishVector(&lc_instance, "sync/state_thread_diff", time_diff);
      }
    }
  }

  void HighlyDynamicRobot::control_thread_func()
  {
    double dt = FLAGS_dt;

    Eigen::VectorXd actuation_ = actuation;
    FSMState *current_fsm_ptr_ = fsm_ptr;
    timespec cal_time0, cal_time1, cal_time2, cal_time3, cal_time4;
    uint64_t index = 0;
    robot_state_storge->update_actuation(index, actuation_);
    while (th_runing)
    {
      clock_gettime(CLOCK_MONOTONIC, &cal_time0);
      mtx_fsm.lock();
      current_fsm_ptr_ = fsm_ptr;
      mtx_fsm.unlock();

      RobotData &new_data_ref = robot_state_storge->get(index, true, true, false);
      RobotState_t &state_des_ = new_data_ref.state_des;
      RobotState_t &state_est_ = new_data_ref.state_est;
      clock_gettime(CLOCK_MONOTONIC, &cal_time1);
      current_fsm_ptr_->onCalTau(state_des_, state_est_, actuation_);

      clock_gettime(CLOCK_MONOTONIC, &cal_time2);
      index++;
      robot_state_storge->update_actuation(index, actuation_);
      clock_gettime(CLOCK_MONOTONIC, &cal_time3);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
      if (FLAGS_cal_time)
      {
        clock_gettime(CLOCK_MONOTONIC, &cal_time4);
        Eigen::Vector4d time = {TIME_DIFF_MS(cal_time0, cal_time1), TIME_DIFF_MS(cal_time0, cal_time2), TIME_DIFF_MS(cal_time0, cal_time3), TIME_DIFF_MS(cal_time0, cal_time4)};
        Eigen::Vector4d time_diff = {TIME_DIFF_MS(cal_time0, cal_time1), TIME_DIFF_MS(cal_time1, cal_time2), TIME_DIFF_MS(cal_time2, cal_time3), TIME_DIFF_MS(cal_time3, cal_time4)};
        lcmPublishVector(&lc_instance, "sync/control_thread", time);
        lcmPublishVector(&lc_instance, "sync/control_thread_diff", time_diff);
      }
    }
  }

  RobotData HighlyDynamicRobot::queryNewestRobotStates()
  {
    return robot_state_storge->newestData();
  }

  void HighlyDynamicRobot::switchArmCtrlMode(bool rosArmMode)
  {
    if (rosArmMode)
    {
      traj_ptr->setRosArmTrue();
    }
    else
    {
      traj_ptr->setRosArmFalse();
    }
  }

  void HighlyDynamicRobot::SetsetEndEffectors(Eigen::Vector2d target_left_right_pos)
  {
    // setting end effectors
    traj_ptr->setEndEffectors(target_left_right_pos);
  }

  void HighlyDynamicRobot::keyboard_thread_func()
  {
    usleep(100000);
    struct sched_param param;
    param.sched_priority = 0;
    auto result = pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    if (result != 0)
    {
      std::cerr << "Failed to set keyboard_thread_func's scheduling parameters. Error: " << strerror(result) << std::endl;
    }
    // pthread_setname_np(pthread_self(), "kuavo_keyboard_thread");
    controlMode_t control_mode = VelocityControl;
    traj_ptr->changeCtlMode(control_mode);
    Eigen::Vector3d cmd_vel_step = RobotConfig.getValue<Eigen::VectorXd>("cmd_vel_step");
    double vx, vy, vyaw;
    Eigen::Vector3d step_cmd = {0.0, 0, 0};

    while (th_runing)
    {
      RobotData robot_states = robot_state_storge->newestData();
      RobotState_t state_des_ = robot_states.state_des;
      if (kbhit())
      {
        Walk_Command = getchar();
        if (Walk_Command == 'm')
        {
          switch (control_mode)
          {
          case VelocityControl:
            control_mode = PositionControl;
            break;
          case PositionControl:
            control_mode = StepControl;
            break;
          case StepControl:
            control_mode = VelocityControl;
            break;
          default:
            break;
          }
          // control_mode = (control_mode == PositionControl) ? VelocityControl : PositionControl;
          traj_ptr->changeCtlMode(control_mode);
          vx = vy = vyaw = 0;
          std::cout << "\n"
                    << controlMode_name_map[control_mode] << "模式" << std::endl;
        }
        if (state_des_.phase == P_stand)
        {
          Eigen::VectorXd left_right(12);
          switch (Walk_Command)
          {
          case 'r':
            std::cout << "\n"
                      << controlMode_name_map[control_mode] << "模式" << std::endl;
            vx = vy = vyaw = 0;
            traj_ptr->changePhases(P_walk);
            break;
          case 'p':
            traj_ptr->changePhases(P_squat, squat_normal);
            break;
          case 't':
            traj_ptr->changePhases(P_squat, squat_quick);
            break;
          case 'j':
            traj_ptr->changePhases(P_jump, jump_pre);
            break;
          case 'v':
            // 按下V键 开始规划手臂路径
            traj_ptr->setRosArmTrue();
            break;
          case 'b':
            // 按下B键 停止规划手臂路径
            traj_ptr->setRosArmFalse();
            break;
          case 'k':
            if(traj_ptr->getEndEffectorType() == EndEffectorType::jodell){
                left_right << 10, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0;
                traj_ptr->setEndhand(left_right);
            }
            else if(traj_ptr->getEndEffectorType() == EndEffectorType::qiangnao){
              // 填充 left 和 right 向量的值
              left_right << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
              traj_ptr->setEndhand(left_right);
            }
            else {
              // None
            }
            break;
          case 'h':
            if(traj_ptr->getEndEffectorType() == EndEffectorType::jodell){
                left_right << 250, 0, 0, 0, 0, 0, 250, 0, 0, 0, 0, 0;
                traj_ptr->setEndhand(left_right);
            }
            else if(traj_ptr->getEndEffectorType() == EndEffectorType::qiangnao){
              // 填充 left 和 right 向量的值
              left_right << 65, 65, 90, 80, 80, 90, 65, 65, 90, 80, 80, 90;
              traj_ptr->setEndhand(left_right);
            }
            else {
              // None
            }
            break;
          case 'w':
            step_cmd = {0.1, 0.0, 0.0};
            traj_ptr->stepCommand(3, step_cmd);
            break;
          case 's':
            step_cmd = {-0.1, 0.0, 0.0};
            traj_ptr->stepCommand(3, step_cmd);
            break;
          case 'a':
            step_cmd = {0.0, 0.05, 0.0};
            traj_ptr->stepCommand(3, step_cmd);
            break;
          case 'd':
            step_cmd = {0.0, -0.05, 0.0};
            traj_ptr->stepCommand(3, step_cmd);
            break;
          case 'q':
            step_cmd = {0.0, 0.0, 8};
            traj_ptr->stepCommand(3, step_cmd);
            break;
          case 'e':
            step_cmd = {0.0, 0.0, -8};
            traj_ptr->stepCommand(3, step_cmd);
            break;
            // case 'l':
            // {
            //   int index_l = 2;
            //   std::pair<int, int> delay_size;
            //   RobotState_t new_state;
            //   auto start = std::chrono::steady_clock::now();
            //   while (true)
            //   {
            //     auto now = std::chrono::steady_clock::now();
            //     auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

            //     if (kbhit())
            //     {
            //       getRobotState(new_state);
            //       char new_cmd = getchar();
            //       if (new_cmd == 'q')
            //         break;
            //     }

            //     delay_size = traj_ptr->changeHandArmPoseDelay(index_l);
            //     if (elapsed_time > delay_size.first){index_l += 1;}
            //     if (index_l >= delay_size.second){break;}

            //     usleep(1000 * 100);
            //   }

            //   traj_ptr->changeHandArmPose(1);
            //   break;
            // }

          default:
            if (std::isdigit(Walk_Command))
            {
              int index = Walk_Command - '0';
              if (index == 9)
              {
                std::string config_name = "poses_1.csv";
                traj_ptr->changeHandArmPoses(config_name);
              }
              else if (index == 8)
              {
                std::string config_name = "poses_0.csv";
                traj_ptr->changeHandArmPoses(config_name);
              }
              else
              {
                // traj_ptr->changeArmPose(index);
                traj_ptr->changeHandArmPose(index);
              }
            }
            break;
          }
        }
        else if (state_des_.phase == P_squat)
        {
          switch (Walk_Command)
          {
          case 'q':
          case 'c':
            traj_ptr->changePhases(P_stand);
            std::cout << "exiting squat status...\n";
            break;
          default:
            break;
          }
        }
        else if (state_des_.phase == P_walk)
        {
          if (Walk_Command == 'c')
          {
            traj_ptr->changePhases(P_stand);
            vx = vy = vyaw = 0;
          }

          {
            if (Walk_Command == 'w')
            {
              if (control_mode == VelocityControl)
              {
                vx += cmd_vel_step[0];
              }
              else if (control_mode == StepControl)
              {
                step_cmd = {0.1, 0.0, 0.0};
                traj_ptr->stepCommand(3, step_cmd);
              }
              else
              {
                PositionDelta cmd = {0.2, 0, 0.0};
                traj_ptr->positionCommand(cmd);
              }
            }
            if (Walk_Command == 's')
            {
              if (control_mode == VelocityControl)
              {
                vx -= cmd_vel_step[0];
              }
              else if (control_mode == StepControl)
              {
                step_cmd = {-0.1, 0.0, 0.0};
                traj_ptr->stepCommand(3, step_cmd);
              }
              else
              {
                PositionDelta cmd = {-0.2, 0, 0.0};
                traj_ptr->positionCommand(cmd);
              }
            }
            if (Walk_Command == 'a')
            {
              if (control_mode == VelocityControl)
              {
                vy += cmd_vel_step[1];
              }
              else if (control_mode == StepControl)
              {
                step_cmd = {0.0, 0.05, 0.0};
                traj_ptr->stepCommand(3, step_cmd);
              }
              else
              {
                PositionDelta cmd = {0, 0.2, 0.0};
                traj_ptr->positionCommand(cmd);
              }
            }
            if (Walk_Command == 'd')
            {
              if (control_mode == VelocityControl)
              {
                vy -= cmd_vel_step[1];
              }
              else if (control_mode == StepControl)
              {
                step_cmd = {0.0, -0.05, 0.0};
                traj_ptr->stepCommand(3, step_cmd);
              }
              else
              {
                PositionDelta cmd = {0, -0.2, 0.0};
                traj_ptr->positionCommand(cmd);
              }
            }
            if (Walk_Command == 'q')
            {
              if (control_mode == VelocityControl)
              {
                vyaw += cmd_vel_step[2];
              }
              else if (control_mode == StepControl)
              {
                step_cmd = {0.0, 0.0, 8};
                traj_ptr->stepCommand(3, step_cmd);
              }
              else
              {
                PositionDelta cmd = {0, 0, 10.0};
                traj_ptr->positionCommand(cmd);
              }
            }
            if (Walk_Command == 'e')
            {
              if (control_mode == VelocityControl)
              {
                vyaw -= cmd_vel_step[2];
              }
              else if (control_mode == StepControl)
              {
                step_cmd = {0.0, 0.0, -8};
                traj_ptr->stepCommand(3, step_cmd);
              }
              else
              {
                PositionDelta cmd = {0, 0, -10.0};
                traj_ptr->positionCommand(cmd);
              }
            }
          }
          if (Walk_Command == ' ')
          {
            if (control_mode == VelocityControl)
            {
              vx = vy = vyaw = 0;
            }
            else
            {
              traj_ptr->clearPositionCMD();
              std::cout << "stop move"
                        << "\r\n";
            }
          }

          if (control_mode == VelocityControl && Walk_Command != '\0' && Walk_Command != '\n' && Walk_Command != EOF)
          {
            auto velocityData = traj_ptr->velocityCommand({vx, vy, vyaw});
            vx = velocityData.vx;
            vy = velocityData.vy;
            vyaw = 0;
          }

          if (Walk_Command == 'v')
          {
            RobotState_t new_state;
            auto start_time = std::chrono::steady_clock::now();
            while (true)
            {
              auto now = std::chrono::steady_clock::now();
              auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();

              if (kbhit())
              {
                getRobotState(new_state);
                char new_cmd = getchar();
                if (new_cmd == 'q')
                  break;
              }
              traj_ptr->velocityCommand({0.2, 0, 5});
              // if (elapsed_time < 5)
              //   traj_ptr->velocityCommand({0.2, 0, 0});
              // else if (elapsed_time < 90 / 5 * 0.35 + 5)
              //   traj_ptr->velocityCommand({0.0, 0, 5});
              // else
              //   start_time = std::chrono::steady_clock::now();
              usleep(1000 * 100);
            }
            traj_ptr->velocityCommand({0.0, 0, 0});
          }
        }
        else if (state_des_.phase == P_jump)
        {
          if (state_des_.sub_phase == jump_pre)
            switch (Walk_Command)
            {
            case 'j':
            case ' ':
              traj_ptr->changePhases(P_jump, jump_take_off);
              break;
            case 'q':
            case 'c':
              traj_ptr->changePhases(P_stand);
              std::cout << "exiting jump status...\n";
              break;
            case 'p':
              traj_ptr->changePhases(P_squat);
              std::cout << "jump to squat \n";
              break;
            default:
              break;
            }
        }
        Walk_Command = '\0';
      }
      usleep(50000);
    }
  }

  int HighlyDynamicRobot::initializeSimulatorAndThreads()
  {

    g_simulator = new systems::Simulator<double>(*diagram, std::move(diagram_context));
    g_simulator->set_target_realtime_rate(FLAGS_realtime);

    th_runing = true;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    plan_thread = std::thread(&HighlyDynamicRobot::plan_thread_func, this);
    if (!plan_thread.joinable())
    {
      printf("plan_thread open failed!\n");
      return -1;
    }

    state_thread = std::thread(&HighlyDynamicRobot::state_thread_func, this);
    if (!state_thread.joinable())
    {
      printf("state_thread open failed!\n");
      return -1;
    }

    control_thread = std::thread(&HighlyDynamicRobot::control_thread_func, this);
    if (!control_thread.joinable())
    {
      printf("control_thread open failed!\n");
      return -1;
    }
    if (listening_keyboard && !FLAGS_disable_keyboard_thread)
    {
      keyboard_thread = std::thread(&HighlyDynamicRobot::keyboard_thread_func, this);
      if (!keyboard_thread.joinable())
      {
        printf("keyboard_thread_func open failed!\n");
        return -1;
      }
    }
    return 0;
  }
  void recordLog(multibody::MultibodyPlant<double> *plant, const drake::systems::VectorLog<double> &log)
  {
    const char *fileName = "./data/state.csv";
    std::ofstream fp;
    fp.open(fileName);
    if (!fp.is_open())
    {
      throw std::runtime_error("Failed to open " + std::string(fileName));
    }

    fp << "time, ";
    for (uint32_t i = 0; i < plant->num_positions(); i++)
    {
      fp << "q" << i << ", ";
    }
    for (uint32_t i = 0; i < plant->num_velocities() - 1; i++)
    {
      fp << "v" << i << ", ";
    }
    fp << "v" << plant->num_velocities() - 1 << "\n";

    std::cout << "Write data ...\n";
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
    const int m = log.num_samples();
    const int n = log.data().rows() + 1;
    Eigen::MatrixXd data(m, n);
    data << log.sample_times(), log.data().transpose();
    fp << data.format(CSVFormat);
    std::cout << "Write complete\n";
    fp.close();
  }

void sigintHandler(int sig)
{
    // Ensure the signal is only handled once
    if (!sigint_caught.exchange(true))
    {
        printf("Signal requested ...\n");

        // Stop the robot thread if the global pointer is not null
        if (global_robot_ptr != nullptr)
        {
            global_robot_ptr->th_runing = false;
        }

        // Sleep for 100 milliseconds
        usleep(1000 * 100);

        // Stop the logger if logging is enabled
        if (FLAGS_log)
        {
            logger_ptr->set_logfilename_suffix(get_logfilename_suffix());
            logger_ptr->set_timestamp(stdout_redirect.get_timestamp());
            logger_ptr->stop();
        }

        printf("Before Finish stdout redirection.\n");

        // Finish stdout redirection
        stdout_redirect.finish();

        // Deinitialize hardware if in real mode and not in playback mode
        if (FLAGS_real && !FLAGS_play_back_mode)
        {
            std::cout << "HWPlantDeInit" << std::endl;
            HardwarePlant::HWPlantDeInit();
            printf("HWPlantDeInit success!\n");
            imu_stop();
        }
#ifdef KUAVO_CATKIN_MAKE_OPTION
        printf("shutdown ROS\n");
        // Shutdown ROS
        ros::shutdown();
#endif
        // Exit the program
        exit(0);
    }
    else
    {
        std::cout << "Ignoring repeated SIGINT." << std::endl;
    }
}

int HighlyDynamicRobot::doMainAsync(int argc, char *argv[])
{

    sched_process(70);
    // 参数解析
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_log = FLAGS_log || FLAGS_log_lcm;
    signal(SIGINT, HighlyDynamic::sigintHandler);

    if (FLAGS_log)
    {
      logger_ptr = new LogWriter();
      logger_ptr->startBinLog("/tmp/lcm_log.bin");
      // logger_ptr->start("/tmp/log.csv");
    }

    if (FLAGS_powerscale < 1.0)
    {
      std::cout << "FLAGS_powerscale should be larger than 1.0" << std::endl;
      std::cout << "Current FLAGS_powerscale value: " << FLAGS_powerscale << std::endl;
      return 1;
    }
    if (FLAGS_real && FLAGS_powerscale != 1.0)
    {
      std::cout << "The powerscale will cause the robot to fall down when run in real robot" << std::endl;
      return 1;
    }

    // 构建和初始化系统图
    buildMultibodyPlant();

    // 实例化硬件模块、状态估计模块、规划、控制模块
    hw_ptr = new HighlyDynamic::HardwarePlant(g_plant, FLAGS_dt, MOTOR_CONTROL_MODE_POSITION);
    Estimate_ptr = new HighlyDynamic::StateEstimation(g_plant, g_plant_with_arm, hw_ptr);
    traj_ptr = new HighlyDynamic::Trajectory(g_plant, end_frames_name);
    wbc_ptr = new HighlyDynamic::WholeBodyController(g_plant, g_plant_with_arm, contact_frames_name);

    // 初始化机器人状态
    initialState(g_plant, g_plant_context, initial_joint_name, initial_joint_pos);

    // 初始化各个模块
    initialRobot();

    // 初始化仿真器和线程
    if (initializeSimulatorAndThreads() != 0)
    {
      std::cerr << "Failed to initialize simulator and threads!" << std::endl;
      return -1;
    }

    std::cout << "Simulation ..." << std::endl;
    return 0;
  }

  int HighlyDynamicRobot::doMain(int argc, char *argv[])
  {
    int ret = 0;
    std::cout << "Current compiled git repo version: " << GIT_DESCRIBE << std::endl; // Print the current repo version
    ret = doMainAsync(argc, argv);
    if (ret != 0)
    {
      std::cout << "robot start failed!\n";
      return -1;
    }
    while (1)
    {
      sleep(1);
    }
    return 0;
  }

  const SensorData_t& HighlyDynamicRobot::getSensorData() 
  {
    std::lock_guard<std::mutex> lock(mtx_sensor_data_); 
    return sensor_data_;
  }

  bool HighlyDynamicRobot::hasHeadJoint()
  {
    return HardwarePlant::hasHeadJoint();
  }

} // namespace HighlyDynamic
