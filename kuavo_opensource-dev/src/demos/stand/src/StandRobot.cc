#include <StandRobot.h>
#ifdef KUAVO_CATKIN_MAKE_OPTION
#include <ros/ros.h>
#endif

DECLARE_double(dt);
DECLARE_double(realtime);
DECLARE_double(simulation_time);
DECLARE_bool(pub);
DECLARE_bool(real);
DECLARE_bool(cali);
DECLARE_uint32(traj);
DECLARE_bool(play_back_mode);
DECLARE_bool(log_lcm);

DECLARE_bool(rm_est);
DECLARE_bool(pub_real);
DECLARE_bool(cal_time);
DECLARE_bool(use_motion_cal);
DECLARE_bool(record_motion);
DECLARE_double(powerscale);

DECLARE_bool(disable_keyboard_thread);

namespace HighlyDynamic
{
  using namespace drake;

  StandRobot::StandRobot() : start_sync_(WHIT_THREAD_NUM), end_sync_(WHIT_THREAD_NUM)
  {
    global_robot_ptr = this;
    std::string pose_filepath = env_utils::GetConfigFileParentPath().append("/pose.csv");
    hand_param_ptr = new CSVParamLoader(pose_filepath, ' ');
    arms_init_pos.resize(NUM_ARM_JOINT);
    arms_init_pos << hand_param_ptr->GetParameter("init_hand_pos");
    if (!lc_instance.good())
    {
      std::cout << "Error: Lcm not good!\n";
      exit(1);
    }
    std::cout << std::fixed << std::setprecision(5);
  };

  void StandRobot::initialFSM()
  {
    static StateMachineStand *fsm_stand_ptr = new StateMachineStand(FSMStateMap, Estimate_ptr, traj_ptr, wbc_ptr);
    static StateMachineError *fsm_error_ptr = new StateMachineError(FSMStateMap, Estimate_ptr, traj_ptr, wbc_ptr);

    FSMStateMap[mainPhase_t::P_stand] = fsm_stand_ptr;
    FSMStateMap[mainPhase_t::P_None] = nullptr;
    FSMStateMap[mainPhase_t::P_ERROR] = fsm_error_ptr;
    size_t num_elements = FSMStateMap.size();
    fsm_ptr = FSMStateMap[mainPhase_t::P_stand];
  }

  void StandRobot::plan_thread_func()
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
    }
  }
  void StandRobot::simStep(RobotState_t &state_des_, Eigen::VectorXd &actuation_)
  {
    Eigen::VectorXd cmd_out;
    hw_ptr->joint2motor(state_des_, actuation_, cmd_out);
    actuation_sim = cmd_out.segment(NUM_JOINT * 2, na_with_arm); // 从cmd_out获取tau
    g_plant_with_arm->get_actuation_input_port().FixValue(g_plant_context_with_arm, actuation_sim);
    g_simulator->AdvanceTo(step_count * FLAGS_dt);
    step_count++;
  }
  void StandRobot::state_thread_func()
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
        stateEstimation->Update(state_des_, state_est_, sensor_data_motor, *g_plant_context_with_arm);
      }
      else // 实物
      {
        hw_ptr->Update(state_des_, actuation_);
        hw_ptr->readSensor(sensor_data_motor);
        stateEstimation->Update(state_des_, state_est_, sensor_data_motor);
      }

      clock_gettime(CLOCK_MONOTONIC, &cal_time2);
      index++;
      robot_state_storge->update_state_est(index, state_est_);

      clock_gettime(CLOCK_MONOTONIC, &cal_time3);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }
  }

  void StandRobot::control_thread_func()
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
    }
  }

  int StandRobot::initializeSimulatorAndThreads()
  {

    g_simulator = new systems::Simulator<double>(*diagram, std::move(diagram_context));
    g_simulator->set_target_realtime_rate(FLAGS_realtime);

    th_runing = true;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    plan_thread = std::thread(&StandRobot::plan_thread_func, this);
    if (!plan_thread.joinable())
    {
      printf("plan_thread open failed!\n");
      return -1;
    }

    state_thread = std::thread(&StandRobot::state_thread_func, this);
    if (!state_thread.joinable())
    {
      printf("state_thread open failed!\n");
      return -1;
    }

    control_thread = std::thread(&StandRobot::control_thread_func, this);
    if (!control_thread.joinable())
    {
      printf("control_thread open failed!\n");
      return -1;
    }
    return 0;
  }

  int StandRobot::doMainAsync(int argc, char *argv[])
  {

    static StdoutStreamBuf stdoutStreamBuf;
    std::cout.rdbuf(&stdoutStreamBuf);
    std::cerr.rdbuf(&stdoutStreamBuf); 

    sched_process(70);
    // 参数解析
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    signal(SIGINT, HighlyDynamic::sigintHandler);

    if (FLAGS_log_lcm)
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
    Estimate_ptr = new HighlyDynamic::StandStateEstimation(g_plant, g_plant_with_arm, hw_ptr);
    traj_ptr = new HighlyDynamic::StandTrajectory(g_plant, end_frames_name);
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

  int StandRobot::doMain(int argc, char *argv[])
  {
    int ret = 0;
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

  void StandRobot::real_init_wait()
  {
    std::vector<double> inital_pos(NUM_JOINT, 0);
    if (!FLAGS_cali)
    {
      inital_pos[0] = 0;

      inital_pos[2] = -1.3 * (180.0 / M_PI);
      inital_pos[3] = 1.6 * (180.0 / M_PI);
      // left ankle
      inital_pos[4] = -0.3 * TO_DEGREE; // 适用二代踝关节顺序

      inital_pos[6] = -0;

      inital_pos[8] = -1.3 * (180.0 / M_PI);
      inital_pos[9] = 1.6 * (180.0 / M_PI);
      // right ankle
      inital_pos[10] = -0.3 * TO_DEGREE;

      for (int i = 0; i < NUM_ARM_JOINT; i++)
      {
        inital_pos[LEGS_TOTEL_JOINT + i] = arms_init_pos[i];
      }
    }

    hw_ptr->jointMoveTo(inital_pos, 60, FLAGS_dt);

    printf("如果当前机器人是收退的状态就代表正常，请缓慢将机器人下降到脚底距离地面 3 CM 左右，按下 o 键，让机器人站起来，站起来的过程中需要手动扶一下机器人帮助它在站立的过程中保持平衡。\r\n");

    while (1)
    {
      if (kbhit())
      {
        Walk_Command = getchar();
      }
      if (Walk_Command == 'o')
      {
        printf("feedback start!!! \r\n");
        break;
      }
      if(AMBAC_ready){
          break;
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

  void StandRobot::initialState(multibody::MultibodyPlant<double> *plant, systems::Context<double> *plant_context,
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
    state_des.arm_q.setZero();
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

    state_des.end_effectors = {{motor_info.end_effector_type[0], 0, 0, 0}, {motor_info.end_effector_type[1], 0, 0, 0}};

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

}

