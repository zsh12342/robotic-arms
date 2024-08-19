#include <HighlyDynamicRobot.h>
#include "csv_player.h"
DECLARE_double(dt);
DECLARE_double(realtime);
DECLARE_bool(pub);
DECLARE_bool(real);
DECLARE_bool(cali);
DECLARE_bool(log);
DECLARE_bool(rm_est);
DECLARE_bool(pub_real);
DECLARE_bool(play_back_mode);
DEFINE_string(play_path, "/tmp/lcm_log.csv", "lcm_log.csv path for playback");
DECLARE_double(powerscale);

#define THREAD_NUM 1
namespace HighlyDynamic
{

    using namespace drake;
    class HighlyDynamicRobotPlayBack : public HighlyDynamicRobot
    {
    public:
        HighlyDynamicRobotPlayBack() : HighlyDynamicRobot(){};

        void state_thread_func() override
        {
            double dt = FLAGS_dt;

            SensorData_t sensor_data_motor;
            sensor_data_motor.resizeJoint(NUM_JOINT);

            StateEstimation *stateEstimation;
            stateEstimation = Estimate_ptr;
            FSMState *current_fsm_ptr_ = fsm_ptr;
            int sensor_data_offset = 0;
            int desire_data_offset = 0;

            if (!logPlayer_ptr->isSim)
            {
                FLAGS_real = true;
                sensor_data_offset = 5;
                std::cout << "log is real\n";
            }
            else
            {
                sensor_data_offset++;
                std::cout << "log is sim\n";
            }
            logPlayer_ptr->readSensor(sensor_data_offset - 1, sensor_data_motor);
            Estimate_ptr->Initialize(state_est, sensor_data_motor, q_initial);
            Eigen::VectorXd actuation_ = actuation;
            RobotState_t state_des_ = state_des;
            RobotState_t state_est_ = state_est;

            while (th_runing)
            {
                mtx_fsm.lock();
                current_fsm_ptr_ = fsm_ptr;
                mtx_fsm.unlock();
                mtx_des.lock();
                mtx_est.lock();
                state_des_ = state_des;
                actuation_ = actuation;
                mtx_est.unlock();
                mtx_des.unlock();

                if (FLAGS_play_back_mode)
                {
                    logPlayer_ptr->readStateDes(step_count + desire_data_offset, state_des_);
                    logPlayer_ptr->readTau(step_count, state_des_.tau);
                    actuation_.segment(0, NUM_JOINT) = state_des_.tau;
                    logPlayer_ptr->readSensor(step_count + sensor_data_offset, sensor_data_motor);
                }

                {
                    Eigen::VectorXd cmd_out;
                    hw_ptr->joint2motor(state_des_, actuation_, cmd_out);

                    actuation_sim = cmd_out.segment(NUM_JOINT * 2, na_with_arm); // 从cmd_out获取tau
                    Eigen::VectorXd qv_with_arm(nq_with_arm + nv_with_arm);
                    Eigen::VectorXd qv_no_arm(nq_ + nv_);
                    qv_no_arm << state_est_.q, state_est_.v;
                    g_plant->SetPositionsAndVelocities(g_plant_context, qv_no_arm);
                    hw_ptr->qv_joint_to_motor(qv_no_arm, qv_with_arm, nq_with_arm, nq_);
                    qv_with_arm.segment(nq_with_arm - NUM_ARM_JOINT, NUM_ARM_JOINT) = state_est_.arm_q;
                    // std::cout << "sim_count:" << sim_count << "\r" << std::flush;
                    g_plant_with_arm->SetPositionsAndVelocities(g_plant_context_with_arm, qv_with_arm);
                    g_plant_with_arm->get_actuation_input_port().FixValue(g_plant_context_with_arm, actuation_sim);
                    sim_count++;
                    g_simulator->AdvanceTo(sim_count * FLAGS_dt);
                    stateEstimation->Update(state_des_, state_est_, sensor_data_motor, *g_plant_context_with_arm);
                }
                // else
                // {
                //     // hw_ptr->Update(state_des_, actuation_);//TODO:在实物上回放也许可以
                //     // hw_ptr->readSensor(sensor_data_motor);
                //     hw_ptr->motor2joint(sensor_data_motor, sensor_data_joint);
                //     stateEstimation->Update(state_des_, state_est_, sensor_data_joint);
                // }
                // if (FLAGS_play_back_mode) // 重新获取状态估计的值, 用于下一次仿真
                // {
                //     logPlayer_ptr->readStateEst(step_count, state_est_);
                // }
                mtx_est.lock();
                state_est = state_est_;
                mtx_est.unlock();
                step_count++;
                next_time.tv_sec += (next_time.tv_nsec + dt * FLAGS_powerscale * 1e9) / 1e9;
                next_time.tv_nsec = (int)(next_time.tv_nsec + dt * FLAGS_powerscale * 1e9) % (int)1e9;

                std::unique_lock<std::mutex> lock(mtx_cv);
                thread_cnt++;
                if (thread_cnt == THREAD_NUM)
                {
                    thread_cnt = 0;
                    lock.unlock();
                    cv.notify_all();
                }
                else
                {
                    cv.wait(lock);
                }
                checkPause();
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
            }
        }

        int doMainAsync(int argc, char *argv[]) override
        {
            sched_process(70);
            gflags::ParseCommandLineFlags(&argc, &argv, true);
            if (!lc_instance.good())
            {
                std::cout << "Error: Lcm not good!\n";
                return -1;
            }
            std::cout << std::fixed << std::setprecision(5);

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

            signal(SIGINT, HighlyDynamic::sigintHandler);
            FLAGS_play_back_mode = true;
            FLAGS_log = true;
            // if (FLAGS_play_back_mode) // 回放模式
            {
                std::string input;
                std::cout << "This log is real or simulated?(<0> for sim , 1 for real)\n";
                std::getline(std::cin, input);
                static CsvPlayer logger_player(FLAGS_play_path, input != "1", FLAGS_dt);
                logPlayer_ptr = &logger_player;
                max_play_back_step = logger_player.getMaxIndex();
            }
            {
                logger_ptr = new LogWriter();
                logger_ptr->startBinLog("/tmp/log_new.bin");
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
            g_simulator = new systems::Simulator<double>(*diagram, std::move(diagram_context));
            g_simulator->set_target_realtime_rate(FLAGS_realtime);

            th_runing = true;
            clock_gettime(CLOCK_MONOTONIC, &next_time);
            while (1)
            {
                RobotConfig.reload();
                initialFSM();
                step_count = 0;
                th_runing = true;
                pause = false;
                // threadStart();
                // state_thread.join();
                // keyboard_thread.join();
                std::cout << ">>> press 'r'|'s' to restart play, ' ' to pause\n press 't' to test IK\n";
                while (1)
                {
                    if (kbhit())
                    {
                        Walk_Command = getchar();
                        if (Walk_Command == ' ')
                        {
                            if (pause && step_count < max_play_back_step)
                            {
                                pause = false;
                                clock_gettime(CLOCK_MONOTONIC, &next_time);
                            }
                            else
                                pause = true;
                            std::cout << ">>> " << ((pause) ? "paused..." : "continue...") << std::endl;
                        }
                        else if (Walk_Command == 'r' || Walk_Command == 's')
                        {
                            step_count = 0;
                            th_runing = false;
                            pause = false;
                            std::cout << ">>> stopping...\n";
                            if (state_thread.joinable())
                                state_thread.join();
                            threadStart();

                            std::cout << ">>> replay...\n";
                            break;
                        }
                        else if (Walk_Command == 't')
                        {
                            int index;
                            std::cout << "Enter an data index to test IK: ";
                            std::cin >> index;
                            RobotState_t state_des_;
                            logPlayer_ptr->readStateDes(index, state_des_);
                            traj_ptr->doIK(state_des_);
                        }

                        Walk_Command = '\0';
                    }
                    usleep(50000);
                }
            }
            return 0;
        }
        int threadStart()
        {
            clock_gettime(CLOCK_MONOTONIC, &next_time);
            // mpc_thread = std::thread(&HighlyDynamicRobotPlayBack::MPC_thread_func, this);
            // if (!mpc_thread.joinable())
            // {
            //     printf("mpc_thread open failed!\n");
            //     return -1;
            // }

            // plan_thread = std::thread(&HighlyDynamicRobotPlayBack::plan_thread_func, this);
            // if (!plan_thread.joinable())
            // {
            //     printf("plan_thread open failed!\n");
            //     return -1;
            // }

            state_thread = std::thread(&HighlyDynamicRobotPlayBack::state_thread_func, this);
            if (!state_thread.joinable())
            {
                printf(">>> state_thread open failed!\n");
                return -1;
            }
            return 0;

            // control_thread = std::thread(&HighlyDynamicRobotPlayBack::control_thread_func, this);
            // if (!control_thread.joinable())
            // {
            //     printf("control_thread open failed!\n");
            //     return -1;
            // }

            // keyboard_thread = std::thread(&HighlyDynamicRobotPlayBack::keyboard_thread_func, this);
            // if (!keyboard_thread.joinable())
            // {
            //     printf("control_thread open failed!\n");
            //     return -1;
            // }
        }

    private:
        void checkPause()
        {
            std::cout << "max_play_back_step"<< max_play_back_step << std::endl;
            std::cout << "step_count" << step_count << std::endl;
            if (step_count >= max_play_back_step - 2)
            {
                std::cout << ">>> play end\npress 'r'|'s' to restart\n";
                pause = true;
            }
            while (pause)
                usleep(10000);
        }
        uint64_t sim_count = 0;
        CsvPlayer *logPlayer_ptr;
        uint64_t max_play_back_step;
        bool pause;
    };

}
int main(int argc, char *argv[])
{
    // sched_process(0);

    HighlyDynamic::HighlyDynamicRobotPlayBack robot;

    robot.doMain(argc, argv);
    return 0;
}
