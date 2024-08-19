#ifndef RUIWO_ACTUATOR_H
#define RUIWO_ACTUATOR_H

#include <Python.h>
#include <vector>
#include <thread>
#include <iostream>
static PyObject *RuiWo_pJoinMethod;// 用于将python线程移动到c++线程中,避免GIL占用

class RuiWoActuator
{
public:
    enum class State {
        None,
        Enabled,
        Disabled
    };

    struct MotorStateData {
        uint8_t id;
        State   state;
        MotorStateData():id(0x0), state(State::None) {}
        MotorStateData(uint8_t id_, State state_):id(id_), state(state_) {}
    };    
    using MotorStateDataVec = std::vector<MotorStateData>;
private:
    PyObject *pModule;
    PyObject *RuiWoActuatorClass;
    PyObject *ActuatorInstance;

    PyObject *pEnableMethod;
    PyObject *pCloseMethod;
    PyObject *pDisableMethod;
    PyObject *pSetPositionMethod;
    PyObject *pSetTorqueMethod;
    PyObject *pSetVelocityMethod;
    PyObject *pGetPositionMethod;
    PyObject *pGetTorqueMethod;
    PyObject *pGetVelocityMethod;
    PyObject *pGetJointStateMethod;
    PyObject *pCheckStateMethod;
    std::string pymodule_path;
    PyGILState_STATE gstate;
    std::thread pythonThread;

public:
    // 需要传入python模块所在路径
    RuiWoActuator(std::string pymodule_path = "");

    ~RuiWoActuator();

    int initialize();
    void enable();
    void disable();
    void close();
    void join();
    // void set_positions(const std::vector<uint8_t> &ids, const std::vector<double> &positions ,std::vector<double> vel ,std::vector<double> pos_kp ,std::vector<double> pos_kd,std::vector<double> torque);
    void set_positions(const std::vector<uint8_t> &ids, const std::vector<double> &positions,const std::vector<double> &torque,const std::vector<double> &velocity);
    void set_torque(const std::vector<uint8_t> &ids, const std::vector<double> &torque);
    void set_velocity(const std::vector<uint8_t> &ids, const std::vector<double> &velocity);
    std::vector<double> get_positions();
    std::vector<double> get_torque();
    std::vector<double> get_velocity();
    std::vector<std::vector<double>> get_joint_state();

    MotorStateDataVec get_motor_state();
};

#endif // RUIWO_ACTUATOR_H
