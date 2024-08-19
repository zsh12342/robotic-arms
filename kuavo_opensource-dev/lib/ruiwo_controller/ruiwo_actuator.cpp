#include "ruiwo_actuator.h"

RuiWoActuator::RuiWoActuator(std::string pymodule_path) : ActuatorInstance(nullptr),
                                                                pEnableMethod(nullptr), pDisableMethod(nullptr),
                                                                pSetPositionMethod(nullptr), pSetTorqueMethod(nullptr), pSetVelocityMethod(nullptr), 
                                                                pGetPositionMethod(nullptr), pGetTorqueMethod(nullptr), pGetVelocityMethod(nullptr),
                                                                pymodule_path(pymodule_path), pGetJointStateMethod(nullptr),pCheckStateMethod(nullptr)
{
}

RuiWoActuator::~RuiWoActuator()
{
    // 析构函数中释放 Python 对象和方法对象
    Py_XDECREF(ActuatorInstance);
    Py_XDECREF(RuiWoActuatorClass);
    Py_XDECREF(pModule);
    Py_XDECREF(pEnableMethod);
    Py_XDECREF(pDisableMethod);
    Py_XDECREF(pSetPositionMethod);
    Py_XDECREF(pSetTorqueMethod);
    Py_XDECREF(pSetVelocityMethod);
    Py_XDECREF(pGetPositionMethod);
    Py_XDECREF(pGetTorqueMethod);
    Py_XDECREF(pGetVelocityMethod);
    Py_XDECREF(pGetJointStateMethod);
    Py_XDECREF(pCloseMethod);
    Py_XDECREF(RuiWo_pJoinMethod);
    Py_XDECREF(pCheckStateMethod);

    // 关闭 Python 解释器
    Py_Finalize();
}

int RuiWoActuator::initialize()
{
    // 初始化 Python 解释器
    Py_Initialize();
    PyRun_SimpleString("import sys");
    std::string add_path_cmd = "sys.path.append('" + pymodule_path + "')";

    PyRun_SimpleString(add_path_cmd.c_str());
    // 导入 Python 模块
    pModule = PyImport_ImportModule("ruiwo_actuator");
    if (!pModule)
    {
        PyErr_Print();
        return -1;
    }

    // 获取 Python 类
    RuiWoActuatorClass = PyObject_GetAttrString(pModule, "RuiWoActuator");
    if (!RuiWoActuatorClass || !PyCallable_Check(RuiWoActuatorClass))
    {
        PyErr_Print();
        return -1;
    }

    // 实例化 Python 类
    ActuatorInstance = PyObject_CallObject(RuiWoActuatorClass, nullptr);
    if (!ActuatorInstance)
    {
        PyErr_Print();
        return -1;
    }

    // 获取类的方法并保存为成员变量
    pEnableMethod = PyObject_GetAttrString(ActuatorInstance, "enable");
    pCloseMethod = PyObject_GetAttrString(ActuatorInstance, "close");
    pDisableMethod = PyObject_GetAttrString(ActuatorInstance, "disable");
    pSetPositionMethod = PyObject_GetAttrString(ActuatorInstance, "set_positions");
    pSetTorqueMethod = PyObject_GetAttrString(ActuatorInstance, "set_torgue");
    pSetVelocityMethod = PyObject_GetAttrString(ActuatorInstance, "set_velocity");
    pGetPositionMethod = PyObject_GetAttrString(ActuatorInstance, "get_positions");
    pGetTorqueMethod = PyObject_GetAttrString(ActuatorInstance, "get_torque");
    pGetVelocityMethod = PyObject_GetAttrString(ActuatorInstance, "get_velocity");
    pGetJointStateMethod = PyObject_GetAttrString(ActuatorInstance, "get_joint_state");
    RuiWo_pJoinMethod = PyObject_GetAttrString(ActuatorInstance, "join");
    pCheckStateMethod = PyObject_GetAttrString(ActuatorInstance, "check_state");

    // 检查获取方法是否成功
    if (!pEnableMethod || !pCloseMethod || !pDisableMethod || !pSetPositionMethod || !pSetTorqueMethod || !pSetVelocityMethod ||
        !pGetPositionMethod || !pGetTorqueMethod|| !pGetVelocityMethod || !pGetJointStateMethod || !RuiWo_pJoinMethod ||
        !pCheckStateMethod)
    {
        PyErr_Print();
        Py_DECREF(ActuatorInstance); // 释放已获取的对象
        return -1;
    }

    // 由于python gil全局解析锁，所以在python多线程环境下，需要在线程中调用join方法
    pythonThread = std::thread(&RuiWoActuator::join, this);
    while (!pythonThread.joinable())
    {
       std::cout << "wait for pythonThread joinable" << std::endl;
       usleep(100000);
    }
    
    struct sched_param param;
    param.sched_priority = 0;
    pthread_setschedparam(pythonThread.native_handle(), SCHED_OTHER, &param);
    return 0;
}

void RuiWoActuator::join()
{

    if (RuiWo_pJoinMethod)
    {
        gstate = PyGILState_Ensure();
        PyObject_CallObject(RuiWo_pJoinMethod, nullptr);
        PyGILState_Release(gstate);
    }
    else
    {
        PyErr_Print();
    }
}

// 添加 enable 方法的 C++ 接口
void RuiWoActuator::enable()
{
    gstate = PyGILState_Ensure();
    if (pEnableMethod)
    {
        PyObject_CallObject(pEnableMethod, nullptr);
    }
    else
    {
        PyErr_Print();
    }
    PyGILState_Release(gstate);
}
void RuiWoActuator::close()
{
    gstate = PyGILState_Ensure();
    if (pCloseMethod)
    {
        PyObject_CallObject(pCloseMethod, nullptr);
        Py_XDECREF(ActuatorInstance);
        Py_XDECREF(RuiWoActuatorClass);
        Py_XDECREF(pModule);
        Py_XDECREF(pEnableMethod);
        Py_XDECREF(pDisableMethod);
        Py_XDECREF(pSetPositionMethod);
        Py_XDECREF(pSetTorqueMethod);
        Py_XDECREF(pSetVelocityMethod);
        Py_XDECREF(pGetPositionMethod);
        Py_XDECREF(pGetTorqueMethod);
        Py_XDECREF(pGetVelocityMethod);
        Py_XDECREF(pGetJointStateMethod);
        Py_XDECREF(pCloseMethod);
        Py_XDECREF(RuiWo_pJoinMethod);
        Py_XDECREF(pCheckStateMethod);
    }
    else
    {
        PyErr_Print();
    }
    PyGILState_Release(gstate);
}
// void RuiWoActuator::join()
// {

//     if (RuiWo_pJoinMethod)
//     {
//         PyObject_CallObject(RuiWo_pJoinMethod, nullptr);
//     }
//     else
//     {
//         PyErr_Print();
//     }
// }
// 添加 disable 方法的 C++ 接口
void RuiWoActuator::disable()
{
    gstate = PyGILState_Ensure();
    if (pDisableMethod)
    {
        PyObject_CallObject(pDisableMethod, nullptr);
    }
    else
    {
        PyErr_Print();
    }
    PyGILState_Release(gstate);
}
// set_positions接口
void RuiWoActuator::set_positions(const std::vector<uint8_t> &ids, const std::vector<double> &positions,const std::vector<double> &torque,const std::vector<double> &velocity)
{
    std::vector<double> rad_position;
    std::vector<double> rad_velocity;
    rad_position = positions;
    rad_velocity = velocity;
    // std::cout << "rad_position:"<<rad_position[0]<<" "<<rad_position[1]<<" "<<rad_position[2]<<" "
    // <<rad_position[3]<<" "<<rad_position[4]<<" "<<rad_position[5]<<std::endl;
    for (size_t i = 0; i < positions.size(); i++)
    {
        rad_position[i] = (positions[i] * 3.14159265358979323846) / 180;
        rad_velocity[i] = (velocity[i] * 3.14159265358979323846) / 180;
    }
    
    gstate = PyGILState_Ensure();
    if (pSetPositionMethod)
    {
        PyObject *pyIds = PyList_New(ids.size());
        PyObject *pyPositions = PyList_New(rad_position.size());
        PyObject *pyTorque = PyList_New(torque.size());
        PyObject *pyVelocity = PyList_New(rad_velocity.size());

        if (!pyIds || !pyPositions || !pyTorque|| !pyVelocity)
        {
            PyErr_Print();
            Py_XDECREF(pyIds);
            Py_XDECREF(pyPositions);
            Py_XDECREF(pyTorque);
            Py_XDECREF(pyVelocity);
            PyGILState_Release(gstate);
            return;
        }

        for (size_t i = 0; i < ids.size(); ++i)
        {
            PyList_SetItem(pyIds, i, PyLong_FromLong(ids[i]));
        }

        for (size_t i = 0; i < rad_position.size(); ++i)
        {
            PyList_SetItem(pyPositions, i, PyFloat_FromDouble(rad_position[i]));
        }

        for (size_t i = 0; i < torque.size(); ++i)
        {
            PyList_SetItem(pyTorque, i, PyFloat_FromDouble(torque[i]));
        }

        for (size_t i = 0; i < velocity.size(); ++i)
        {
            PyList_SetItem(pyVelocity, i, PyFloat_FromDouble(velocity[i]));
        }

        PyObject *args = PyTuple_Pack(4, pyIds, pyPositions, pyTorque, pyVelocity);
        if (!args)
        {
            PyErr_Print();
            Py_DECREF(pyIds);
            Py_DECREF(pyPositions);
            Py_DECREF(pyTorque);
            Py_DECREF(pyVelocity);
            PyGILState_Release(gstate);
            return;
        }

        PyObject_CallObject(pSetPositionMethod, args);

        Py_DECREF(args);
        Py_DECREF(pyIds);
        Py_DECREF(pyPositions);
        Py_DECREF(pyTorque);
        Py_DECREF(pyVelocity);
    }
    else
    {
        PyErr_Print();
    }
    PyGILState_Release(gstate);
}


// set_torque接口
void RuiWoActuator::set_torque(const std::vector<uint8_t> &ids, const std::vector<double> &torque)
{
    
    gstate = PyGILState_Ensure();
    if (pSetTorqueMethod)
    {
        PyObject *pyIds = PyList_New(ids.size());
        PyObject *pyTorque = PyList_New(torque.size());

        if (!pyIds || !pyTorque)
        {
            PyErr_Print();
            Py_XDECREF(pyIds);
            Py_XDECREF(pyTorque);
            PyGILState_Release(gstate);
            return;
        }

        for (size_t i = 0; i < ids.size(); ++i)
        {
            PyList_SetItem(pyIds, i, PyLong_FromLong(ids[i]));
        }

        for (size_t i = 0; i < torque.size(); ++i)
        {
            PyList_SetItem(pyTorque, i, PyFloat_FromDouble(torque[i]));
        }

        PyObject *args = PyTuple_Pack(2, pyIds, pyTorque);
        if (!args)
        {
            PyErr_Print();
            Py_DECREF(pyIds);
            Py_DECREF(pyTorque);
            PyGILState_Release(gstate);
            return;
        }

        PyObject_CallObject(pSetTorqueMethod, args);

        Py_DECREF(args);
        Py_DECREF(pyIds);
        Py_DECREF(pyTorque);
    }
    else
    {
        PyErr_Print();
    }
    PyGILState_Release(gstate);
}

// set_velocity 接口
void RuiWoActuator::set_velocity(const std::vector<uint8_t> &ids, const std::vector<double> &velocity)
{
    
    gstate = PyGILState_Ensure();
    if (pSetVelocityMethod)
    {
        PyObject *pyIds = PyList_New(ids.size());
        PyObject *pyVelocity = PyList_New(velocity.size());

        if (!pyIds || !pyVelocity)
        {
            PyErr_Print();
            Py_XDECREF(pyIds);
            Py_XDECREF(pyVelocity);
            PyGILState_Release(gstate);
            return;
        }

        for (size_t i = 0; i < ids.size(); ++i)
        {
            PyList_SetItem(pyIds, i, PyLong_FromLong(ids[i]));
        }

        for (size_t i = 0; i < velocity.size(); ++i)
        {
            PyList_SetItem(pyVelocity, i, PyFloat_FromDouble(velocity[i]));
        }

        PyObject *args = PyTuple_Pack(2, pyIds, pyVelocity);
        if (!args)
        {
            PyErr_Print();
            Py_DECREF(pyIds);
            Py_DECREF(pyVelocity);
            PyGILState_Release(gstate);
            return;
        }

        PyObject_CallObject(pSetVelocityMethod, args);

        Py_DECREF(args);
        Py_DECREF(pyIds);
        Py_DECREF(pyVelocity);
    }
    else
    {
        PyErr_Print();
    }
    PyGILState_Release(gstate);
}



// 添加 get_positions 方法的 C++ 接口
std::vector<double> RuiWoActuator::get_positions()
{
    std::vector<double> stateList;
    gstate = PyGILState_Ensure();
    if (pGetPositionMethod)
    {
        // PyObject *result = PyObject_GetAttrString(ActuatorInstance, "current_positions");
        PyObject *result = PyObject_CallObject(pGetPositionMethod, nullptr);

        if (result && PyList_Check(result))
        {
            Py_ssize_t size = PyList_Size(result);

            for (Py_ssize_t i = 0; i < size; ++i)
            {
                PyObject *item = PyList_GetItem(result, i);
                // std::cout << "item[" << i << "] = " << PyFloat_AsDouble(item) << std::endl;
                stateList.push_back(PyFloat_AsDouble(item));
                // std::cout << "stateList[" << i << "] = " << stateList[i] << std::endl;
            }
        }
        else
        {
            PyErr_Print();
        }

        Py_XDECREF(result);
    }
    else
    {
        PyErr_Print();
    }
    PyGILState_Release(gstate);

    return stateList;
}

// 添加 get_torque 方法的 C++ 接口
std::vector<double> RuiWoActuator::get_torque()
{
    std::vector<double> stateList;
    gstate = PyGILState_Ensure();
    if (pGetTorqueMethod)
    {
        // PyObject *result = PyObject_GetAttrString(ActuatorInstance, "current_torque");
        PyObject *result = PyObject_CallObject(pGetTorqueMethod, nullptr);

        if (result && PyList_Check(result))
        {
            Py_ssize_t size = PyList_Size(result);

            for (Py_ssize_t i = 0; i < size; ++i)
            {
                PyObject *item = PyList_GetItem(result, i);
                // std::cout << "item[" << i << "] = " << PyFloat_AsDouble(item) << std::endl;
                stateList.push_back(PyFloat_AsDouble(item));
                // std::cout << "stateList[" << i << "] = " << stateList[i] << std::endl;
            }
        }
        else
        {
            PyErr_Print();
        }

        Py_XDECREF(result);
    }
    else
    {
        PyErr_Print();
    }
    PyGILState_Release(gstate);

    return stateList;
}

// 添加 get_velocity 方法的 C++ 接口
std::vector<double> RuiWoActuator::get_velocity()
{
    std::vector<double> stateList;
    gstate = PyGILState_Ensure();
    if (pGetVelocityMethod)
    {
        // PyObject *result = PyObject_GetAttrString(ActuatorInstance, "current_velocity");
        PyObject *result = PyObject_CallObject(pGetVelocityMethod, nullptr);

        if (result && PyList_Check(result))
        {
            Py_ssize_t size = PyList_Size(result);

            for (Py_ssize_t i = 0; i < size; ++i)
            {
                PyObject *item = PyList_GetItem(result, i);
                // std::cout << "item[" << i << "] = " << PyFloat_AsDouble(item) << std::endl;
                stateList.push_back(PyFloat_AsDouble(item));
                // std::cout << "stateList[" << i << "] = " << stateList[i] << std::endl;
            }
        }
        else
        {
            PyErr_Print();
        }

        Py_XDECREF(result);
    }
    else
    {
        PyErr_Print();
    }
    PyGILState_Release(gstate);

    return stateList;
}

std::vector<std::vector<double>> RuiWoActuator::get_joint_state()
{
    std::vector<std::vector<double>> statusList;

    gstate = PyGILState_Ensure();
    
    if (pGetJointStateMethod)
    {
        // PyObject *result = PyObject_GetAttrString(ActuatorInstance, "joint_status");
        PyObject *result = PyObject_CallObject(pGetJointStateMethod, nullptr);
        if (result && PyList_Check(result))
        {
            Py_ssize_t size = PyList_Size(result);

            for (Py_ssize_t i = 0; i < size; ++i)
            {
                std::vector<double> status;
                PyObject *item = PyList_GetItem(result, i);
                
                if (item && PyList_Check(item))
                {
                    Py_ssize_t size2 = PyList_Size(item);
                    
                    for (Py_ssize_t j = 0; j < size2; ++j)
                    {
                        PyObject *item2 = PyList_GetItem(item, j);
                        double value = PyFloat_AsDouble(item2);
                        if (!PyErr_Occurred()) // Check for errors
                            status.push_back(value);
                        else {
                            PyErr_Print();
                            // Handle error if necessary
                        }
                    }
                }

                statusList.push_back(status);
            }
        }
        else
        {
            PyErr_Print();
            // Handle error if necessary
        }

        Py_XDECREF(result); // Release the reference to the result object
    }
    else
    {
        PyErr_Print();
        // Handle error if necessary
    }

    PyGILState_Release(gstate);

    return statusList;
}

RuiWoActuator::MotorStateDataVec RuiWoActuator::get_motor_state()
{
    MotorStateDataVec motor_states;

    gstate = PyGILState_Ensure();
    if (pCheckStateMethod){
        PyObject *result = PyObject_CallObject(pCheckStateMethod, nullptr);
        if (result && PyList_Check(result)) {
            Py_ssize_t size = PyList_Size(result);
            if(size == 2) {
                PyObject *enable_motors = PyList_GetItem(result, 0);
                PyObject *disable_motors = PyList_GetItem(result, 1);
                
                auto process_motor_id = [&](PyObject *motors, RuiWoActuator::State state) {
                    if (motors && PyList_Check(motors)) {
                        Py_ssize_t size = PyList_Size(motors);
                        for (Py_ssize_t i = 0; i < size; ++i){
                            PyObject *motor_id = PyList_GetItem(motors, i);
                            int id = PyLong_AsUnsignedLongMask(motor_id);
                            if (!PyErr_Occurred()) 
                                motor_states.push_back(MotorStateData{id ,state});
                            else 
                                PyErr_Print();
                        }
                    }
                };

                process_motor_id(enable_motors, RuiWoActuator::State::Enabled);
                process_motor_id(disable_motors, RuiWoActuator::State::Disabled);
            } // if size == 2
        }
    }
    else
    {
        PyErr_Print();
    }
    PyGILState_Release(gstate);

    return motor_states;
}