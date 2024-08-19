#ifndef STARK_SDK_H
#define STARK_SDK_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef _WIN32
#ifdef BUILDING_STARK_SHARED
#define SDK_EXTERN __declspec(dllexport)
#elif USING_STARK_SHARED
#define SDK_EXTERN __declspec(dllimport)
#else
#define SDK_EXTERN
#endif
#else
#define SDK_EXTERN
#endif

#if __APPLE__
#include <TargetConditionals.h>
#endif

#include <stdbool.h>
#include <stdint.h>

// CFFI_DEF_START

// ENUM
// ==============================================================================
typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO = 1,
    LOG_LEVEL_WARN = 2,
    LOG_LEVEL_ERROR = 3,
    LOG_LEVEL_NONE = 4,
} StarkLogLevel;

typedef enum {
    STARK_ERROR_NONE = 0,
    STARK_ERROR_UNKNOWN = -1,
    STARK_ERROR_INVALID_PARAMS = -2,
    STARK_ERROR_INVALID_DATA = -3,
    STARK_ERROR_PARSE_FAILED = -4,

    STARK_ERROR_SYSTEM_IS_BUSY = -11,
} StarkError;

typedef enum {
    MEDIUM_RIGHT = 1,
    MEDIUM_LEFT = 2,
    SMALL_RIGHT = 3,
    SMALL_LEFT = 4
} StarkHandType;

typedef enum {
    STARK_FORCE_LEVEL_SMALL = 1,
    STARK_FORCE_LEVEL_NORMAL = 2,
    STARK_FORCE_LEVEL_FULL = 3,
} StarkForceLevel;

typedef enum {
    FINGERID_THUMB = 1,
    FINGERID_THUMB_AUX = 2,
    FINGERID_INDEX = 3,
    FINGERID_MIDDLE = 4,
    FINGERID_RING = 5,
    FINGERID_PINKY = 6
} StarkFingerId;

typedef enum {
    MOTOR_IDLE = 0,           /* 电机空闲，表示电机未运行 */
    MOTOR_RUNNING = 1,        /* 电机运行中，表示电机正在运行 */
    MOTOR_PROTECTED_ZONE = 2, /* 临界保护区，表示电机正在运行，启动中或即将停止 */
    MOTOR_STALL = 3           /* 电机堵转 */
} MotroState;

typedef enum {
    LED_MODE_SHUTDOWN = 1,   // turn-off LED
    LED_MODE_KEEP = 2,
    LED_MODE_BLINK = 3,      // 1Hz freq on-off cycle, 50% duty cycle
    LED_MODE_ONE_SHOT = 4,   // on-100ms, then off, no repeat, if repeatedly receive this, time counter will be reset at each time received (e.g., repeatedly receive interval <= 100ms equal always on)
    LED_MODE_BLINK0_5HZ = 5, // 0.5Hz freq on-off cycle, 50% duty cycle
    LED_MODE_BLINK2HZ = 6    // 2Hz freq on-off cycle, 50% duty cycle
} StarkLedMode;

typedef enum {
    LED_COLOR_UNCHANGED = 0, // not equal to OFF, since majorly use LedMode to turn-off LED.
    LED_COLOR_R = 1,
    LED_COLOR_G = 2,
    LED_COLOR_RG = 3,
    LED_COLOR_B = 4,
    LED_COLOR_RB = 5,
    LED_COLOR_GB = 6,
    LED_COLOR_RGB = 7
} StarkLedColor;

typedef enum {
    PRESSING = 1,
    NOT_PRESSING = 2,
} StarkPressState;

typedef enum {
    STARK_DFU_STATE_IDLE = 0,
    STARK_DFU_STATE_ENABLING = 1,
    STARK_DFU_STATE_STARTED = 2,
    STARK_DFU_STATE_TRANSFER = 3,
    STARK_DFU_STATE_COMPLETED = 4,
    STARK_DFU_STATE_ABORTED = 5,
} StarkDfuState;

typedef enum {
    STARK_ACTION_CMD_SET_START = 1,
    STARK_ACTION_CMD_SET_FINISH = 2,
    STARK_ACTION_CMD_READ_START = 3,
    STARK_ACTION_CMD_READ_FINISH = 4,
    STARK_ACTION_CMD_SAVE = 5,
    STARK_ACTION_CMD_RUN = 6,
} StarkActionCmd;

// Model
// ==============================================================================

// StarkDevice and related struct definitions
typedef struct StarkDevice StarkDevice;

typedef struct {
    /* proto3协议，设备ID范围： 10~254，默认为10，其中254为广播地址 */
    /* Modbus协议，设备ID范围： 0~254， 默认为1， 其中0为广播地址, 广播仅适用于控制指令 */
    int serial_device_id;  /* 设备ID */ 
    int baudrate;          /* 波特率 */ 
} SerialPortCfg;

typedef struct {
    int hand_type;       /* 设备类型: 左右、大小 StarkHandType */ 
    char sn[20];         /* 序列号 */ 
    char fw_version[20]; /* 固件版本 */ 
} MotorboardInfo;

typedef struct {
    uint8_t finger_positions[6]; // 0 ~ 100 for each finger. 0 for fully open, 100 for fully close
    int8_t finger_speeds[6];     // -100 ~ +100 for each finger. 0 for stop, positive number for close finger, negative number for open finger.
    int8_t finger_currents[6];   // -100 ~ +100 for each finger. Fraction of max motor current, absolute number.
    int8_t finger_pwms[6];       // -100 ~ +100 for each motor.
} StarkFingerStatus;

typedef struct {
    int n_finger_status;
    StarkFingerStatus ** finger_status;
} FingerStatusData;

typedef struct {
    int n_motor_status;
    int * motor_status;
} MotorStatusData;

typedef struct {
    int led_color; // 手背灯颜色 StarkLedColor
    int led_mode;  // 手背灯模式 StarkLedMode
} LedInfo;

typedef struct {
    int timestamp;
    int button_id;
    int press_status; // StarkPressState
} ButtonPressEvent;

typedef struct {
    uint16_t duration; // ms
    uint16_t index;
    uint16_t motor_positions[6]; 
    int16_t motor_speeds[6];
    int16_t motor_strengths[6];
} ActionSequenceData;

typedef struct {
    int action_id;
    int action_num;
    ActionSequenceData ** data;
} ActionSequence;

// API
// ==============================================================================

// Callbacks
typedef int (*WriteDataCB)(const char *device_id, const uint8_t *data, int size);
typedef void (*StarkLogCB)(int level, const char *msg);
typedef void (*StarkFloatValueCB)(const char *device_id, float value);
typedef void (*StarkIntValueCB)(const char *device_id, int value);
typedef void (*StarkValue2CB)(const char *device_id, int value1, int value2);
typedef void (*StarkValuesCB)(const char *device_id, int *values);
typedef void (*StarkUint16ValuesCB)(const char *device_id, uint16_t *values);
typedef void (*SerialPortCfgCB)(const char *device_id, SerialPortCfg *cfg);
typedef void (*MotorboardInfoCB)(const char *device_id, MotorboardInfo *info);
typedef void (*FingerStatusCB)(const char *device_id, FingerStatusData *data);
typedef void (*MotroStatusCB)(const char *device_id, MotorStatusData *data);
typedef void (*ActionSequenceCB)(const char *device_id, ActionSequence* data);
typedef void (*LedInfoCB)(const char *device_id, LedInfo *info);
typedef void (*ButtonEventCB)(const char *device_id, ButtonPressEvent *event);
typedef void (*DfuReadCB)(const char *device_id);
typedef void (*DfuStateCB)(const char *device_id, int state);
typedef void (*DfuProgressCB)(const char *device_id, float progress);

// Global
SDK_EXTERN const char *stark_get_sdk_version();

// logger
SDK_EXTERN void stark_set_log_level(StarkLogLevel logLevel);
SDK_EXTERN void stark_set_log_callback(StarkLogCB cb);
SDK_EXTERN void stark_log(StarkLogLevel logLevel, const char *format, ...);
SDK_EXTERN const char *stark_err_code_to_msg(int err_code);
typedef void (*StarkErrorCB)(const char *device_id, int error);
SDK_EXTERN int stark_set_error_callback(StarkDevice *device, StarkErrorCB cb);

// 0. serialport cfg getters & setters
// NOTE: device will reboot after setting serialport cfg, device id or baudrate
SDK_EXTERN void stark_get_serialport_cfg(StarkDevice *device, SerialPortCfgCB cb);
SDK_EXTERN void stark_set_serial_baudrate(StarkDevice *device, int baudrate); // baudrate: 115200, 57600, 19200
SDK_EXTERN void stark_set_serial_device_id(StarkDevice *device, int device_id); // device_id: 10 ~ 253, 254 for broadcast

// 1. StarkDevice create
SDK_EXTERN StarkDevice *stark_create_serial_device(const char *uuid, const int device_id);
SDK_EXTERN StarkDevice *stark_create_modbus_device(const char *uuid, const int device_id);
SDK_EXTERN StarkDevice *stark_get_device(const char *uuid);

// 2. Device read & write
// serial device
SDK_EXTERN int stark_did_receive_data(StarkDevice *device, const uint8_t *data, int size);
SDK_EXTERN void stark_set_write_data_callback(WriteDataCB cb);

// modbus device
typedef int (*ModbusWriteCB)(const char *device_id, const uint16_t *data, int register_address, int count);
typedef uint16_t * (*ModbusReadValuesCB)(const char *device_id, int register_address, int count);
typedef uint16_t (*ModbusReadValueCB)(const char *device_id, int register_address);
SDK_EXTERN void modbus_set_write_registers_callback(StarkDevice *device, ModbusWriteCB cb);
SDK_EXTERN void modbus_set_read_holding_register_callback(StarkDevice *device, ModbusReadValueCB cb);
SDK_EXTERN void modbus_set_read_holding_registers_callback(StarkDevice *device, ModbusReadValuesCB cb);
SDK_EXTERN void modbus_set_read_input_register_callback(StarkDevice *device, ModbusReadValueCB cb);
SDK_EXTERN void modbus_set_read_input_registers_callback(StarkDevice *device, ModbusReadValuesCB cb);

// 3. Motorboard info getters
SDK_EXTERN void stark_get_hand_type(StarkDevice *device, StarkIntValueCB cb);
SDK_EXTERN void stark_get_motorboard_info(StarkDevice *device, MotorboardInfoCB cb);

// 4. Motorboard Voltage getters
SDK_EXTERN void stark_get_voltage(StarkDevice *device, StarkFloatValueCB cb);

// 5. Max Current getters & setters
SDK_EXTERN void stark_get_max_current(StarkDevice *device, StarkIntValueCB cb);
SDK_EXTERN void stark_set_max_current(StarkDevice *device, int max_current);

// 6. ForceLevel getters & setters
SDK_EXTERN void stark_get_force_level(StarkDevice *device, StarkIntValueCB cb);
SDK_EXTERN void stark_set_force_level(StarkDevice *device, int force_level); // StarkForceLevel
SDK_EXTERN void stark_get_turbo_mode(StarkDevice *device, StarkIntValueCB cb);
SDK_EXTERN void stark_set_turbo_mode(StarkDevice *device, int turbo_mode);

// 7. Finger status getters & setters
SDK_EXTERN void stark_get_finger_status(StarkDevice *device, FingerStatusCB cb);
SDK_EXTERN void stark_get_motor_status(StarkDevice *device, MotroStatusCB cb);
SDK_EXTERN void stark_reset_finger_positions(StarkDevice *device);
SDK_EXTERN void stark_set_finger_position(StarkDevice *device, int finger_position);
SDK_EXTERN void stark_set_finger_positions(StarkDevice *device, const int finger_positions[6]);
SDK_EXTERN void stark_set_finger_speed(StarkDevice *device, int finger_speed);
SDK_EXTERN void stark_set_finger_speeds(StarkDevice *device, const int finger_speeds[6]);
SDK_EXTERN void stark_group_set_finger_positions(const int finger_positions[6]);
SDK_EXTERN void stark_group_set_finger_speeds(const int finger_speeds[6]);

// 7.1 Modbus-RTU, Finger status getters & setters
SDK_EXTERN void stark_get_finger_positions(StarkDevice *device, StarkUint16ValuesCB cb);
SDK_EXTERN void stark_get_finger_speeds(StarkDevice *device, StarkUint16ValuesCB cb);
SDK_EXTERN void stark_get_finger_currents(StarkDevice *device, StarkUint16ValuesCB cb);

// 8. LED
SDK_EXTERN void stark_get_led_info(StarkDevice *device, LedInfoCB cb);
SDK_EXTERN void stark_set_led_info(StarkDevice *device, int led_mode, int led_color); // StarkLedMode, StarkLedColor

// 9. Button event
SDK_EXTERN void stark_get_button_event(StarkDevice *device, ButtonEventCB cb);

// 10. Action sequences
// transfer action sequence to device memory
SDK_EXTERN void stark_transfer_action_sequence(StarkDevice *device, int action_id, int action_num, uint16_t ** action_sequence);
// save action sequence to device Flash
SDK_EXTERN void stark_save_action_sequence(StarkDevice *device, int action_id);
// run action sequence
SDK_EXTERN void stark_run_action_sequence(StarkDevice *device, int action_id);
// get action sequence detail with action_id
SDK_EXTERN void stark_get_action_sequence(StarkDevice *device, int action_id, ActionSequenceCB cb);

// Factory Methods
SDK_EXTERN void factory_set_device_sn(StarkDevice *device, const char *operation_key, const char *sn);
SDK_EXTERN void factory_set_hand_type(StarkDevice *device, const char *operation_key, int hand_type);

// DFU Methods
SDK_EXTERN const char* stark_dfu_state_to_string(int state);
SDK_EXTERN void stark_set_dfu_read_callback(StarkDevice *device, DfuReadCB cb);
SDK_EXTERN void stark_set_dfu_state_callback(StarkDevice *device, DfuStateCB cb);
SDK_EXTERN void stark_set_dfu_progress_callback(StarkDevice *device, DfuProgressCB cb);
SDK_EXTERN void stark_set_dfu_cfg(StarkDevice *device,
                                  uint8_t dfu_enabling_delay    /* = 8 */,    /* Delay time for enabling DFU (1~30 seconds) in */
                                  uint8_t dfu_enabling_interval /* = 10 */,   /* Retry interval for enabling DFU (1~30 seconds) */
                                  uint8_t dfu_applying_delay    /* = 10 */);  /* Waiting time for applying new firmware (1~30 seconds) */
SDK_EXTERN void stark_start_dfu(StarkDevice *device, const char *dfu_file_path);
SDK_EXTERN void stark_abort_dfu(StarkDevice *device);

// Test Methods
SDK_EXTERN int hello(StarkDevice *device);

// CFFI_DEF_END

#ifdef __cplusplus
}
#endif
#endif
