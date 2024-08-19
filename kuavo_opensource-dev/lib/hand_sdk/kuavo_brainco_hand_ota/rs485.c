#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <libserialport.h>
#include <stark_sdk.h>
#include <time.h>
#include <execinfo.h>
#include <signal.h>
#include <uv.h>

#define BAUD_RATE 115200

/* The ports we will use. */
struct sp_port *port = NULL;

/* Helper function for error handling. */
int check(enum sp_return result);
void printData(const uint8_t *data, int size, bool format);

// 发送数据到 RS485
int send_data(const char* device_id, const uint8_t *data, int size) {
    if (port == NULL) {
        printf("send_data, while Port is NULL.\n");
        return -1;
    }

	printf("Sending (%d bytes).\n", size);
    // printData(data, size, true);

    /* We'll allow a 1 second timeout for send. */
	unsigned int timeout = 1000;
    int result = check(sp_blocking_write(port, data, size, timeout));
	if (result == size) {
        printf("Sent %d bytes successfully.\n", size);
    } else {
        printf("Timed out, %d/%d bytes sent.\n", result, size);
    }
    return 0;
}

// 等待串口数据的辅助函数
int wait_for_data(int timeout) {
    int size = 0;
    bool more_data = false;
    time_t start = time(NULL);

    while (time(NULL) - start <= timeout) {
        int available = sp_input_waiting(port);
        if ((more_data && available == 0) || (more_data && available == size)) {
            break;
        }
        if (available > 0 && available != size) {
            more_data = true;
            size = available;
            printf("Data waiting: %d bytes.\n", size);
        }
        usleep(10000); // 等待10毫秒
    }

    return size;
}

// 接收数据的主函数
void receive_data(StarkDevice *device, int timeout) {
    if (port == NULL) {
        printf("receive_data: Port is NULL.\n");
        return;
    }

    printf("Waiting for data...\n");
    int bytes_waiting = wait_for_data(timeout); // 等待最多timeout秒钟

    if (bytes_waiting <= 0) {
        printf("No data waiting, bytes_waiting: %d.\n", bytes_waiting);
        return;
    }

    printf("Receiving %d bytes data.\n", bytes_waiting);

    // 分配缓冲区接收数据
    uint8_t *buf = (uint8_t *)malloc(bytes_waiting);
    if (buf == NULL) {
        perror("malloc failed");
        return;
    }

    // 设置超时时间为100毫秒
    int result = sp_blocking_read(port, buf, bytes_waiting, 100);
    if (result > 0) {
        printf("Received %d bytes successfully.\n", result);
        printData(buf, result, true);
        stark_did_receive_data(device, buf, result);
    } else {
        printf("Error receiving data: %s\n", sp_last_error_message());
    }

    free(buf); // 释放接收缓冲区
}

void open_serial(const char* serial_port) {
    /* Open and configure port. */
	check(sp_get_port_by_name(serial_port, &port));

	printf("Opening port.\n");
	check(sp_open(port, SP_MODE_READ_WRITE));

	printf("Setting port to %u 8N1, no flow control.\n", BAUD_RATE);
	check(sp_set_baudrate(port, BAUD_RATE));
	check(sp_set_bits(port, 8));
	check(sp_set_parity(port, SP_PARITY_NONE));
	check(sp_set_stopbits(port, 1));
	check(sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));

    sp_flush(port, SP_BUF_BOTH);
}

void close_serial() {
    if (port == NULL) return;
    printf("Closing port.\n");
	check(sp_close(port));
	sp_free_port(port);
}

void on_error(const char *device_id, int error) {
    if (device_id == NULL) {
        printf("on_error, error: %d\n", error);
        return;
    }
    printf("on_error, device_id: %s, error: %d\n", device_id, error);
}

void on_serialport_cfg(const char *device_id, SerialPortCfg *cfg) {
    printf("on_serialport_cfg, device_id: %s, serial_device_id: %d, baudrate: %d\n", device_id, cfg->serial_device_id, cfg->baudrate);
}

void on_hand_type(const char *device_id, int hand_type) {
    printf("on_hand_type, device_id: %s, hand_type: %d\n", device_id, hand_type);
}

void on_force_level(const char *device_id, int force_level) {
    printf("on_force_level, device_id: %s, force_level: %d\n", device_id, force_level);
}

void on_motorboard_info(const char *device_id, MotorboardInfo *info) {
    printf("on_motorboard_info, device_id: %s, hand_type: %d, sn: %s, fw_version: %s\n", device_id, info->hand_type, info->sn, info->fw_version);
}

void on_voltage(const char *device_id, float voltage) {
    printf("on_voltage, device_id: %s, voltage: %f\n", device_id, voltage);
}

void on_limit_current(const char *device_id, int limit_current) {
    printf("on_limit_current, device_id: %s, limit_current: %d\n", device_id, limit_current);
}

void on_finger_status(const char *device_id, FingerStatusData *finger_status) {
    printf("on_finger_status, device_id: %s, finger_status: %p\n", device_id, finger_status);
}

void on_motor_status(const char *device_id, MotorStatusData *motor_status) {
    printf("on_motor_status, device_id: %s, motor_status: %p\n", device_id, motor_status);
}

void on_button_event(const char *device_id, ButtonPressEvent *event) {
    printf("on_button_event, device_id: %s, button_event: %p\n", device_id, event);
}

void timer_close_cb(uv_handle_t* handle){
    if(handle){
        free(handle);
        handle = NULL;
    }
}

void loop_timer_cb(uv_timer_t* handle) {
    // printf("Timer callback called\n");
}

void start_loop() {
    uv_loop_t *loop = uv_default_loop();

    // 分配内存给定时器句柄
    uv_timer_t *timer_handle = (uv_timer_t*)malloc(sizeof(uv_timer_t));
    if (timer_handle == NULL) {
        fprintf(stderr, "Failed to allocate memory for timer handle\n");
        return;
    }

    // 初始化定时器
    uv_timer_init(loop, timer_handle);

    // 启动定时器，每2000ms触发一次回调
    int result = uv_timer_start(timer_handle, loop_timer_cb, 3000, 0);
    if (result != 0) {
        fprintf(stderr, "Failed to start timer: %s\n", uv_strerror(result));
        free(timer_handle);
        return;
    }

    // 运行事件循环
    uv_run(loop, UV_RUN_DEFAULT);

    // 停止和关闭定时器
    uv_timer_stop(timer_handle);
    uv_close((uv_handle_t*)timer_handle, timer_close_cb);

    printf("Event loop finished\n");
}

void handler(int sig) {
    void *array[10];
    size_t size;

    // 获取堆栈帧
    size = backtrace(array, 10);

    // 打印所有堆栈帧到 stderr
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <serial_port>\n", argv[0]);
        return 1;
    }
    const char *serial_port = argv[1];

    signal(SIGSEGV, handler);   // Install our handler for SIGSEGV (segmentation fault)
    signal(SIGABRT, handler);   // Install our handler for SIGABRT (abort signal)
    printf("----------------------Main begin----------------------\n");
    printf("StarkSDK version: v%s\n", stark_get_sdk_version());
    stark_set_write_data_callback(send_data);

    open_serial(serial_port);

    const int finger_positions[6] = {100, 100, 100, 100, 100, 100};
    const int finger_speeds[6] = {30, 30, 30, 30, -30, 30};
    stark_group_set_finger_speeds(finger_speeds);
    stark_group_set_finger_positions(finger_positions);
    
    const char* device_id = "Stark_OK";
    const int serial_device_id = 2;
    StarkDevice* device = stark_create_serial_device(device_id, serial_device_id);
    stark_set_error_callback(device, on_error);
    stark_set_finger_positions(device, finger_positions);

    if (strstr(serial_port, "L") != NULL) {
        factory_set_hand_type(device, "stark-level1", MEDIUM_LEFT);
    } else if (strstr(serial_port, "R") != NULL) {
        factory_set_hand_type(device, "stark-level1", MEDIUM_RIGHT);
    }

    stark_get_motorboard_info(device, on_motorboard_info);
    receive_data(device, 1);

    // 延时确保前面运动能到位置
    sleep(3);

    const int finger_positions1[6] = {0, 0, 0, 0, 0, 0};
    const int finger_speeds1[6] = {30, 30, 30, 30, -30, 30};
    stark_group_set_finger_speeds(finger_speeds1);
    stark_group_set_finger_positions(finger_positions1);

    start_loop();
    close_serial();
    return 0;
}

/* Helper function for error handling. */
int check(enum sp_return result)
{
	/* For this example we'll just exit on any error by calling abort(). */
	char *error_message;

	switch (result) {
	case SP_ERR_ARG:
		printf("Error: Invalid argument.\n");
		abort();
	case SP_ERR_FAIL:
		error_message = sp_last_error_message();
		printf("Error: Failed: %s\n", error_message);
		sp_free_error_message(error_message);
		abort();
	case SP_ERR_SUPP:
		printf("Error: Not supported.\n");
		abort();
	case SP_ERR_MEM:
		printf("Error: Couldn't allocate memory.\n");
		abort();
	case SP_OK:
	default:
		return result;
	}
}

void printData(const uint8_t *data, int size, bool format) {
    for (int i = 0; i < size; i++) {
        if (format) {
            printf("0x%02X", data[i]); // 打印当前字节的十六进制表示
        } else {
            printf("%02X", data[i]); // 打印当前字节的十六进制表示
        }
        if (i != size - 1) {
            if (format) printf(","); // 除了最后一个字节外，每个字节后面加逗号
        } else {
            printf("\n"); // 最后一个字节后面加换行符
        }
    }
}
