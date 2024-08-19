#ifndef HAND_SDK_H
#define HAND_SDK_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <glob.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include "stark_common.pb-c.h"
#include "main_mtr_shared.pb-c.h"
#include "main_to_mtr.pb-c.h"
#include "mtr_to_main.pb-c.h"
#include "softCRC.h"
#include <pthread.h>
#include <stdbool.h>
#include <unistd.h>


typedef struct {
    int (*init)();
    int (*scan_com)(char ***com_list);
    int (*open_com)(char **com_list);
    int (*close_com)(int port);
    void (*send_com)(int port, uint8_t *buf, uint32_t len);
    int (*build_frame)(uint8_t **frame, uint8_t dst, uint8_t src, uint8_t *protobuf, uint8_t protobuf_size);
    int (*subscribe_motor)(int port, FingerStatusSampleRate sample_rate);
    int (*unsubscribe_motor)(int port);
    int (*set_motor_position)(int port, int8_t *pos);
    int (*set_motor_speed)(int port, int8_t *speed);
    int (*set_motor_current)(int port, int8_t *current);
    void (*send_position)(int port, int8_t *pos);
    void (*send_speed)(int port, int8_t *speed);
    void (*send_current)(int port, int8_t *current);
    int8_t qiangnao_port[2];
    int8_t left_value[6];
    int8_t right_value[6];
    pthread_mutex_t mutex;
    bool target_update;
} HandSDK;

extern HandSDK hand_sdk;

#endif /* HAND_SDK_H */