#ifndef UART_CMD_H
#define UART_CMD_H

#include <stdint.h>

#define NUM_MOTORS 3

typedef enum {
    MOTOR_TYPE_AK60 = 60,
    MOTOR_TYPE_AK70 = 70,
} MotorType;

typedef struct {
    float pos;
    float vel;
    float kp;
    float kd;
    float tff;
    uint8_t can_id;
    MotorType type;
    uint8_t enable_pending;
} MotorState;

void         uart_cmd_init(void);
void         uart_cmd_feed(uint8_t byte);
MotorState  *uart_cmd_motor(uint8_t idx);

#endif
