#ifndef __STEPPER_H
#define __STEPPER_H

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* 引脚定义 */
#define DIR_PORT    GPIOA
#define DIR_PIN     GPIO_PIN_1

extern TIM_HandleTypeDef htim2;

/* 定时器参数 - 2MHz计数 */
#define TIM_FREQ        2000000
#define MIN_PULSE_US    20
#define MIN_ARR         40          // 2MHz下，40对应50kHz
#define MAX_FREQ        50000

/* S曲线表长度 */
#define ACCEL_LEN   200
#define DECEL_LEN   200

/* 运动状态 */
typedef enum {
    STATE_IDLE = 0,
    STATE_ACCEL,
    STATE_CRUISE,
    STATE_DECEL,
    STATE_DONE,
    STATE_ESTOP
} MotorState;

/* 电机结构体 */
typedef struct {
    uint32_t pulse_per_rev;
    uint32_t pulse_per_mm;
    uint32_t max_freq;
    uint32_t accel_steps;
    uint32_t decel_steps;
    
    uint32_t total_steps;
    uint8_t  direction;
    uint16_t return_count;
    uint16_t return_cur;
    uint8_t  return_dir;
    
    MotorState state;
    uint32_t cur_step;
    uint32_t accel_end;
    uint32_t decel_start;
    uint16_t lut_idx;
    uint32_t cruise_arr;        // ← 新增：匀速ARR，根据max_freq计算
    uint8_t  running;
    uint8_t  need_return;
} Stepper_t;

extern Stepper_t motor;

void     Stepper_Init(void);
uint8_t  Stepper_Start(uint32_t steps, uint8_t dir);
void     Stepper_Stop(void);
void     Stepper_EStop(void);
void     Stepper_OnTimer(void);
uint32_t Stepper_MmToSteps(uint32_t mm);
uint32_t Stepper_RevToSteps(uint32_t rev);

#endif
