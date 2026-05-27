#include "stepper.h"

Stepper_t motor;

/* 加速段查找表 - 2MHz版本 */
static const uint16_t accel_lut[ACCEL_LEN] = {
     9999, 9996, 9978, 9932, 9844, 9704, 9502, 9240, 8914, 8532, 8102, 7638,
     7152, 6656, 6164, 5684, 5224, 4790, 4384, 4008, 3664, 3350, 3062, 2804,
     2568, 2356, 2166, 1992, 1836, 1696, 1568, 1452, 1348, 1254, 1166, 1088,
     1016,  952,  892,  836,  786,  740,  698,  658,  622,  590,  558,  530,
      504,  478,  456,  434,  414,  396,  378,  362,  346,  332,  318,  306,
      294,  282,  272,  262,  252,  244,  234,  226,  218,  212,  204,  198,
      192,  186,  180,  176,  170,  166,  160,  156,  152,  148,  144,  140,
      138,  134,  130,  128,  124,  122,  118,  116,  114,  112,  108,  106,
      104,  102,  100,   98,   96,   94,   94,   92,   90,   88,   86,   86,
       84,   82,   82,   80,   78,   78,   76,   76,   74,   74,   72,   72,
       70,   70,   68,   68,   68,   66,   66,   64,   64,   64,   62,   62,
       62,   62,   60,   60,   60,   58,   58,   58,   58,   56,   56,   56,
       56,   56,   54,   54,   54,   54,   54,   54,   52,   52,   52,   52,
       52,   52,   52,   52,   50,   50,   50,   50,   50,   50,   50,   50,
       50,   50,   50,   50,   50,   48,   48,   48,   48,   48,   48,   48,
       48,   48,   48,   48,   48,   48,   48,   48,   48,   48,   48,   48,
       48,   48,   48,   48,   48,   48,   48,   48,
};

/* 减速段查找表 - 2MHz版本 */
static const uint16_t decel_lut[DECEL_LEN] = {
       48,   48,   48,   48,   48,   48,   48,   48,   48,   48,   48,   48,
       48,   48,   48,   48,   48,   48,   48,   48,   48,   48,   48,   48,
       48,   48,   48,   50,   50,   50,   50,   50,   50,   50,   50,   50,
       50,   50,   50,   50,   52,   52,   52,   52,   52,   52,   52,   52,
       54,   54,   54,   54,   54,   54,   56,   56,   56,   56,   56,   58,
       58,   58,   58,   60,   60,   60,   62,   62,   62,   62,   64,   64,
       64,   66,   66,   68,   68,   68,   70,   70,   72,   72,   74,   74,
       76,   76,   78,   78,   80,   82,   82,   84,   86,   86,   88,   90,
       92,   94,   94,   96,   98,  100,  102,  104,  106,  108,  112,  114,
      116,  118,  122,  124,  128,  130,  134,  138,  140,  144,  148,  152,
      156,  160,  166,  170,  176,  180,  186,  192,  198,  204,  212,  218,
      226,  234,  244,  252,  262,  272,  282,  294,  306,  318,  332,  346,
      362,  378,  396,  414,  434,  456,  478,  504,  530,  558,  590,  622,
      658,  698,  740,  786,  836,  892,  952, 1016, 1088, 1166, 1254, 1348,
     1452, 1568, 1696, 1836, 1992, 2166, 2356, 2568, 2804, 3062, 3350, 3664,
     4008, 4384, 4790, 5224, 5684, 6164, 6656, 7152, 7638, 8102, 8532, 8914,
     9240, 9502, 9704, 9844, 9932, 9978, 9996, 9999,
};

static uint16_t LimitARR(uint16_t arr)
{
    if (arr < MIN_ARR) arr = MIN_ARR;
    return arr;
}

static void SetPWM(uint16_t arr)
{
    arr = LimitARR(arr);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, arr / 2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

/* 根据频率计算ARR */
static uint32_t FreqToArr(uint32_t freq)
{
    if (freq == 0) freq = 200;
    if (freq > MAX_FREQ) freq = MAX_FREQ;
    return (TIM_FREQ / freq) - 1;
}

void Stepper_Init(void)
{
    memset(&motor, 0, sizeof(Stepper_t));
    motor.pulse_per_rev = 200;
    motor.pulse_per_mm  = 200;
    motor.max_freq      = 50000;
    motor.accel_steps   = 200;
    motor.decel_steps   = 200;
    motor.cruise_arr    = FreqToArr(motor.max_freq);  // 根据max_freq计算
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
    motor.state = STATE_IDLE;
}

uint32_t Stepper_MmToSteps(uint32_t mm)
{
    return (motor.pulse_per_mm == 0) ? 0 : mm * motor.pulse_per_mm;
}

uint32_t Stepper_RevToSteps(uint32_t rev)
{
    return (motor.pulse_per_rev == 0) ? 0 : rev * motor.pulse_per_rev;
}

uint8_t Stepper_Start(uint32_t steps, uint8_t dir)
{
    if (motor.running) return 1;
    if (steps == 0)    return 2;

    uint32_t half = steps / 2;
    if (motor.accel_steps > ACCEL_LEN) motor.accel_steps = ACCEL_LEN;
    if (motor.decel_steps > DECEL_LEN) motor.decel_steps = DECEL_LEN;
    if (motor.accel_steps > half)      motor.accel_steps = half;
    if (motor.decel_steps > half)      motor.decel_steps = half;

    /* 重新计算匀速ARR */
    motor.cruise_arr = FreqToArr(motor.max_freq);

    motor.total_steps = steps;
    motor.cur_step    = 0;
    motor.accel_end   = motor.accel_steps;
    motor.decel_start = steps - motor.decel_steps;
    motor.lut_idx     = 0;

    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    motor.direction = dir;
    HAL_Delay(5);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim2);
    SetPWM(accel_lut[0]);

    motor.state   = STATE_ACCEL;
    motor.running = 1;
    return 0;
}

void Stepper_Stop(void)
{
    if (!motor.running) return;
    motor.state       = STATE_DECEL;
    motor.decel_start = motor.cur_step;
    motor.decel_steps = motor.total_steps - motor.cur_step;
    if (motor.decel_steps > DECEL_LEN) motor.decel_steps = DECEL_LEN;
    motor.lut_idx = 0;
}

void Stepper_EStop(void)
{
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Base_Stop_IT(&htim2);
    motor.state   = STATE_ESTOP;
    motor.running = 0;
}

void Stepper_OnTimer(void)
{
    if (!motor.running) return;

    motor.cur_step++;

    switch (motor.state) {
        case STATE_ACCEL:
            if (motor.cur_step < motor.accel_end) {
                motor.lut_idx = motor.cur_step;
                if (motor.lut_idx >= ACCEL_LEN) motor.lut_idx = ACCEL_LEN - 1;
                SetPWM(accel_lut[motor.lut_idx]);
            } else {
                motor.state = STATE_CRUISE;
                SetPWM(motor.cruise_arr);   // ← 使用动态计算的ARR
            }
            break;

        case STATE_CRUISE:
            if (motor.cur_step >= motor.decel_start) {
                motor.state  = STATE_DECEL;
                motor.lut_idx = 0;
                SetPWM(decel_lut[0]);
            }
            break;

        case STATE_DECEL:
            motor.lut_idx++;
            if (motor.lut_idx < motor.decel_steps && motor.lut_idx < DECEL_LEN) {
                SetPWM(decel_lut[motor.lut_idx]);
            } else {
                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                HAL_TIM_Base_Stop_IT(&htim2);
                motor.state   = STATE_DONE;
                motor.running = 0;
                if (motor.return_count > 0 && motor.return_cur < motor.return_count) {
                    motor.need_return = 1;
                }
            }
            break;

        default:
            break;
    }
}
