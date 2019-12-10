#ifndef _BLDC_H
#define _BLDC_H

#include "stm32f405xx.h"
#include "tim.h"
#include "adc.h"
#include "stdint.h"

typedef struct
{
    uint8_t hallUVW[3];        //1读取三个HALL的状态
    uint8_t hallState;         //2当前的HALL的状态
    uint8_t oldHallState;      //3历史HALL状态
    int32_t stepAngleError;    //4步进角度误差
    int32_t stepAngle;         //5步进角度
    int32_t stepAngleFilter;   //6步进角度滤波器
    uint16_t speedCount;       //7速度计数值
    uint16_t speedCountFilter; //8速度计数滤波值
    uint16_t speedCountOld;    //9速度计数历史值
    uint32_t speedCoeff;       //a速度系数
    uint8_t poles;             //b电机极对数
    uint8_t moveState;         //c电机旋转状态
    uint16_t speedRPMF;        //d电机旋转速度滤波
    uint16_t speedRPM;         //e电机旋转速度
    uint16_t filterEdge;
    uint16_t filterCount;
    uint16_t filterCountF;
    uint16_t queueStatus[3];
} HALL, *pHALL;

typedef struct
{
    uint8_t controlMode;   //1control mode
    uint8_t tripFlagDMC;   //2over current flag
    uint8_t driveCar;      //3start drive
    uint8_t oldDriveCar;   //4
    uint8_t clearPWMtripz; //5
    uint8_t runMode;       //6
    uint16_t switchCount;  //7
    uint8_t startOrder;    //8
    uint16_t duty;         //9
    uint16_t speedCount;   //a
    uint16_t currentCount;
    uint16_t aimSpeed;
    uint32_t aimDuty;
    uint32_t INVERSION;
    uint32_t temp;
} STATE, *pSTATE;

typedef struct
{
    float ref;       // Input: Reference input
    float fdb;       // Input: Feedback input
    float err;       // Variable: Error
    float kp;        // Parameter: Proportional gain
    float up;        // Variable: Proportional output
    float ui;        // Variable: Integral output
    float ud;        // Variable: Derivative output
    float outPreSat; // Variable: Pre-saturated output
    float outMax;    // Parameter: Maximum output
    float outMin;    // Parameter: Minimum output
    float out;       // Output: PID output
    float satErr;    // Variable: Saturated difference
    float ki;        // Parameter: Integral gain
    float kc;        // Parameter: Integral correction gain
    float kd;        // Parameter: Derivative gain
    float up1;       // History: Previous proportional output
    float ui_1;
    float outF;
} PIDREG_T, *p_PIDREG_T;

typedef struct
{
    int32_t busCurrF;          //DC Bus  Current
    int32_t busCurr;           //DC Bus  Current
    int32_t phaseWCurr;        //Phase U Current
    int32_t phaseUCurr;        //Phase U Current
    int32_t phaseVCurr;        //Phase V Current
    int32_t busVoltage;        //Bus  Voltage
    int32_t rpSpeedVoltage;    //RP1_Voltage
    int32_t offsetPhaseU_Curr; //Phase U Current
    int32_t offsetPhaseV_Curr; //Phase V Current
    int32_t offsetPhaseW_Curr; //Phase V Current
    int32_t coeffFilterK1;
    int32_t coeffFilterK2;
} ADCSamp, *pADCSamp;

#define HALL_DEFAULTS                                              \
    {                                                              \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0 \
    }
#define STATE_DEFAULTS                              \
    {                                               \
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 \
    }
#define PIDREG_T_DEFAULTS                                    \
    {                                                        \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 \
    }
#define ADCSAMP_DEFAULTS                    \
    {                                       \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 24 \
    }
#define MOTOR_POLES 2
#define HALL_U_STATUS HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)
#define HALL_V_STATUS HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)
#define HALL_W_STATUS HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)

#define __IQMPY(A, B) ((A) * (B))

#define FIRSTORDER_LPF_CACL(Xn, Yn_1, a) \
    do                                   \
    {                                    \
        Yn_1 = (1 - a) * Yn_1 + a * Xn;  \
    } while (0)

#define UP16LIMIT(var, max, min)               \
    do                                         \
    {                                          \
        (var) = (var) > (max) ? (max) : (var); \
        (var) = (var) < (min) ? (min) : (var); \
    } while (0)

#define PID_CALC(v)                       \
    do                                    \
    {                                     \
    v.err = v.ref - v.fdb;                \
    v.up = __IQMPY(v.kp, v.err);          \
    v.ui = v.ui + __IQMPY(v.ki, v.up);    \
    UP16LIMIT(v.ui, v.outMax, v.outMin);  \
    v.ud = v.kd * (v.up - v.up1);         \
    v.out = v.up + v.ui + v.ud;           \
    UP16LIMIT(v.out, v.outMax, v.outMin); \
    v.up1 = v.up;                         \
    }while(0)

#define MOS_Q41PWM                                                    \
    do                                                                \
    {                                                                 \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, stateContr.duty); \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);               \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);               \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);        \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);        \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);          \
    } while (0)

#define MOS_Q16PWM                                                    \
    do                                                                \
    {                                                                 \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, stateContr.duty); \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);               \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);               \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);        \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);        \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);          \
    } while (0)

#define MOS_Q63PWM                                                    \
    do                                                                \
    {                                                                 \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);               \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, stateContr.duty); \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);               \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);        \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);        \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);          \
    } while (0)

#define MOS_Q32PWM                                                    \
    do                                                                \
    {                                                                 \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);               \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, stateContr.duty); \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);               \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);        \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);        \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);          \
    } while (0)

#define MOS_Q25PWM                                                    \
    do                                                                \
    {                                                                 \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);               \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);               \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, stateContr.duty); \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);        \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);        \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);          \
    } while (0)

#define MOS_Q54PWM                                                    \
    do                                                                \
    {                                                                 \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);               \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);               \
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, stateContr.duty); \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);        \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);        \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);          \
    } while (0)

#define START_MOTOR                                             \
    do                                                          \
    {                                                           \
        TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_ENABLE); \
        TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_ENABLE); \
        TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_3, TIM_CCx_ENABLE); \
    } while (0)

#define STOP_MOTOR                                               \
    do                                                           \
    {                                                            \
        TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_DISABLE); \
        TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_DISABLE); \
        TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_3, TIM_CCx_DISABLE); \
    } while (0)

#define TEST_MANUELL 0
#define LOOP 1                          //1:open loop 2:speed loop 3:speed and current loop
#define PWM_FREQ ((uint16_t)18)
#define TORQUE 0x00ff
#define FILTER_LONG 0xffff
#define SPEED_KP 0.05
#define SPEED_KI 0.01
#define SPEED_KC 0.01
#define CURRENT_KP 2
#define CURRENT_KI 0.2
#define CURRENT_KC 0.01

void PID_Init(void);
void threeHallPara_Init(void);
uint8_t uHALLEdge(uint8_t val);
void hallSwitch(void);
void offsetCurrentRead(void);
void HALL_ADCSample(void);
void motorTestProgram(void);

#endif