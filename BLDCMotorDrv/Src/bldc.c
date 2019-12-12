#include "bldc.h"
#include "myprint.h"

volatile uint16_t ADC_DualConvertValTab[5];

HALL hallTree;
STATE stateContr;
PIDREG_T piSpd = PIDREG_T_DEFAULTS;
PIDREG_T piICurr = PIDREG_T_DEFAULTS;
ADCSamp ADCSampPare = ADCSAMP_DEFAULTS;

void PID_Init(void)
{
    piSpd.kp = SPEED_KP;
    piSpd.ki = SPEED_KI;
    piSpd.kc = SPEED_KC;
    piSpd.outMax = 3000;
    piSpd.outMin = 30;
    piSpd.ref = 0;
    piICurr.kp = CURRENT_KP;
    piICurr.ki = CURRENT_KI;
    piICurr.kc = CURRENT_KC;
    piICurr.outMin = 30;
    piICurr.outMax = 3000;
    piICurr.ref = 0;
}

void threeHallPara_Init(void)
{
    hallTree.poles = MOTOR_POLES;
    hallTree.speedCoeff = 1600 * 60 / hallTree.poles / 2;
}

uint8_t uHALLEdge(uint8_t val)
{
    static uint8_t oldVal = 0;
    if (oldVal != val)
    {
        oldVal = val;
        if (val == 0)
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    return 2;
}

void hallSwitch(void)
{
    hallTree.hallUVW[0] = HALL_U_STATUS & 0x01;
    hallTree.hallUVW[1] = HALL_V_STATUS & 0x01;
    hallTree.hallUVW[2] = HALL_W_STATUS & 0x01;
    hallTree.filterEdge = uHALLEdge(hallTree.hallUVW[0]);
    if (hallTree.filterEdge == 1)
    {
        FIRSTORDER_LPF_CACL(hallTree.filterCount, hallTree.filterCountF, 0.1379);
        hallTree.speedRPM = hallTree.speedCoeff / hallTree.filterCountF;
        FIRSTORDER_LPF_CACL(hallTree.speedRPM, hallTree.speedRPMF, 0.2379);
        hallTree.filterCount = 0;
    }
    if (hallTree.filterEdge == 0)
    {
        hallTree.filterCount = 0;
    }
    if (hallTree.filterEdge == 2)
    {
        hallTree.filterCount++;
    }
    hallTree.hallState = hallTree.hallUVW[0] + (hallTree.hallUVW[1] << 1) + (hallTree.hallUVW[2] << 2);
    if (!stateContr.INVERSION)
    {
        hallTree.hallState = 7 - hallTree.hallState;
    }
    if (hallTree.hallState != hallTree.oldHallState)
    {
        switch (hallTree.hallState)
        {
        case 0x05:
        {
            MOS_Q32PWM;
        }
        break;
        case 0x01:
        {
            MOS_Q63PWM;
        }
        break;
        case 0x03:
        {
            MOS_Q16PWM;
        }
        break;
        case 0x02:
        {
            MOS_Q41PWM;
        }
        break;
        case 0x06:
        {
            MOS_Q54PWM;
        }
        break;
        case 0x04:
        {
            MOS_Q25PWM;
        }
        break;
        default:
        {
            STOP_MOTOR;
            hallTree.speedRPM = 0;
        }
        break;
        }
    }
    if (hallTree.oldHallState == hallTree.hallState)
    {
        switch (hallTree.hallState)
        {
        case 0x05:
        {
            MOS_Q32PWM;
        }
        break;
        case 0x01:
        {
            MOS_Q63PWM;
        }
        break;
        case 0x03:
        {
            MOS_Q16PWM;
        }
        break;
        case 0x02:
        {
            MOS_Q41PWM;
        }
        break;
        case 0x06:
        {
            MOS_Q54PWM;
        }
        break;
        case 0x04:
        {
            MOS_Q25PWM;
        }
        break;
        default:
        {
            STOP_MOTOR;
            hallTree.speedRPM = 0;
        }
        break;
        }
    }
    hallTree.oldHallState = hallTree.hallState;
}

void offsetCurrentRead(void)
{
    static uint16_t ADC_PhaseU_Curr[64];
    static uint16_t ADC_PhaseV_Curr[64];
    static uint16_t ADC_PhaseW_Curr[64];
    static uint8_t i = 0;

    ADC_PhaseU_Curr[i] = ADC_DualConvertValTab[0];
    ADC_PhaseV_Curr[i] = ADC_DualConvertValTab[1];
    ADC_PhaseW_Curr[i] = ADC_DualConvertValTab[4];

    i++;
    if (i >= 64)
    {
        i = 0;
    }
    if (stateContr.driveCar == 0)
    {
        uint32_t sum_U = 0;
        uint32_t sum_V = 0;
        uint32_t sum_W = 0;
        uint8_t j;
        for (j = 0; j < 64; j++)
        {
            sum_U += ADC_PhaseU_Curr[j];
            sum_V += ADC_PhaseV_Curr[j];
            sum_W += ADC_PhaseW_Curr[j];
        }
        ADCSampPare.offsetPhaseU_Curr = sum_U / 64;
        ADCSampPare.offsetPhaseV_Curr = sum_V / 64;
        ADCSampPare.offsetPhaseW_Curr = sum_W / 64;
    }
}

void HALL_ADCSample(void)
{
    ADCSampPare.phaseUCurr = ADC_DualConvertValTab[0] - ADCSampPare.offsetPhaseU_Curr;
    ADCSampPare.phaseVCurr = ADC_DualConvertValTab[1] - ADCSampPare.offsetPhaseV_Curr;
    ADCSampPare.phaseWCurr = ADC_DualConvertValTab[4] - ADCSampPare.offsetPhaseW_Curr;
    ADCSampPare.busVoltage = ADC_DualConvertValTab[3];
    switch (hallTree.hallState)
    {
    case 0x05:
    {
        ADCSampPare.busCurr = ADCSampPare.phaseVCurr + ADCSampPare.phaseUCurr;
    }
    break;
    case 0x01:
    {
        ADCSampPare.busCurr = ADCSampPare.phaseVCurr + ADCSampPare.phaseWCurr;
    }
    break;
    case 0x03:
    {
        ADCSampPare.busCurr = ADCSampPare.phaseUCurr + ADCSampPare.phaseWCurr;
    }
    break;
    case 0x02:
    {
        ADCSampPare.busCurr = ADCSampPare.phaseUCurr + ADCSampPare.phaseVCurr;
    }
    break;
    case 0x06:
    {
        ADCSampPare.busCurr = ADCSampPare.phaseWCurr + ADCSampPare.phaseVCurr;
    }
    break;
    case 0x04:
    {
        ADCSampPare.busCurr = ADCSampPare.phaseWCurr + ADCSampPare.phaseUCurr;
    }
    break;
    default:
    {
        ADCSampPare.busCurr = ADCSampPare.busCurr;
    }
    break;
    }
}

void motorTestProgram(void)
{
    stateContr.duty = 1399;
    HAL_Delay(300);
    MOS_Q16PWM;
    HAL_Delay(300);
    myPrint("Q16PWM HALL TABLE IS %d %d %d \r\n",HALL_U_STATUS,HALL_V_STATUS,HALL_W_STATUS);
    HAL_Delay(300);
    MOS_Q63PWM;
    HAL_Delay(300);
    myPrint("Q63PWM HALL TABLE IS %d %d %d \r\n",HALL_U_STATUS,HALL_V_STATUS,HALL_W_STATUS);
    HAL_Delay(300);
    MOS_Q32PWM;
    HAL_Delay(300);
    myPrint("Q32PWM HALL TABLE IS %d %d %d \r\n",HALL_U_STATUS,HALL_V_STATUS,HALL_W_STATUS);
    HAL_Delay(300);
    MOS_Q25PWM;
    HAL_Delay(300);
    myPrint("Q25PWM HALL TABLE IS %d %d %d \r\n",HALL_U_STATUS,HALL_V_STATUS,HALL_W_STATUS);
    HAL_Delay(300);
    MOS_Q54PWM;
    HAL_Delay(300);
    myPrint("Q54PWM HALL TABLE IS %d %d %d \r\n",HALL_U_STATUS,HALL_V_STATUS,HALL_W_STATUS);
    HAL_Delay(300);
    MOS_Q41PWM;
    HAL_Delay(300);
    myPrint("Q41PWM HALL TABLE IS %d %d %d \r\n",HALL_U_STATUS,HALL_V_STATUS,HALL_W_STATUS);
}

void BLDCInit(void)
{
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_DualConvertValTab, 5) != HAL_OK)
    {
        while (1)
        {
            myPrint("error code:hall_dma_error!\r\n");
        }
        HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    }
    if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) //start it will cause motor en
    {
        while (1)
        {
            myPrint("error code:hall_base_start_it_error!\r\n ");
        }
    }
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    stateContr.controlMode = LOOP;
    switch (stateContr.controlMode)
    {
    case 0x1:
    {
        stateContr.aimSpeed = 100;
        stateContr.aimDuty = 1000 * stateContr.aimSpeed / 100;
    }
    break;
    case 0x2:
    {
        piSpd.ref = 50;
    }
    break;
    case 0x3:
    {
        piSpd.ref = 50;
        stateContr.driveCar = 0;
    }
    break;
    }
}