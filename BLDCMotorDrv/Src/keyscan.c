#include "keyscan.h"

extern HALL hallTree;
extern STATE stateContr;
extern PIDREG_T piSpd;
extern PIDREG_T piICurr;

void run(void)
{
    START_MOTOR;
    stateContr.driveCar = 1;
    if (stateContr.controlMode == 1)
    {
        stateContr.aimSpeed = 20;
        stateContr.aimDuty = 7200 / PWM_FREQ * stateContr.aimSpeed / 100;
    }
    if (stateContr.controlMode == 2 || stateContr.controlMode == 3)
    {
        piSpd.ref = 500;
    }
}

void stop(void)
{
    stateContr.duty = 0;
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
    STOP_MOTOR;
    stateContr.aimSpeed = 0;
    stateContr.aimDuty = 0;
    stateContr.driveCar = 0;
    stateContr.startOrder = 1;
    stateContr.currentCount = 0;
    stateContr.speedCount = 0;

    hallTree.filterCount = 0;
    hallTree.filterCountF = 0;
    hallTree.speedRPM = 0;
    hallTree.speedRPMF = 0;

    piSpd.err = 0;
    piSpd.fdb = 0;
    piSpd.out = 0;
    piSpd.outF = 0;
    piSpd.outPreSat = 0;
    piSpd.ref = 0;
    piSpd.satErr = 0;
    piSpd.ud = 0;
    piSpd.ui = 0;
    piSpd.ui_1 = 0;
    piSpd.up = 0;
    piSpd.up1 = 0;
}

void up(void)
{
    if (stateContr.controlMode == 2 || stateContr.controlMode == 3)
    {
        piSpd.ref += 10;
    }
    if (stateContr.controlMode == 1)
    {
        stateContr.aimSpeed += 1;
        stateContr.aimDuty = 4500 * stateContr.aimSpeed / 100;
    }
    if (stateContr.aimSpeed > 80)
    {
        stateContr.aimSpeed = 80;
    }
    if (piSpd.ref > 2500)
    {
        piSpd.ref = 2500;
    }
}

void down(void)
{
    if (stateContr.controlMode == 2 || stateContr.controlMode == 3)
    {
        piSpd.ref -= 10;
    }
    if (stateContr.controlMode == 1)
    {
        stateContr.aimSpeed -= 1;
        stateContr.aimDuty = 4500 * stateContr.aimSpeed / 100;
    }
    if (stateContr.aimSpeed < 10)
    {
        stateContr.aimSpeed = 10;
    }
    if (piSpd.ref < 300)
    {
        piSpd.ref = 300;
    }
}

void dir(void)
{
    if (stateContr.INVERSION == 1)
    {
        stateContr.INVERSION = 0;
        stateContr.temp = 0;
    }
    else
    {
        stateContr.INVERSION = 1;
        stateContr.temp = 1;
    }
}

void keyScan(void)
{
    static uint16_t count = 0;
    if (RUN_STATUS == 0 && (count == 0))
    {
        myPrint("run**************************!\r\n");
        run();
        count = 20;
    }
    if (STOP_STATUS == 0 && (count == 0))
    {
        myPrint("stop!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
        stop();
        count = 20;
    }
    if (UP_STATUS == 0 && (count == 0))
    {
        myPrint("up!+++++++++++++++++++++++++++\r\n");
        up();
        count = 20;
    }
    if (DOWN_STATUS == 0 && (count == 0))
    {
        myPrint("down!---------------------------\r\n");
        down();
        count = 20;
    }
    if (DIR_STATUS == 0 && (count == 0))
    {
        myPrint("dir!|||||||||||||||||||||||||||||||\r\n");
        stop();
        dir();
        count = 20;
    }
    if (count > 0)
    {
        count--;
    }
}