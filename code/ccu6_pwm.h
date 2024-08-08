#ifndef _ccu6_pwm_H
#define _ccu6_pwm_H

#include "ifxccu6_regdef.h"
#include "zf_common_headfile.h"

/*

    AH IfxCcu61_CC60_P20_8_OUT;
    BH IfxCcu61_CC61_P20_9_OUT;
    CH IfxCcu61_CC62_P20_10_OUT;
    AL IfxCcu61_COUT60_P33_12_OUT;
    BL IfxCcu61_COUT61_P33_10_OUT;
    CL IfxCcu61_COUT62_P33_8_OUT;
*/

#define FCY ((uint32)100000000)                 // 系统时钟
#define FPWM ((uint16)20000)                    // PWM频率
#define PWM_PRIOD_LOAD (uint16)(FCY / FPWM / 2) // PWM周期装载值
#define DEADTIME_LOAD (50)                      // 死区装载值

extern Ifx_CCU6 *ccu6SFR;

void ccu6_pwm_init(void);
uint8 ccu61_get_trap_flag(void);

#endif
