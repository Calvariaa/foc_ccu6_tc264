#ifndef _ccu6_pwm_H
#define _ccu6_pwm_H

#include "ifxccu6_regdef.h"
#include "zf_common_headfile.h"

#define FCY ((uint32)100000000)                 // ϵͳʱ��
#define FPWM ((uint16)20000)                    // PWMƵ��
#define PWM_PRIOD_LOAD (uint16)(FCY / FPWM / 2) // PWM����װ��ֵ
#define DEADTIME_LOAD (50)                      // ����װ��ֵ

extern Ifx_CCU6 *ccu6SFR;

void ccu6_pwm_init(void);
uint8 ccu61_get_trap_flag(void);

#endif
