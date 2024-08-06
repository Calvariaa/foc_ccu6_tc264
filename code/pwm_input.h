/*********************************************************************************************************************
 * TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * Copyright (c) 2022 SEEKFREE ��ɿƼ�
 *
 * ���ļ��� TC264 ��Դ���һ����
 *
 * TC264 ��Դ�� ��������
 * �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
 * �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
 *
 * ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
 * ����û�������������Ի��ʺ��ض���;�ı�֤
 * ����ϸ����μ� GPL
 *
 * ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
 * ���û�У������<https://www.gnu.org/licenses/>
 *
 * ����ע����
 * ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
 * �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
 * ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
 * ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
 *
 * �ļ�����          pwm_input
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          ADS v1.8.0
 * ����ƽ̨          TC264D
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2023-02-01       pudding             first version
 ********************************************************************************************************************/
#ifndef _PWM_INPUT_H
#define _PWM_INPUT_H

#include "IFXGTM_TIM_IN.h"
#include "zf_common_headfile.h"
#include "zf_driver_gpio.h"

// #define MOTOR_DIR_IN_PIN    P11_3
// #define MOTOR_PWM_IN_PIN    P11_2

#define MOTOR_DIR_IN_PIN P02_7
#define MOTOR_PWM_IN_PIN P02_6

#define MOTOR_back_DIR_IN_PIN P21_3
#define MOTOR_back_PWM_IN_PIN P21_4

#define GTM_PWM_IN_PRIORITY 80      // ���벶���ж����ȼ�
#define GTM_PWM_IN_PRIORITY_back 70 // ���벶���ж����ȼ�
extern uint16 i;
extern uint16 pwm_in_duty;
extern IfxGtm_Tim_In driver;
extern IfxGtm_Tim_In_Config config;
extern uint16 pwm_in_duty_back;
extern IfxGtm_Tim_In driver_back;
extern IfxGtm_Tim_In_Config config_back;
void pwm_input_init(void);
void pwm_back_input_init(void);
void TIM_InitConfig(IfxGtm_Tim_TinMap pin);
void TIM_GetPwm(IfxGtm_Tim_TinMap pin, float32 *Duty);
#endif
