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
 * �ļ�����          pwm_output
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

#ifndef _PWM_OUTPUT_H_
#define _PWM_OUTPUT_H_

#include "zf_common_headfile.h"

extern int16 encoder;
#define MOTOR_SPEED_OUT_PIN (ATOM1_CH6_P00_7) // �����ת�ٶ��������
#define MOTOR_DIR_OUT_PIN (P00_5)             // ������з����������

typedef enum
{
    FORWARD, // ��ת
    REVERSE, // ��ת
} MOTOR_DIR_enum;

typedef enum
{
    MOTOR_DISABLE, // �����ر�
    MOTOR_ENABLE,  // ����ʹ��
} MOTOR_EN_STATUS_enum;

typedef struct
{
    MOTOR_EN_STATUS_enum en_status; // ָʾ���ʹ��״̬
    uint8 brake_flag;               // ָʾ��ǰɲ���Ƿ���Ч    1������ɲ��  0����������
    MOTOR_DIR_enum dir;             // �����ת���� FORWARD����ת  REVERSE����ת     BRAKE��ɲ��
    int32 set_speed;                // ���õ��ٶ�
    int32 current_speed;                // ���õ��ٶ�
    int32 max_speed;                // �ٶ����ֵ
    int32 min_speed;                // �ٶ���Сֵ
} motor_struct;

extern motor_struct motor_control;
extern int16 duty; // PWMռ�ձȳ�ֵ

// void motor_init(void);
void motor_information_out_init(void);
void motor_set_dir(void);
void motor_dir_out(void);
void motor_speed_out(void);

void mos_all_phrase_open(uint16, uint16, uint16);
void mos_close();
#endif /* CODE_PWM_OUTPUT_H_ */
