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
 * �ļ�����          adc
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

#ifndef _ADC_H
#define _ADC_H

#include "zf_common_typedef.h"

// �˷ŵķŴ���Ϊ75��   ��������Ϊ2m��  �˷��вο���ѹ1.65V ��˼���ʵ�ʵ�����ʽ���£�
// I = ((ADC / 4096) * 3.3 - 1.65) / 75 / 0.002
// ����ADCΪʵ�ʲ�ֵ��IΪ������ʵ�ʵ��� �򻯹�ʽ�ã�I = (ADC - 2048) * 0.00537

// 2mOum
// #define CURRENT_TRANSITION_VALUE (0.00537) // �������ת��ϵ��

// 10mOum
#define CURRENT_TRANSITION_VALUE (0.0013425) // �������ת��ϵ��

// ɲ������ͨ���Ƚ���ʵ��  ����ԭ��ͼ��֪�Ƚϻ�׼Ϊ2.55V ��˼���ɲ��������ʽ���£�
// I = (2.55 - 1.65) / 75 / 0.002 = 6A
// �����޸�ɲ��������ֵ����ͨ������Ӳ��ʵ��

#define ADC_GATHER_RESOLUTION (ADC_12BIT)      // ADC�ɼ��ֱ���
#define BOARD_POTENTIOMET_PORT (ADC2_CH11_A45) // ���ص�λ����ȡͨ��
#define BATTERY_PHASE_PORT (ADC2_CH12_A46)     // ��Դ��ѹ��ȡͨ��
#define A_PHASE_PORT (ADC2_CH3_A35)            // A�������ȡͨ��
#define B_PHASE_PORT (ADC2_CH4_A36)            // B�������ȡͨ��
#define C_PHASE_PORT (ADC2_CH5_A37)            // C�������ȡͨ��
#define BUS_PHASE_PORT (ADC2_CH6_A38)          // ĸ�ߵ�����ȡͨ��

typedef struct
{
    float battery_voltage; // ���ص�λ����ѹ
    float voltage_bus;     // ĸ�ߵ���
    float current_a;       // A�����
    float current_b;       // B�����
    float current_c;       // C�����
    uint16 current_board;  // ���ص�λ����ѹ
} adc_struct;

extern adc_struct adc_information;

void adc_collection_init(void);
void adc_read(void);
void battery_check(void);

#endif
