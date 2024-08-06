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

#include "zf_driver_adc.h"

#include "adc.h"

adc_struct adc_information;

void adc_collection_init(void)
{
    adc_init(BOARD_POTENTIOMET_PORT, ADC_GATHER_RESOLUTION); // ��ʼ�����ص�λ���ɼ��˿�
    adc_init(BATTERY_PHASE_PORT, ADC_GATHER_RESOLUTION);     // ��ʼ����Դ��ѹ�ɼ��˿�
    adc_init(A_PHASE_PORT, ADC_GATHER_RESOLUTION);           // ��ʼ��A������ɼ��˿�
    adc_init(B_PHASE_PORT, ADC_GATHER_RESOLUTION);           // ��ʼ��B������ɼ��˿�
    adc_init(C_PHASE_PORT, ADC_GATHER_RESOLUTION);           // ��ʼ��C������ɼ��˿�
    adc_init(BUS_PHASE_PORT, ADC_GATHER_RESOLUTION);         // ��ʼ��ĸ�ߵ����ɼ��˿�
}

void adc_read(void)
{
    uint16 adc_value_a_phase = 0;
    uint16 adc_value_b_phase = 0;
    uint16 adc_value_c_phase = 0;
    uint16 adc_value_bus_phase = 0;
    uint16 adc_value_battery_phase = 0;

    adc_value_a_phase = adc_convert(A_PHASE_PORT);                                                // ��ȡA������ɼ��˿ڵ�ֵ
    adc_value_b_phase = adc_convert(B_PHASE_PORT);                                                // ��ȡA������ɼ��˿ڵ�ֵ
    adc_value_c_phase = adc_convert(C_PHASE_PORT);                                                // ��ȡA������ɼ��˿ڵ�ֵ
    adc_value_bus_phase = adc_convert(BUS_PHASE_PORT);                                            // ��ȡĸ�ߵ����ɼ��˿ڵ�ֵ
    adc_value_battery_phase = adc_convert(BATTERY_PHASE_PORT);                                    // ��ȡ��Դ��ѹ�ɼ��˿ڵ�ֵ
    adc_information.current_board = adc_convert(BOARD_POTENTIOMET_PORT);                          // ��ȡ���ص�λ����ѹ
    adc_information.current_a = (float)(adc_value_a_phase - 2048) * CURRENT_TRANSITION_VALUE;     // ����A�����
    adc_information.current_b = (float)(adc_value_b_phase - 2048) * CURRENT_TRANSITION_VALUE;     // ����B�����
    adc_information.current_c = (float)(adc_value_c_phase - 2048) * CURRENT_TRANSITION_VALUE;     // ����C�����
    adc_information.voltage_bus = (float)(adc_value_bus_phase - 2048) * CURRENT_TRANSITION_VALUE; // ����ĸ�ߵ���
    adc_information.battery_voltage = (float)adc_value_battery_phase / 4096 * 3.3 * 11;           // �����ص�ѹ
}

#define BATTERY_PROTECT_VALUE (3.7f)
void battery_check(void)
{
    static uint32 check_count = 0;

    if (adc_information.battery_voltage > 5.0)
    {
        if (adc_information.battery_voltage < 9.0)
        {
            if (adc_information.battery_voltage < (BATTERY_PROTECT_VALUE * 2)) // 2s��ص�ѹ���õ���7.4V
            {
                check_count++;
            }
            else
            {
                // ����ѹ���ͣ�����1����ٻָ����
                check_count = (check_count > 100 && check_count < 300) ? (check_count + 1) : 0;
            }
        }
        else if (adc_information.battery_voltage < 13.0)
        {
            if (adc_information.battery_voltage < (BATTERY_PROTECT_VALUE * 3)) // 3s��ص�ѹ���õ���11.1V
            {
                check_count++;
            }
            else
            {
                check_count = (check_count > 100 && check_count < 300) ? (check_count + 1) : 0;
            }
        }
        else if (adc_information.battery_voltage < 26.0)
        {
            if (adc_information.battery_voltage < (BATTERY_PROTECT_VALUE * 6)) // 6s��ص�ѹ���õ���22.2V
            {
                check_count++;
            }
            else
            {
                check_count = (check_count > 100 && check_count < 300) ? (check_count + 1) : 0;
            }
        }
    }
    //
    // else
    // {
    //     motor_control.battery_state = NO_BATTERY;
    // }

    // if(check_count == 0)
    // {
    //     motor_control.battery_state = NORMAL_VOLTAGE;
    // }
    // if(check_count > 100)                                       // ��ѹ���ͳ���500ms��ȷ�Ϲ���
    // {
    //     motor_control.battery_state = LOW_VOLTAGE;

    // }
}
