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
 * �ļ�����          gpio
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

#include "zf_driver_pwm.h"
#include "zf_driver_delay.h"

#include "gpio.h"

uint8 key_status = 1;  // ��ǰ����״̬
uint8 key_last_status; // ��һ�ΰ���״̬
uint8 key_flag;        // ����������־λ
uint8 board_check_id = 0;

void board_check_init()
{
    gpio_init(EN_PIN, GPI, 1, GPI_PULL_UP);

    if (!gpio_get_level(EN_PIN))
    {
        board_check_id = 0;  // en board
        zero_reval = 4.41;
    }
    else
    {
        board_check_id = 1;  // no en board
        zero_reval = 3.41;
    }
}


void led_init()
{
    gpio_init(LED_RUN_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(LED_ERR_PIN, GPO, 1, GPO_PUSH_PULL);
    // gpio_init(LED_MODEL_PIN, GPO, 1, GPO_PUSH_PULL); 
}

void led_output()
{
    if (ccu61_get_trap_flag())
    {
        gpio_set_level(LED_ERR_PIN, 0); // �������ϵ�
        gpio_set_level(LED_RUN_PIN, 1); // �ر����е�
    }
    else
    {
        gpio_set_level(LED_ERR_PIN, 1); // �رչ��ϵ�
        if (timer_1ms >= 50)
            gpio_set_level(LED_RUN_PIN, 0); // �������е�
        else
            gpio_set_level(LED_RUN_PIN, 1); // �ر����е�
    }
    // if (model_state)
    // if (0)
    // {
    //     gpio_set_level(LED_MODEL_PIN, 1);
    // }
    // else
    // {
    //     gpio_set_level(LED_MODEL_PIN, 0);
    // }
}
