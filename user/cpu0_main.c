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
 * �ļ�����          cpu0_main
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          ADS v1.9.4
 * ����ƽ̨          TC264D
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2022-09-15       pudding            first version
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
 // ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

float data_send[32];

bool protect_flag = true;
// **************************** �������� ****************************
int core0_main(void)
{
    clock_init(); // ��ȡʱ��Ƶ��<��ر���>
    debug_init(); // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�û����� ���������ʼ�������

    // ��ʼ��LED����
    led_init();
    board_check_init();

    // ��ʼ����������
    key_init(5);

    // ��ʼ��adcͨ����adc���ڲɼ���Դ��ѹ��ĸ�ߵ������������
    adc_collection_init();

    move_filter_double_init(&id_ref_filter); // �����˲���ʼ��
    move_filter_double_init(&iq_ref_filter); // �����˲���ʼ��
    move_filter_double_init(&speed_filter); // �����˲���ʼ��

    // ��ʼ������ٶ��뷽����Ϣ������
    motor_information_out_init();

    // �����źŲ����ʼ��
    pwm_input_init();

    // PID������ʼ��
    // closed_loop_pi_init();

    as5047p_init();

    // ��ʼ����ʱ��,�����������PWM
    ccu6_pwm_init();

    // ��ʼ����ʱ��,���ڼ���ռ�ձ�
    pit_ms_init(CCU60_CH0, 1);

    ccu6_pwm_buzzer();

    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready(); // �ȴ����к��ĳ�ʼ�����

    set_zero_angle(get_magnet_angle(get_magnet_val()));

    protect_flag = false;
    while (TRUE)
    {
        led_output(); // ���ݵ�ǰ״̬��������Ϩ��LED��

        data_send[0] = (float)zero_reval;
        data_send[4] = (float)pwm_in_duty;
        data_send[5] = (float)motor_control.current_speed;
        data_send[16] = (float)zero_angle;

        // pwm_set_freq(MOTOR_SPEED_OUT_PIN, 50, 5000);
        if (timer_1ms >= 50)
        {
            for (int8 i = 0; i <= 16; i++)
                printf("%f,", data_send[i]);
            printf("-1.0\r\n");
        }
    }
}

#pragma section all restore
// **************************** �������� ****************************
