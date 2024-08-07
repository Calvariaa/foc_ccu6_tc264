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

float data_send[8];
// **************************** �������� ****************************
int core0_main(void)
{
    clock_init(); // ��ȡʱ��Ƶ��<��ر���>
    debug_init(); // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�û����� ���������ʼ�������

    // ��ʼ��LED����
    led_init();

    // ��ʼ����������
    // key_init();

    // ��ʼ��adcͨ����adc���ڲɼ���Դ��ѹ��ĸ�ߵ������������
    adc_collection_init();

    move_filter_double_init(&id_ref_filter); // �����˲���ʼ��
    move_filter_double_init(&iq_ref_filter); // �����˲���ʼ��

    // ��ʼ������ٶ��뷽����Ϣ������
    //  motor_information_out_init();

    // �����źŲ����ʼ��
    pwm_input_init();

    // �����ʼ��
    // motor_init();

    // PID������ʼ��
    // closed_loop_pi_init();
    AS5407P_Init();
    // ��ʼ����ʱ��,�����������PWM
    ccu6_pwm_init();

    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready(); // �ȴ����к��ĳ�ʼ�����

    while (TRUE)
    {
        // data_send[0]=(uint16) hall_value_now;
        //                data_send[0]=(uint16) clark.Alpha;
        //                data_send[1]=(uint16) clark.Beta;
        //                data_send[2]=(uint16) park.id_ref;
        //                data_send[3]=(uint16) park.iq_ref;
        // data_send[4]=(uint16)adc_information.current_a;
        //   data_send[5]=(uint16)adc_information.current_b;
        //  data_send[6]=(uint16)adc_information.current_c;
        // Data_Send(UART_0,data_send);
        // data_send[4]=(uint16)adc_information.current_a;
        //   data_send[5]=(uint16)adc_information.current_b;
        // data_send[6]=(uint16)adc_information.current_c;
        // buff1[0]=0x3FFF;
        // buff1[0]=0x3FFF;
        led_output(); // ���ݵ�ǰ״̬��������Ϩ��LED��
        // Get_Rotor_Angle();
        //  AS5047P_TEST_PRINTF();
        //////////////////////////////////////////
        // DUZHI();
        // data_send[0]=(uint16) (Get_Rotor_Angle());
        // data_send[1]=date_cibianmaqi[1];
        //  data_send[2]=date_cibianmaqi[2];
        // data_send[3]=date_cibianmaqi[3];
        /// data_send[4]=date_cibianmaqi[4];
        // data_send[5]=date_cibianmaqi[5];
        // data_send[6]=date_cibianmaqi[6];
        // data_send[7]=date_cibianmaqi[7];

        // Data_Send(UART_0, data_send); // ����

        data_send[0] = (float)theta;
        for (int8 i = 0; i <= 10; i++)
            printf("%f,", data_send[i]);
        printf("-1.0\r\n");

        //////////////////////////////////////////////////////////////
        // �������ݵ�����ʾ���� ����ʾ������������ https://pan.baidu.com/s/198CMXTZsbI3HAEqNXDngBw
        //        data_conversion((uint16)speed_filter.data_average, pwm_in_duty, hall_value_now, duty, virtual_scope_data);
        //        uart_write_buffer(UART_0, virtual_scope_data, sizeof(virtual_scope_data));  //����ת����ɺ�ʹ�ô��ڷ��ͽ���������ݷ��ͳ�ȥ
    }
}

#pragma section all restore
// **************************** �������� ****************************
