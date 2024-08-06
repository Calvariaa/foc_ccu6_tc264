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
 * ��������          ADS v1.8.0
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

// char buff;
// uint16 *buff=0x3FFF;
// uint16 *buff2;
// #define buff1     0x3FFF

// extern uint16 buff666;
// extern uint16 buf666[8];
uint16 buff1;
uint16 buff2[8];
uint16 data_send[8];
uint16 a;
uint16 b;
uint16 c;
void Data_Send(uart_index_enum uartn, unsigned short int *pst) // ��λ��
{
    unsigned char _cnt = 0;
    unsigned char sum = 0;
    unsigned char data_to_send[23]; // ���ͻ���
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x02;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = (unsigned char)(pst[0] >> 8); // ��8λ
    data_to_send[_cnt++] = (unsigned char)pst[0];        // ��8λ
    data_to_send[_cnt++] = (unsigned char)(pst[1] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[1];
    data_to_send[_cnt++] = (unsigned char)(pst[2] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[2];
    data_to_send[_cnt++] = (unsigned char)(pst[3] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[3];
    data_to_send[_cnt++] = (unsigned char)(pst[4] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[4];
    data_to_send[_cnt++] = (unsigned char)(pst[5] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[5];
    data_to_send[_cnt++] = (unsigned char)(pst[6] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[6];
    data_to_send[_cnt++] = (unsigned char)(pst[7] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[7];
    data_to_send[_cnt++] = (unsigned char)(pst[8] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[8];
    data_to_send[3] = _cnt - 4;
    sum = 0;
    for (unsigned char i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    for (unsigned char i = 0; i < _cnt; i++)
        uart_write_byte(uartn, data_to_send[i]);
}
// �ر�˵�����ÿ�Դ��Ŀ�����ڸ�����ͬѧ������Լ���С��ʱ���вο���Ӳ������������ܹ�ֱ�ӿ���ʹ�����Լ�����Ʒ�У����Ҳο����������Ӳ������������������
// �ر�˵�����ÿ�Դ��Ŀ�����ڸ�����ͬѧ������Լ���С��ʱ���вο���Ӳ������������ܹ�ֱ�ӿ���ʹ�����Լ�����Ʒ�У����Ҳο����������Ӳ������������������
// �ر�˵�����ÿ�Դ��Ŀ�����ڸ�����ͬѧ������Լ���С��ʱ���вο���Ӳ������������ܹ�ֱ�ӿ���ʹ�����Լ�����Ʒ�У����Ҳο����������Ӳ������������������

// #define DUQU(x)              ((x) ? (gpio_set_level(P20_13,1)) : (gpio_set_level(P20_13,0)))

// uint16 date_cibianmaqi[8];
// void DUZHI()
//{
//    DUQU(0);
//    spi_read_16bit_registers(0, 0x3FFF|0x80, date_cibianmaqi, 8);
//   DUQU(1);
//}

int core0_main(void)
{
    // ��ȡʱ��Ƶ��  ��ر���
    get_clock();

    // ��ʼ��LED����
    led_init();

    // ��ʼ����������
    key_init();

    // ��ʼ��adcͨ����adc���ڲɼ���Դ��ѹ��ĸ�ߵ������������
    adc_collection_init();

    // ������ʼ��
    // hall_init();

    // ����ƽ���˲���ʼ��
    // move_filter_init(&speed_filter);
    //   move_filter_double_init(&current_a_filter);//�����˲���ʼ��
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

    // ��ʼ����ʱ��,���ڼ���ռ�ձ�
    //  pit_interrupt_us(CCU6_0, PIT_CH0, 20);
    //   pit_interrupt_us(CCU6_0, PIT_CH0, 10);
    enableInterrupts();

    while (1)
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

        Data_Send(UART_0, data_send); // ����

        //////////////////////////////////////////////////////////////
        // �������ݵ�����ʾ���� ����ʾ������������ https://pan.baidu.com/s/198CMXTZsbI3HAEqNXDngBw
        //        data_conversion((uint16)speed_filter.data_average, pwm_in_duty, hall_value_now, duty, virtual_scope_data);
        //        uart_write_buffer(UART_0, virtual_scope_data, sizeof(virtual_scope_data));  //����ת����ɺ�ʹ�ô��ڷ��ͽ���������ݷ��ͳ�ȥ
    }
}

#pragma section all restore
