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
 * �ļ�����          motor
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

#include "IFXGTM_TIM_IN.h"
#include "ifxGtm_Tim.h"
#include "motor.h"

int32 encoder = 0; // ������ֵ
motor_struct motor_control;
////-------------------------------------------------------------------------------------------------------------------
//// �������     ���ת����Ϣ������ų�ʼ��
//// ����˵��     void
//// ʹ��ʾ��     motor_information_out_init();
//// ��ע��Ϣ
////-------------------------------------------------------------------------------------------------------------------
void motor_information_out_init(void)
{
    pwm_init(MOTOR_SPEED_OUT_PIN, 50, 0);                // ��ʼ������ٶ�������ţ����ΪƵ�ʱ仯���źţ�������ת��Ϊÿ����5000ת���������ϵ�Ƶ��Ϊ5000Hz��
    gpio_init(MOTOR_DIR_OUT_PIN, GPO, 0, GPO_PUSH_PULL); // ��ʼ���������������ţ��û��˽��գ�
}
////-------------------------------------------------------------------------------------------------------------------
//// �������     ����ٶ����
//// ����˵��     void
//// ʹ��ʾ��     motor_speed_out();
//// ��ע��Ϣ
////-------------------------------------------------------------------------------------------------------------------
void motor_speed_out(void)
{
    encoder = (int32)encoder_get_count(AS5047P_TIM); // �ɼ���Ӧ����������
    encoder_clear_count(AS5047P_TIM);         // �����Ӧ����

    // motor_control.current_speed = encoder;

    encoder = (int32)(-speed_filter.data_average);
    // encoder = (int32)50;

    // motor_control.current_speed = encoder;
    if (encoder)
    {
        pwm_set_freq(MOTOR_SPEED_OUT_PIN, func_abs(encoder) * 1024, 5000);
        if (encoder < 0)
            gpio_set_level(MOTOR_DIR_OUT_PIN, 1);
        if (encoder > 0)
            gpio_set_level(MOTOR_DIR_OUT_PIN, 0);
    }
    else
    {
        pwm_set_freq(MOTOR_SPEED_OUT_PIN, 1000, 0);
        gpio_set_level(MOTOR_DIR_OUT_PIN, 0);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ������ת����
// ����˵��     void
// ʹ��ʾ��     motor_set_dir();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void motor_set_dir()
{
    if (gpio_get_level(MOTOR_DIR_IN_PIN) == 1)
    {
        motor_control.dir = FORWARD;
    }
    else
    {
        motor_control.dir = REVERSE;
    }
}

void motor_set_speed()
{
    // if (gpio_get_level(MOTOR_DIR_IN_PIN) == 1)
    // {
    //     motor_control.dir = FORWARD;
    // }
    // else
    // {
    //     motor_control.dir = REVERSE;
    // }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������������     �ر���������
//  @param      periodAH:       A����PWMռ�ձ�
//  @param      periodBH:       B����PWMռ�ձ�
//  @param      periodCH:       C����PWMռ�ձ�
//  @return     void
//  @since      �������Ķ���ģʽ��װ��ֵԽ�󣬸ߵ�ƽʱ��Խ��
//-------------------------------------------------------------------------------------------------------------------
void mos_all_phrase_open(uint16 periodAH, uint16 periodBH, uint16 periodCH)
{
    ccu6SFR->MODCTR.B.T12MODEN = 0x3F; // 0011 1111                   //�û��ֲ�27.8�½��Լ���

    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_0, periodAH); // ���ñȽ�ֵ
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_1, periodBH); // ���ñȽ�ֵ
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_2, periodCH); // ���ñȽ�ֵ
    IfxCcu6_enableShadowTransfer(ccu6SFR, TRUE, FALSE);                  // ʹ��
}
void mos_close()
{
    ccu6SFR->MODCTR.B.T12MODEN = 0x2A;

    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_0, 0);
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_1, 0);
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_2, 0);
    IfxCcu6_enableShadowTransfer(ccu6SFR, TRUE, FALSE);
}
