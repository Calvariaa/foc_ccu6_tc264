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
 * �ļ�����          isr
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          ADS v1.9.20
 * ����ƽ̨          TC264D
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2022-09-15       pudding            first version
 ********************************************************************************************************************/

#include "isr_config.h"
#include "isr.h"

 // PWM�жϴ�����
IFX_INTERRUPT(ccu6_t12_pwm, 0, CCU60_T12_ISR_PRIORITY)
{
    IfxCpu_enableInterrupts();
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t12PeriodMatch);

    if (timer_1ms > 50)  // 50ms
        foc_commutation();
}

uint64 timer_1ms = 0u;
// **************************** PIT�жϺ��� ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    pit_clear_flag(CCU60_CH0);

    if (init_finish_flag)
        timer_1ms++;

    if (timer_1ms < 50)
        return;

    motor_speed_out();

    // if (abs(_read_dangle()) < I_Error_Speed)
    // {
    //     if (I_Error_Cnt <= I_Error_Dat)
    //         I_Error_Cnt++;
    // }

    if (pwm_in_duty < 324)
    {
        if (protect_flag == 0)
            set_zero_angle(get_magnet_angle(get_magnet_val()));

        full_rotations = 0;
        full_rotations_last = 0;

        protect_flag = 1;
        pwm_in_speed = 0;

        motor_control.set_speed = 0;
    }
    else if (pwm_in_duty < 1325)
    {
        protect_flag = 0;
        motor_set_dir();
        pwm_in_speed = (pwm_in_duty - 324) / 2;

        motor_control.set_speed = (float)(MINMAX((pwm_in_speed / 10000.f), -0.2f, 0.2f)) * (motor_control.dir ? 1 : -1);
    }
    else
    {
        if (protect_flag == 0)
            set_zero_angle(get_magnet_angle(get_magnet_val()));

        full_rotations = 0;
        full_rotations_last = 0;

        protect_flag = 1;
        pwm_in_speed = 0;

        motor_control.set_speed = 0;
    }
}

// ���벶��
int8 new_data_filter = 0;
IFX_INTERRUPT(gtm_pwm_in, 0, GTM_PWM_IN_PRIORITY)
{
    IfxGtm_Tim_In_update(&driver);

    if (FALSE == driver.newData)
    {
        if (gpio_get_level(MOTOR_PWM_IN_PIN))
        {
            if (new_data_filter > 0)
            {
                new_data_filter--;
            }
            else
            {
                driver.periodTick = 20000;
                driver.pulseLengthTick = driver.periodTick;
            }
        }
        else
        {
            new_data_filter = 3;
            driver.periodTick = 20000;
            driver.pulseLengthTick = 0;
        }
    }
    else
    {
        new_data_filter = 0;
    }
    pwm_in_duty = (uint16)func_limit_ab((driver.pulseLengthTick * PWM_PRIOD_LOAD / driver.periodTick), 0, PWM_PRIOD_LOAD);
    // 5000 -> 1325
    // 1000 -> 325

}

IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    pit_clear_flag(CCU60_CH1);
}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    pit_clear_flag(CCU61_CH0);
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    pit_clear_flag(CCU61_CH1);
}
// **************************** PIT�жϺ��� ****************************

// **************************** �ⲿ�жϺ��� ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);            // �����ж�Ƕ��
    if (exti_flag_get(ERU_CH0_REQ0_P15_4)) // ͨ��0�ж�
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
    }

    if (exti_flag_get(ERU_CH4_REQ13_P15_5)) // ͨ��4�ж�
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��

    if (exti_flag_get(ERU_CH1_REQ10_P14_3)) // ͨ��1�ж�
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler(); // ToF ģ�� INT �����ж�
    }

    if (exti_flag_get(ERU_CH5_REQ1_P15_8)) // ͨ��5�ж�
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);
    }
}

// ��������ͷpclk����Ĭ��ռ���� 2ͨ�������ڴ���DMA��������ﲻ�ٶ����жϺ���
// IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // �����ж�Ƕ��
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // ͨ��2�ж�
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // ͨ��6�ж�
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }

IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);            // �����ж�Ƕ��
    if (exti_flag_get(ERU_CH3_REQ6_P02_0)) // ͨ��3�ж�
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler(); // ����ͷ�����ɼ�ͳһ�ص�����
    }
    if (exti_flag_get(ERU_CH7_REQ16_P15_1)) // ͨ��7�ж�
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
    }
}
// **************************** �ⲿ�жϺ��� ****************************

// **************************** DMA�жϺ��� ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    camera_dma_handler();       // ����ͷ�ɼ����ͳһ�ص�����
}
// **************************** DMA�жϺ��� ****************************

// **************************** �����жϺ��� ****************************
// ����0Ĭ����Ϊ���Դ���
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��

#if DEBUG_UART_USE_INTERRUPT   // ������� debug �����ж�
    debug_interrupr_handler(); // ���� debug ���ڽ��մ����� ���ݻᱻ debug ���λ�������ȡ
#endif                         // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
}

// ����1Ĭ�����ӵ�����ͷ���ô���
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    camera_uart_handler();      // ����ͷ��������ͳһ�ص�����
}

// ����2Ĭ�����ӵ�����ת����ģ��
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);     // �����ж�Ƕ��
    wireless_module_uart_handler(); // ����ģ��ͳһ�ص�����
}
// ����3Ĭ�����ӵ�GPS��λģ��
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    gnss_uart_callback();       // GNSS���ڻص�����
}

// ����ͨѶ�����ж�
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart3_handle);
}
