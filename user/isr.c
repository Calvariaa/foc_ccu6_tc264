/*********************************************************************************************************************
 * TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 TC264 开源库的一部分
 *
 * TC264 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          isr
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.9.20
 * 适用平台          TC264D
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-09-15       pudding            first version
 ********************************************************************************************************************/

#include "isr_config.h"
#include "isr.h"

#define START_DELAY_FLAG (timer_1ms < 1000 || !init_finish_flag)

 // PWM中断处理函数
IFX_INTERRUPT(ccu6_t12_pwm, 0, CCU60_T12_ISR_PRIORITY)
{
    IfxCpu_enableInterrupts();
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t12PeriodMatch);


    if (START_DELAY_FLAG)
        return;


    if (pwm_in_duty < 324)
    {
        // if (protect_flag == 0)
        set_zero_angle(get_magnet_angle(get_magnet_val()));

        reset_rotations();
        // full_rotations = 0;
        full_rotations_last = 0;

        set_angle = theta_magnet;
        expect_rotations = full_rotations;

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
        // if (protect_flag == 0)
        set_zero_angle(get_magnet_angle(get_magnet_val()));

        reset_rotations();
        // full_rotations = 0;
        full_rotations_last = 0;

        set_angle = theta_magnet;
        expect_rotations = full_rotations;

        protect_flag = 1;
        pwm_in_speed = 0;

        motor_control.set_speed = 0;
    }

    foc_commutation();
}

uint64 timer_1ms = 0u;
uint16 ierror_count = 0u;
// **************************** PIT中断函数 ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0); // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);

    if (init_finish_flag)
        timer_1ms++;

    if (START_DELAY_FLAG)
        return;

    if (timer_1ms % 5 == 0)
        motor_speed_out();

    if (fabsf(Park_in.u_q) >= FOC_UQ_MAX) {
        if (ierror_count < 32760)
            ierror_count++;
    }
    else ierror_count = 0u;

    // if (abs(_read_dangle()) < I_Error_Speed)
    // {
    //     if (ierror_count <= I_Error_Dat)
    //         ierror_count++;
    // }

}

// 输入捕获
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
    interrupt_global_enable(0); // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);
}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0); // 开启中断嵌套
    pit_clear_flag(CCU61_CH0);
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0); // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);
}
// **************************** PIT中断函数 ****************************

// **************************** 外部中断函数 ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);            // 开启中断嵌套
    if (exti_flag_get(ERU_CH0_REQ0_P15_4)) // 通道0中断
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
    }

    if (exti_flag_get(ERU_CH4_REQ13_P15_5)) // 通道4中断
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套

    if (exti_flag_get(ERU_CH1_REQ10_P14_3)) // 通道1中断
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler(); // ToF 模块 INT 更新中断
    }

    if (exti_flag_get(ERU_CH5_REQ1_P15_8)) // 通道5中断
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);
    }
}

// 由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
// IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // 开启中断嵌套
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // 通道2中断
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // 通道6中断
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }

IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);            // 开启中断嵌套
    if (exti_flag_get(ERU_CH3_REQ6_P02_0)) // 通道3中断
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler(); // 摄像头触发采集统一回调函数
    }
    if (exti_flag_get(ERU_CH7_REQ16_P15_1)) // 通道7中断
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
    }
}
// **************************** 外部中断函数 ****************************

// **************************** DMA中断函数 ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套
    camera_dma_handler();       // 摄像头采集完成统一回调函数
}
// **************************** DMA中断函数 ****************************

// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套
}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套

#if DEBUG_UART_USE_INTERRUPT   // 如果开启 debug 串口中断
    debug_interrupr_handler(); // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                         // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
}

// 串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套
    camera_uart_handler();      // 摄像头参数配置统一回调函数
}

// 串口2默认连接到无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套
}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);     // 开启中断嵌套
    wireless_module_uart_handler(); // 无线模块统一回调函数
}
// 串口3默认连接到GPS定位模块
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套
}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套
    gnss_uart_callback();       // GNSS串口回调函数
}

// 串口通讯错误中断
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
