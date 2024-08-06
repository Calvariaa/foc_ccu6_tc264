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
 * 文件名称          cpu0_main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.8.0
 * 适用平台          TC264D
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-09-15       pudding            first version
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

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
void Data_Send(uart_index_enum uartn, unsigned short int *pst) // 上位机
{
    unsigned char _cnt = 0;
    unsigned char sum = 0;
    unsigned char data_to_send[23]; // 发送缓存
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x02;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = (unsigned char)(pst[0] >> 8); // 高8位
    data_to_send[_cnt++] = (unsigned char)pst[0];        // 低8位
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
// 特别说明：该开源项目仅用于各参赛同学在设计自己的小车时进行参考，硬件和软件都不能够直接拷贝使用在自己的作品中，请大家参考后重新完成硬件和软件的设计制作。
// 特别说明：该开源项目仅用于各参赛同学在设计自己的小车时进行参考，硬件和软件都不能够直接拷贝使用在自己的作品中，请大家参考后重新完成硬件和软件的设计制作。
// 特别说明：该开源项目仅用于各参赛同学在设计自己的小车时进行参考，硬件和软件都不能够直接拷贝使用在自己的作品中，请大家参考后重新完成硬件和软件的设计制作。

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
    // 获取时钟频率  务必保留
    get_clock();

    // 初始化LED引脚
    led_init();

    // 初始化按键引脚
    key_init();

    // 初始化adc通道，adc用于采集电源电压、母线电流、相电流的
    adc_collection_init();

    // 霍尔初始化
    // hall_init();

    // 滑动平均滤波初始化
    // move_filter_init(&speed_filter);
    //   move_filter_double_init(&current_a_filter);//滑动滤波初始化
    move_filter_double_init(&id_ref_filter); // 滑动滤波初始化
    move_filter_double_init(&iq_ref_filter); // 滑动滤波初始化
    // 初始化输出速度与方向信息的引脚
    //  motor_information_out_init();

    // 输入信号捕获初始化
    pwm_input_init();

    // 电机初始化
    // motor_init();

    // PID参数初始化
    // closed_loop_pi_init();
    AS5407P_Init();
    // 初始化定时器,用于输出互补PWM
    ccu6_pwm_init();

    // 初始化定时器,用于计算占空比
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
        led_output(); // 根据当前状态点亮或者熄灭LED灯
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

        Data_Send(UART_0, data_send); // 串口

        //////////////////////////////////////////////////////////////
        // 发送数据到虚拟示波器 虚拟示波器下载链接 https://pan.baidu.com/s/198CMXTZsbI3HAEqNXDngBw
        //        data_conversion((uint16)speed_filter.data_average, pwm_in_duty, hall_value_now, duty, virtual_scope_data);
        //        uart_write_buffer(UART_0, virtual_scope_data, sizeof(virtual_scope_data));  //数据转换完成后，使用串口发送将数组的内容发送出去
    }
}

#pragma section all restore
