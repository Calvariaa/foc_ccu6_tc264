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
 * 开发环境          ADS v1.9.4
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

float data_send[32];

bool protect_flag = true;
// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init(); // 获取时钟频率<务必保留>
    debug_init(); // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等

    // 初始化LED引脚
    led_init();
    board_check_init();

    // 初始化按键引脚
    key_init(5);

    // 初始化adc通道，adc用于采集电源电压、母线电流、相电流的
    adc_collection_init();

    move_filter_double_init(&id_ref_filter); // 滑动滤波初始化
    move_filter_double_init(&iq_ref_filter); // 滑动滤波初始化
    move_filter_double_init(&speed_filter); // 滑动滤波初始化

    // 初始化输出速度与方向信息的引脚
    motor_information_out_init();

    // 输入信号捕获初始化
    pwm_input_init();

    // PID参数初始化
    // closed_loop_pi_init();

    as5047p_init();

    // 初始化定时器,用于输出互补PWM
    ccu6_pwm_init();

    // 初始化定时器,用于计算占空比
    pit_ms_init(CCU60_CH0, 1);

    ccu6_pwm_buzzer();

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready(); // 等待所有核心初始化完毕

    set_zero_angle(get_magnet_angle(get_magnet_val()));

    protect_flag = false;
    while (TRUE)
    {
        led_output(); // 根据当前状态点亮或者熄灭LED灯

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
// **************************** 代码区域 ****************************
