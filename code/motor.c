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
 * 文件名称          motor
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.8.0
 * 适用平台          TC264D
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2023-02-01       pudding             first version
 ********************************************************************************************************************/

#include "IFXGTM_TIM_IN.h"
#include "ifxGtm_Tim.h"
#include "motor.h"

int32 encoder = 0; // 编码器值
motor_struct motor_control;
////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     电机转速信息输出引脚初始化
//// 参数说明     void
//// 使用示例     motor_information_out_init();
//// 备注信息
////-------------------------------------------------------------------------------------------------------------------
void motor_information_out_init(void)
{
    pwm_init(MOTOR_SPEED_OUT_PIN, 50, 0);                // 初始化电机速度输出引脚，输出为频率变化的信号，例如电机转速为每分钟5000转，则引脚上的频率为5000Hz。
    gpio_init(MOTOR_DIR_OUT_PIN, GPO, 0, GPO_PUSH_PULL); // 初始化电机方向输出引脚（用户端接收）
}
////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     电机速度输出
//// 参数说明     void
//// 使用示例     motor_speed_out();
//// 备注信息
////-------------------------------------------------------------------------------------------------------------------
void motor_speed_out(void)
{
    encoder = (int32)encoder_get_count(AS5047P_TIM); // 采集对应编码器数据
    encoder_clear_count(AS5047P_TIM);         // 清除对应计数

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
// 函数简介     设置旋转方向
// 参数说明     void
// 使用示例     motor_set_dir();
// 备注信息
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
//  @brief      开启所有上桥     关闭所有下桥
//  @param      periodAH:       A上桥PWM占空比
//  @param      periodBH:       B上桥PWM占空比
//  @param      periodCH:       C上桥PWM占空比
//  @return     void
//  @since      采用中心对齐模式，装载值越大，高电平时间越短
//-------------------------------------------------------------------------------------------------------------------
void mos_all_phrase_open(uint16 periodAH, uint16 periodBH, uint16 periodCH)
{
    ccu6SFR->MODCTR.B.T12MODEN = 0x3F; // 0011 1111                   //用户手册27.8章节自己看

    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_0, periodAH); // 设置比较值
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_1, periodBH); // 设置比较值
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_2, periodCH); // 设置比较值
    IfxCcu6_enableShadowTransfer(ccu6SFR, TRUE, FALSE);                  // 使能
}
void mos_close()
{
    ccu6SFR->MODCTR.B.T12MODEN = 0x2A;

    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_0, 0);
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_1, 0);
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_2, 0);
    IfxCcu6_enableShadowTransfer(ccu6SFR, TRUE, FALSE);
}
