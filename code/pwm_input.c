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
 * 文件名称          pwm_input
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

#include "ifxGtm_Tim.h"
#include "ccu6_pwm.h"
#include "pwm_input.h"

uint16 pwm_in_duty;
uint16 pwm_in_speed;
IfxGtm_Tim_In driver;
IfxGtm_Tim_In driver_back;
IfxGtm_Tim_In_Config config;
IfxGtm_Tim_In_Config config_back;
uint16 i;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     输入捕获初始化
// 参数说明
// 参数说明
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void pwm_input_init(void)
{
    IfxGtm_enable(&MODULE_GTM);

    if (!(MODULE_GTM.CMU.CLK_EN.U & 0x2))
    {
        IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, (float)FCY);
        IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK1);
    }

    IfxGtm_Tim_In_initConfig(&config, &MODULE_GTM);
    config.timIndex = IfxGtm_Tim_1;
    config.channelIndex = IfxGtm_Tim_Ch_6;
    config.isrPriority = GTM_PWM_IN_PRIORITY;
    config.capture.irqOnNewVal = TRUE;
    config.capture.irqOnCntOverflow = TRUE;
    config.timeout.clock = IfxGtm_Cmu_Clk_0;
    config.filter.inputPin = &IfxGtm_TIM1_6_TIN6_P02_6_IN;
    config.filter.inputPinMode = IfxPort_InputMode_pullDown;
    IfxGtm_Tim_In_init(&driver, &config);
    driver.periodTick = FPWM;

    gpio_init(MOTOR_DIR_IN_PIN, GPI, 0, GPI_PULL_DOWN); // 初始化方向设置引脚
}
